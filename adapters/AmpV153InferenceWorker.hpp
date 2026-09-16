#pragma once

#include "Workers/NN/CommonLocoInferenceWorker.hpp"
#include "Workers/NN/NetInferenceWorker.h"
#include "Utils/ZenBuffer.hpp"
#include "Utils/StaticStringUtils.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace z {
// FrameworkToActor[f] is the actor joint index for framework slot f.
// Keep the original device mapping and motor bus unchanged. Only this adapter
// converts q/dq into actor order and q_target back into framework order.
template<typename SchedulerType, CTString NetName, typename Scalar,
         auto FrameworkToActor>
class AmpV153InferenceWorker
    : public CommonLocoInferenceWorker<SchedulerType, NetName, Scalar, 29>
{
    using Base = CommonLocoInferenceWorker<SchedulerType, NetName, Scalar, 29>;
    using Joints = math::Vector<Scalar, 29>;
    using Vec3 = math::Vector<Scalar, 3>;
    using Frame = math::Vector<Scalar, 98>;
    static constexpr size_t HistoryLength = 15;
    static_assert(FrameworkToActor.size() == 29);

public:
    AmpV153InferenceWorker(typename SchedulerType::Ptr scheduler,
                          const nlohmann::json& net_cfg,
                          const nlohmann::json& motor_cfg)
        : Base(scheduler, net_cfg, motor_cfg), history_(HistoryLength)
    {
        std::array<bool, 29> seen{};
        for (size_t f = 0; f < 29; ++f) {
            const size_t a = FrameworkToActor[f];
            if (a >= 29 || seen[a]) throw std::runtime_error("AMP joint mapping must be a permutation of 0..28");
            seen[a] = true;
            actor_to_framework_[a] = f;
        }
        const auto& pre = net_cfg["Preprocess"];
        policy_dt_ = pre["PolicyDt"].template get<Scalar>();
        cycle_time_ = net_cfg["Network"]["Cycle_time"].template get<Scalar>();
        if (!(policy_dt_ > 0) || !(cycle_time_ > 0))
            throw std::runtime_error("AMP timing must be positive");
        if (pre.value("UseAlterImu", false))
            throw std::runtime_error("AMP v153 adapter requires the main pelvis IMU");
        const auto& scales = pre["CommandScales"];
        if (scales.size() != 3) throw std::runtime_error("AMP requires three command scales");
        for (size_t i = 0; i < 3; ++i) command_scales_[i] = scales[i].template get<Scalar>();
        clip_targets_ = net_cfg["Postprocess"].value("clip_joint_targets", false);
        this->InputOrtTensors__.push_back(this->WarpOrtTensor(input_));
        this->OutputOrtTensors__.push_back(this->WarpOrtTensor(output_));
    }

    void PreProcess() override
    {
        start_ = std::chrono::steady_clock::now();
        Joints q, dq, previous_action;
        Vec3 command, gyro, rpy;
        this->Scheduler->template GetData<"CurrentMotorPosition">(q);
        this->Scheduler->template GetData<"CurrentMotorVelocity">(dq);
        this->Scheduler->template GetData<concat(NetName, "NetLastAction")>(previous_action);
        this->Scheduler->template GetData<concat(NetName, "NetUserCommand3")>(command);
        this->Scheduler->template GetData<"AngleVelocityValue">(gyro);
        this->Scheduler->template GetData<"AngleValue">(rpy);
        // RPY is in radians, body-to-world R=Rz(yaw)Ry(pitch)Rx(roll).
        // The actor receives R^T*[0,0,-1], never the Euler angles themselves.
        const Vec3 gravity = ComputeProjectedGravity(rpy);
        this->Scheduler->template SetData<concat(NetName, "NetProjectedGravity")>(gravity);
        const Scalar angle = static_cast<Scalar>(2.0 * 3.14159265358979323846)
            * static_cast<Scalar>(policy_step_) * policy_dt_ / cycle_time_;
        Frame frame;
        frame[0] = std::sin(angle);
        frame[1] = std::cos(angle);
        for (size_t i = 0; i < 3; ++i) frame[2 + i] = command[i] * command_scales_[i];
        for (size_t a = 0; a < 29; ++a) {
            const size_t f = actor_to_framework_[a];
            frame[5 + a] = (q[f] - this->JointDefaultPos[f]) * this->Scales_dof_pos[f];
            frame[34 + a] = dq[f] * this->Scales_dof_vel[f];
            // NetLastAction is deliberately in actor order, before action_scale.
            frame[63 + a] = initialized_ ? previous_action[a] : Scalar(0);
        }
        for (size_t i = 0; i < 3; ++i) {
            frame[92 + i] = gyro[i] * this->Scales_ang_vel[i];
            frame[95 + i] = gravity[i] * this->Scales_project_gravity[i];
        }
        if (!initialized_) {
            for (size_t i = 0; i < HistoryLength; ++i) history_.push(frame);
            initialized_ = true;
        } else history_.push(frame);
        math::Vector<Scalar, 1470> obs;
        for (size_t i = 0; i < HistoryLength; ++i)
            std::copy(history_[i].begin(), history_[i].end(), obs.begin() + 98 * i);
        input_.Array() = decltype(obs)::clamp(obs, -this->ClipObservation, this->ClipObservation);
        ++policy_step_;
    }

    void PostProcess() override
    {
        const auto action = Joints::clamp(output_.toVector(), -this->ClipAction, this->ClipAction);
        this->Scheduler->template SetData<concat(NetName, "NetLastAction")>(action);
        Joints target;
        for (size_t f = 0; f < 29; ++f)
            target[f] = this->JointDefaultPos[f] + this->ActionScale[f] * action[FrameworkToActor[f]];
        if (clip_targets_) target = Joints::clamp(target, this->JointClipLower, this->JointClipUpper);
        this->Scheduler->template SetData<concat(NetName, "Action")>(target);
        const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_).count();
        this->Scheduler->template SetData<concat(NetName, "InferenceTime")>(static_cast<Scalar>(elapsed));
    }

private:
    math::Tensor<Scalar, 1, 1470> input_;
    math::Tensor<Scalar, 1, 29> output_;
    RingBuffer<Frame> history_;
    std::array<size_t, 29> actor_to_framework_{};
    Vec3 command_scales_;
    Scalar policy_dt_ = Scalar(0.01), cycle_time_ = Scalar(0.64);
    size_t policy_step_ = 0;
    bool initialized_ = false, clip_targets_ = false;
    std::chrono::steady_clock::time_point start_;
};
} // namespace z
