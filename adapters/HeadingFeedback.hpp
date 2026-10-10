#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace z {
// Command-space outer loop; radians, rad/s, seconds. No motor/ONNX changes.
template<class Scalar> class HeadingFeedback {
public:
    bool enabled = false;
    Scalar kp = Scalar(.5), limit = Scalar(.2);
    Scalar target = 0, error = 0;
    void Reset() { initialized_ = false; }
    static Scalar Wrap(Scalar x) { return std::atan2(std::sin(x), std::cos(x)); }
    Scalar Update(Scalar yaw, Scalar user_rate, Scalar dt) {
        if (!std::isfinite(yaw) || !std::isfinite(user_rate) ||
            !std::isfinite(dt) || dt <= 0)
            throw std::runtime_error("Invalid heading feedback input");
        if (!enabled) {
            target = yaw; error = 0; initialized_ = false;
            return user_rate; // exact legacy behavior when disabled
        }
        if (!initialized_) { target = yaw; initialized_ = true; }
        error = Wrap(target - yaw);
        const Scalar output = std::clamp(user_rate + kp * error, -limit, limit);
        // Advance reference to the next policy instant using ONLY user intent.
        target = Wrap(target + user_rate * dt);
        return output;
    }
private:
    bool initialized_ = false;
};
}
