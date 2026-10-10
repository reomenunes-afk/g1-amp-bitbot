# G1 AMP 模型与航向反馈部署说明

## 本次部署范围

本次新增 11 个经过用户视觉筛选的行走模型：

- v252_3000.onnx
- v254_2800.onnx
- v257_2200.onnx
- v301_2400.onnx
- v308_2900.onnx
- v309_2700.onnx
- v311_2200.onnx
- v313_2900.onnx
- v324_3000.onnx
- v326_2500.onnx
- v337_3000.onnx

文件位于 CheckPoints/，默认选择 v311_2200.onnx。它们均使用 float32 输入 input [1,1470]、输出 output [1,29]，15 帧历史，每帧 98 维；步态周期 1.295 秒，策略周期 0.01 秒（100 Hz）。不需要上传训练用专家 npz 或判别器。

## 从 v153 切换

MuJoCo 和真机均由 user_func.cpp 读取项目 settings/CtrlConfig.json。只修改 Workers.WalkNet1 中对应字段，勿修改 DanceNet1 的周期和节点名：

```json
"Network": {
  "ModelPath": "CheckPoints/v311_2200.onnx",
  "Cycle_time": 1.295,
  "InputNodeNames": ["input"],
  "OutputNodeNames": ["output"]
}
```

WalkNet1.Preprocess 保留 PolicyDt=0.01、UseAlterImu=false，并配置：

```json
"HeadingFeedback": {
  "Enabled": true,
  "Kp": 0.5,
  "YawRateLimit": 0.2
}
```

其他关节映射、默认位置、观测和动作缩放、PD 参数不因航向反馈改变。真机电脑若有单独调过的配置，应先比较再合并，不要覆盖其设备/网卡配置。退回旧 v153 模型时，需同时恢复其 Cycle_time=0.64 和 obs/actions 节点名。

当前程序启动还会构造舞蹈网络，必须保留 DanceNet1.ModelPath 指定的 policy_gangnam.onnx，即使本次只测试行走。不得因清理候选行走模型而删除它。

## 纠偏原理与数据来源

纠偏在 AmpV153InferenceWorker::PreProcess 中、命令缩放之前执行，不修改 ONNX、关节映射或电机接口。真机读取 LowState 中主 IMU 的 rpy[2]，经 UnitreeImu::GetYaw() 与 ImuProcessWorker 写入 AngleValue[2]。不使用仿真世界位置、动捕或实际线速度。

进入行走事件请求清除旧目标；下一次行走推理用当时 yaw 初始化目标。进入初始化姿态或舞蹈事件关闭纠偏。每次策略推理：

1. error = atan2(sin(target-yaw), cos(target-yaw))。
2. policy_yaw_rate = clip(user_yaw_rate + Kp*error, -YawRateLimit, YawRateLimit)。
3. target = wrap(target + user_yaw_rate*PolicyDt)，推进至下次推理时刻。
4. vx/vy 不变；原始 NetUserCommand3 不被覆盖。

YawRateLimit=0.2 限制的是最终用户角速度与纠偏之和，单位 rad/s，不只是纠偏增量。零角速度指令表示保持已锁定的目标方向。Enabled=false 时直接传递原始用户指令。

这是比例航向反馈，无积分项；可能存在稳态误差，也不能消除 IMU 自身航向漂移。直行比例增益与既有 IsaacGym 播放相同，但 IsaacGym 播放限幅为 1 rad/s，部署为 0.2 rad/s；不可视为完全相同的闭环。

## 初始化与 MuJoCo Reset

当前代码没有直立检测或 IMU 数据新鲜度检测。应在机器人直立、IMU 数据已正常更新时进入行走。不要在倒地状态锁定目标：大倾角下欧拉 yaw 可能跳变约 180°。

MuJoCo 界面 Reset 只重置仿真，不通知本纠偏器；不会自动清除目标方向。重置后在直立状态重新触发行走事件（r）以锁定当前方向。纠偏目标重置并不等同于清空策略历史或相位。

## 文件依赖

- adapters/HeadingFeedback.hpp：航向反馈核心，新文件。
- adapters/AmpV153InferenceWorker.hpp：读取配置、处理角速度指令与记录日志。
- types.hpp：增加调度器及日志字段。
- user_command.cpp：行走/初始化/舞蹈事件控制目标重置与启停。
- settings/CtrlConfig.json：选择模型与反馈参数。
- tests/heading_feedback_test.cpp：独立逻辑测试，不连接机器人或控制电机。

首次同步 C++ 改动必须重新编译。之后仅修改同接口模型或 JSON 参数，重启程序即可读取。

## 在真机电脑检查与编译

先备份本机已有改动，确认更新的提交。保留真机 settings/bitbot_unitree.xml 中已经验证的网卡、设备映射和通信配置。

项目根目录运行独立测试：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic tests/heading_feedback_test.cpp -o /tmp/g1_heading_feedback_test
/tmp/g1_heading_feedback_test
```

真机编译使用独立目录，避免误用仿真构建：

```bash
cmake -S . -B build_real -DBUILD_SIMULATION=OFF
cmake --build build_real -j8
```

产物为 build_real/bin/main_app；BUILD_SIMULATION=OFF 选择 settings/bitbot_unitree.xml。MuJoCo 使用 BUILD_SIMULATION=ON，选择 settings/bitbot_mujoco.xml。两者共享 CtrlConfig.json。PROJECT_ROOT_DIR 在编译时确定，应在部署电脑重新编译，不复制本机 build 二进制。

启动前确认 ModelPath 文件存在，节点名、输入输出维度、步态周期、策略周期一致。启动电机控制由现场用户执行，代码审阅或测试不意味着授权自动运行真机程序。

## 日志与验证状态

日志新增：

- WalkNet1HeadingStatus：当前目标 yaw、实际 yaw、wrap 后误差，单位 rad。
- WalkNet1PolicyCommand3：送入观测缩放之前的最终 vx、vy、角速度。
- WalkNet1NetUserCommand3：用户原始指令。

单元测试覆盖关闭直通、纠偏方向、限幅、跨 ±π、只用用户角速度推进目标、重入模式、理想角速度跟踪系统收敛和非有限输入。这只验证反馈算法，不验证真机传感器、实时性或步态稳定性。

用户已在 MuJoCo 中观察到纠偏有效；已知 Reset/倒地初始化问题如上。尚未完成本次新模型及纠偏的真机行走验证。首次真机测试应重点核对 IMU yaw 更新与方向、初始化目标、纠偏符号及漂移。
