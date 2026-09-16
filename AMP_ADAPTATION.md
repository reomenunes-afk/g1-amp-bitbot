# G1 AMP v153：bitbot-unitree / Codex 独立适配

## 基线及权限边界

- 上游：https://github.com/ZzzzzzS/bitbot-unitree.git
- 主仓库基线：`b7833b7b52ac979b5f17cd61dfad64ef48b473ba`
- CtrlZ 子模块：`9d71e9386ed886c5832d63ec4602954fa9e9d111`
- 本次只修改配置、添加推理适配和仿真资产。没有编译、加载 ONNX 推理、启动仿真或真机。
- 没有添加自动上电、自动 START、自动复位或自动行走配置。
- `main.cpp`、`user_func.cpp`、`user_command.*`、`Bitbot_Unitree/`、真机 XML、按键映射、CMakeLists.txt 和 CtrlZ 子模块均与基线一致。
- 后续克隆须使用 `git clone --recursive`，不能遗漏 CtrlZ 子模块。

## 修改清单

| 文件 | 用途 |
|---|---|
| `adapters/AmpV153InferenceWorker.hpp` | 专用 Actor 观测/动作适配；不修改公共 Worker |
| `types.hpp` | 仅新增 include 和选择 AMP Worker；原关节映射完全保留 |
| `settings/CtrlConfig.json` | AMP 配置、策略分频、框架顺序参数、零默认指令、相对资源路径 |
| `settings/bitbot_mujoco.xml` | 指向 AMP 平地场景，匹配初始关节角（度）和仿真 PD |
| `models/g1_amp_v153.xml` | 已有 v153 训练 URDF 派生的 MuJoCo 资产；骨盆自由根高度0.8 m |
| `models/scene_amp_v153.xml` | 方格平地，2 ms，implicitfast；不覆盖原场景 |
| `models/amp_meshes/` | 35个网格，与 v153 训练资产逐文件 SHA-256 比对一致 |
| `CheckPoints/amp_v153_projected_gravity_model_*.onnx` | 1400、2500、2800 三个既有导出模型 |

没有复制 HHD 的应用代码、深度任务或构建目录。MuJoCo 模型/网格和 ONNX 属于同一策略的共享资产，不是另一套适配器实现。

## 关节顺序：本仓库不能照搬 HHD 的恒等映射

原版 `JOINT_ID_MAP` 保持不变：

```
框架槽位 f → 设备ID / v153 Actor索引 a
[0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28]
```

已核对仿真设备ID 0..28对应 v153 的左腿、右腿、腰、左臂、右臂顺序。

- CurrentMotorPosition/Velocity：原框架顺序。
- DefaultPosition、WalkNet1 的 Kp/Kd、action_scale、目标角上下限：原框架顺序。
- Actor 的98维观测及29维原始动作：v153 训练顺序。
- WalkNet1NetLastAction：Actor 顺序、裁剪后且未乘 action_scale。
- WalkNet1Action：转换回框架顺序的目标角。
- 不改变 SDK motor 索引、设备数组或公共 MotorWorker。

## 模型与训练来源

训练工作树：`/home/hyk/humanoid-gym/.worktrees/v153-actor-projected-gravity`

训练 commit：`6cdf195f0e7d1498f26ef38255d00270fe80d0df`

任务：`G1_amp_kit_command_matched_fingertip_symmetry003`

训练目录：`logs/amp_ppo/Sep14_01-21-46_amp_v153_actor_projected_gravity_local5080`

默认模型：`CheckPoints/amp_v153_projected_gravity_model_2500.onnx`

| 模型 | ONNX SHA-256 |
|---|---|
| 1400 | `1ffaefe6000951cd0514ab42618ccc0a5e70a6f6edc9ff67d87c72370fa589d9` |
| 2500 | `eebfde9bd1627b1962d938c33734439b2c623d16f6603bdddb35fb5c21011260` |
| 2800 | `26657a4ceb969f5d44b449a0fbfed668f2f675b46c3aa1a940286581ab517394` |

MLP：1470→512→256→128→29，隐藏层 ELU。ONNX float32、opset17，obs→actions，动态 batch，部署 batch=1。

每帧98维：sin/cos相位2 + 缩放指令3 + q-default 29 + dq*0.05 29 + previous action29 + body gyro3 + projected gravity3。

历史15帧，oldest→newest；首帧重复15次，首次 last action 为零；obs/action clip±18。相位周期0.64 s，按策略执行次数×0.01 s计算。

主 IMU RPY（弧度）通过原版 ComputeProjectedGravity 转换成机体坐标系下的单位重力方向，Actor 不直接接收 Euler。不读取额外线速度、深度、Estimator、AMP判别器或专家动作。

q_target[f] = DefaultPosition[f] + action_scale[f] * action[JOINT_ID_MAP[f]]。
默认不额外裁剪目标角，不主动加入训练时的随机动作混合或噪声。训练限制不代表真机安全认证。

## 启动、复位和共享参数的影响

- 原版9/8/p/r及停止事件未修改；由用户操作，不自动发送事件。
- `Scheduler.dt=0.002`保持原值，`PolicyFrequency`由10改5，策略100 Hz；AMP PolicyDt=0.01。
- 该配置项在原程序也供 DanceInferTask 使用，因此舞蹈任务分频也随之变为5。没有宣称舞蹈功能通过验证。
- 原 `ResetTask` 分频10、复位算法、Duration=3保持不变。
- DefaultPosition 已按原框架顺序改为 AMP 姿态，因此 p 的复位目标会变化；这是一项明确的配置影响。
- 真机 XML 中初始增益、网络接口、500 Hz BusFrequency均未修改。WalkNet1配置中的 AMP 增益在原 ActionManager 切换规则下使用，不能视为所有启动阶段都使用这些增益。
- 原 r 回调会同时启用两套推理任务，并在3秒后关闭舞蹈任务，保留原版动作切换机制。不得将其误认为本次新增的自动启动；任何修改需单独批准。
- 默认速度[0,0,0]，vx范围[-0.3,0.6]，vy=0，yaw-rate范围[-0.2,0.2]；零指令不保证物理上完全静止。
- 相位和历史在新建 Worker 后首次推理初始化，不保证暂停、查看器 Reset 或重复切换策略会重新初始化；干净重复试验请退出重启。
- 仿真主、副IMU站点目前均在骨盆，仅使用主IMU。副IMU并不是实机胸部传感器的物理复现。

## 编译、运行与迁移（由用户执行）

本次未创建机器专用 VS Code 设置或自动运行脚本。请先选择当前电脑上的编译器，配置匹配架构的 ONNX Runtime 和依赖路径，再手动编译。

- 仿真：BUILD_SIMULATION=ON；真机：使用独立构建目录和目标机器依赖，BUILD_SIMULATION=OFF。
- 默认资源路径相对于**仓库根目录**。VS Code 启动 cwd 必须是本仓库根目录。
- 若使用默认 build 目录，编译完成后的应用路径通常是 `build/bin/main_app`；本次没有生成或运行它。
- 不使用旧 HHD 的运行脚本或可执行文件。
- 不携带 x86-64 CMakeCache、二进制或 .build-env 到 ARM64机器冒充本机依赖。
- 原舞蹈模型和CSV仍在初始化中加载，本次只把有效路径改为仓库根目录相对路径，没有删除原逻辑。
- 每次测试由用户提供编译末尾日志、实际启动cwd、加载的模型路径、按键后的状态和机器人表现。
- 若真机按 p 不复位，停在此环节查事件/状态、复位任务、目标角与实际角，不继续按 r。

## 本次验证范围

已完成只读/静态检查：Git受保护文件无差异；子模块干净；JSON/XML语法；29关节映射及配置重排；默认姿态单位；PD；传感器名称；500/100 Hz关系；零命令；35网格与训练源一致；三个ONNX哈希一致。

**没有编译、ONNX推理、MuJoCo载入/运行或真机测试，不能据此称为 sim-to-sim 已通过。**
