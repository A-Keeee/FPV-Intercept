# uav_rl_guidance

基于**强化学习 (GRU 策略)** 的视觉制导拦截节点，是 `uav_vision_png` 的**即插即用替代方案**。

## 核心思路

传统方案使用比例导引（PNG）算法计算拦截轨迹。本方案使用**离线训练 → 在线推理**的强化学习策略取代 PNG：

```
/camera_detect_result  →  15维观测特征  →  GRU策略 (ONNX Runtime)  →  速度指令
                                                        ↓ (异常时)
                                                 内置 PNG 回退控制器
```

- **策略**：GRU 循环神经网络（BC + PPO 训练），输入 bbox 视觉特征 + 自身状态，输出 NED 速度指令
- **推理**：ONNX Runtime，零 PyTorch 依赖，纯 C++ 实现
- **安全兜底**：策略异常时自动回退到内置 PNG 控制器（watchdog 机制）
- **A/B 对比**：`fallback_png:=true` 可切换为纯 PNG 基线模式

## 与 uav_vision_png 的关系

| | uav_vision_png | uav_rl_guidance |
|---|---|---|
| 制导算法 | 比例导引 (PNG) | GRU 强化学习策略 |
| 话题接口 | 完全相同 | ← 兼容 |
| 状态机 | TAKE_OFF → SEARCHING → INTERCEPT → ... | ← 完全一致 |
| CSV 格式 | 完全相同列 | ← 兼容 |
| 实现语言 | C++ | C++ |
| 模型依赖 | 无 | ONNX Runtime (~15MB) |

## 文件结构

```
uav_rl_guidance/
├── CMakeLists.txt                  # ament_cmake 构建
├── package.xml
├── README.md
├── config/
│   └── params.yaml                 # 参数配置（模型路径、PNG 增益、丢失阈值等）
├── launch/
│   └── rl_guidance.launch.py       # 启动文件
├── models/
│   ├── policy.onnx                 # ONNX 推理模型（运行时加载）
│   ├── policy.pt                   # TorchScript 模型（用于重新导出 ONNX）
│   └── policy_meta.json            # 模型元信息（动作解码参数等）
├── include/uav_rl_guidance/
│   └── rl_guidance_node.hpp        # C++ 类声明
└── src/
    ├── rl_guidance_node.cpp        # C++ 完整实现
    └── export_onnx.py              # ONNX 导出脚本（模型更新时使用）
```

## 依赖

| 依赖 | 用途 |
|---|---|
| ONNX Runtime | GRU 策略推理 |
| Eigen3 | 几何运算（姿态旋转、LOS 计算） |
| GeographicLib | GPS→NED 坐标变换（统计用） |
| rclcpp / px4_msgs / uav_common_msg | ROS 2 通信 |

## 启动方式

```bash
# 标准启动（加载 ONNX 策略）
ros2 launch uav_rl_guidance rl_guidance.launch.py

# 纯 PNG 基线模式（A/B 对比，不加载策略）
ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

# 台架调试（跳过起飞，直接进入 SEARCHING）
ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
```

## 主要参数

最终速度指令统一通过 50 Hz 限幅器，RL 策略、PNG/watchdog 回退、续飞、减速、
重捕获与 DONE 共用同一状态。默认 `max_speed=3.0 m/s`，水平速度矢量变化率
`max_accel_xy=0.8 m/s²`，垂直变化率 `max_accel_z=0.6 m/s²`，
垂直速度绝对值 `max_vertical_speed=1.0 m/s`；偏航角速度上限为
`max_yaw_rate=0.8 rad/s`，变化率 `max_yaw_accel=1.0 rad/s²`。
`max_vertical_correction=0.8 m/s` 限制 PNG 垂直视场补偿，补偿后还受总速度限制。

`speed_cmd=5.0` 保留模型训练时的原始动作解码范围；调低实际飞行指令速度请使用
`max_speed`。它可以小于动作解码的 2 m/s 最小值，最终输出仍受限。
例如可直接用 launch 参数选择更缓和的速度和加速过程：

```bash
ros2 launch uav_rl_guidance rl_guidance.launch.py max_speed:=2.5 max_accel_xy:=0.6
```

进入速度控制时从实测速度初始化，已超限的已有速度会逐步降回上限。
短暂丢失保持实际已发布速度，达到 `coast_thresh` 后平滑减速，失效期间暂停策略更新。
这些限制约束发送给 PX4 的速度/偏航指令；实际加速度和倾角由 PX4 内环决定，
需要通过 Gazebo 姿态和轨迹验证，不能据此保证实际倾角的硬上限。

记录方式与视觉 PNG 一致：默认 `record_csv=false`，每次运行使用独立的 ROS 日志文件，
每秒记录一次 `[STATS] event=progress`，命中时立即记录 `event=hit state=DONE`，
正常退出时输出 `event=summary hit=true/false`。日志包含 GPS 距离、最近接距离、
源检测结果/丢失结果计数，以及 RL 特有的 `cmd_source` 和 `watchdog_total`。
`cmd_source` 为 `policy`、`png` 或 `png_watchdog`。

成功判定独立于策略推理和记录，在 50 Hz 主控制循环中运行；短暂丢失续飞、
减速和 TRACK_LOST 都继续检查。检测话题完全断流超过 `detection_timeout=0.5` 秒时
进入 TRACK_LOST，重新检测到目标后重置 GRU/PNG 状态并恢复 INTERCEPT。
成功条件仍为两机 GPS 换算距离小于 `hit_radius`，不等同于 Gazebo 接触事件。

日志默认保存在 `~/.ros/log/`，节点文件名包含进程号和时间戳；可指定工作区目录：

```bash
source install/setup.bash
ROS_LOG_DIR="$PWD/log/rl_guidance" ros2 launch uav_rl_guidance rl_guidance.launch.py
```

CSV 为可选的 10 Hz 导出，不读取旧内容，开启时覆盖指定文件。
打开或写入失败只输出一次错误并停止 CSV 导出，仍继续成功判定和 ROS 日志记录：

```bash
ros2 launch uav_rl_guidance rl_guidance.launch.py record_csv:=true csv_path:=/tmp/rl_experiment.csv
```

CSV 记录回调与控制回调使用同一执行器，慢磁盘仍可能拖延回调，默认关闭。
需要完整数据用于绘图或回放时可另开终端录制 rosbag：

```bash
ros2 bag record /rosout /camera_detect_result /vpng_data \
  /px4_1/fmu/out/vehicle_odometry /px4_1/fmu/out/vehicle_local_position \
  /px4_1/fmu/out/vehicle_gps_position /px4_2/fmu/out/vehicle_gps_position \
  /px4_1/fmu/in/trajectory_setpoint
```

launch 默认读取 YAML，仅显式传入的 `fallback_png`、`bench_test`、`record_csv`、
`csv_path` 和速度/加速度限幅参数覆盖 YAML；可通过
`params_file:=/absolute/path/params.yaml` 选择配置。

集成测试使用独立 ROS domain 和模拟 PX4 输入，覆盖实际 ONNX 策略、PNG 基线、
只读/写入失败的 CSV、默认日志落盘、视觉丢失/断流后的成功判定、可选 CSV 导出，
以及转向、丢失续飞/减速、恢复和 DONE 之间的速度/加速度指令限制：

```bash
colcon build --packages-select uav_rl_guidance --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select uav_rl_guidance --ctest-args -R 'test_statistics|test_setpoint_limiter' --output-on-failure
colcon test-result --test-result-base build/uav_rl_guidance --verbose
```

参见 `config/params.yaml`，关键参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `model_path` | `models/policy.onnx` | ONNX 模型路径 |
| `fallback_png` | `false` | 纯 PNG 基线模式 |
| `bench_test` | `false` | 跳过起飞（台架调试） |
| `standby_altitude` | `-6.0` | 待机高度（NED，负值向上） |
| `speed_cmd` | `5.0` | 原始动作解码速度上限；最终指令受 `max_speed` 限制 |
| `kv` / `kz` | `4.0` | PNG 俯仰/方位增益 |
| `coast_thresh` | `30` | 惯性续飞帧数 |
| `lost_thresh` | `90` | 目标丢失切换 TRACK_LOST 帧数 |
| `hit_radius` | `0.8` | 命中判定半径 (m) |

## 模型更新

配套训练项目为同级目录 [End2End-FPVIntercept](../../End2End-FPVIntercept/README.zh-CN.md)，
Python 包名为 `aerointercept`。本节点使用 15 维观测输入、128 维 GRU 隐藏状态的策略；
更新兼容的 `models/policy.pt` 后，重新导出 ONNX：

```bash
cd uav_rl_guidance
python3 src/export_onnx.py
```

该脚本读取 `models/policy.pt`，生成 `models/policy.onnx`。
端到端图像策略的输入与该节点不同，部署方式见训练项目说明。

## Watchdog 安全机制

RL 策略输出异常时自动回退：

| 触发条件 | 处理 |
|---|---|
| 策略输出 NaN/Inf | 回退 PNG，连续 20 帧（1s），重置 GRU 隐藏状态 |
| 速度指令 > 1.5×speed_cmd | 同上 |
| 推理异常（ONNX 报错等） | 同上 |

回退期间 PNG 控制器接管，策略隐藏状态保留。20 帧后自动恢复策略推理。
