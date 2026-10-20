# FPV-Intercept
### 基于视觉伺服的自主拦截无人机系统（PX4 / ROS 2）

<div align="center">


</div>

<div align="center">


</div>

## 项目来源与原作者说明

本项目基于 [Eaglewzw（Verse）](https://github.com/Eaglewzw) 的
[Autonomous-Intercept-Drone](https://github.com/Eaglewzw/Autonomous-Intercept-Drone)
进行本机环境适配与修改，感谢原作者及贡献者的项目实现与资料分享。
以下引用原项目 README 中的说明：

> 本项目是一个面向自主无人机拦截任务的实验验证平台，集成了视觉感知、目标检测、比例导引（PNG）算法及飞行控制方法。该项目旨在为无人机自主拦截、制导算法研究及视觉伺服控制提供一套完整的仿真参考方案，适用于学习与学术研究。
>
> —— [原作者项目说明（中文 README）](https://github.com/Eaglewzw/Autonomous-Intercept-Drone/blob/main/README.zh-CN.md)

本文中的原项目介绍、算法说明及演示资料引用自上游仓库；本机安装配置、
PX4 / ROS 2 接口适配、一键启动与就绪检查等内容为当前版本的补充。

---

## 项目简介

当前项目目录为 `FPV-Intercept/`，对应 PX4 / ROS 2 视觉制导与仿真工作空间。
同级目录 [End2End-FPVIntercept](../End2End-FPVIntercept/README.zh-CN.md)
提供 Python 策略训练与端到端视觉拦截实验，具体训练及模型接口以该项目说明为准。

该系统在 ROS 2 环境下开发，结合 PX4 固件与 Gazebo 仿真器，实现了从目标发现到仿真拦截的全链路闭环。主要功能包括：

*   **视觉目标检测**：利用训练的 YOLO 模型和跟踪模型（LightTrack）实时识别并跟踪视场内的无人机目标。
*   **比例导引制导**：基于比例导引律（Proportional Navigation Guidance, PNG）生成拦截轨迹，实现对动态目标的精确前置拦截。
*   **仿真验证体系**：基于 PX4 软件在环（SITL）搭建，支持在 Gazebo 环境中进行算法的离线验证与参数调优。
*   **数据分析工具**：内置运动学与视觉指标记录脚本，用于定量评估制导律精度与视觉追踪稳定性。

---

## 快速上手

### 本机安装（Ubuntu 22.04 / 用户 ake）

本仓库根目录 `FPV-Intercept` 就是本机的 ROS 2 工作空间，
各功能包直接位于根目录下。安装脚本在这里生成 `build/`、`install/`、`log/`，
并使用根目录中的 `px4_msgs/`，无需另建工作空间或 `src/` 符号链接。
本机模型及 CSV 默认路径指向当前仓库目录。
PX4 使用 `v1.16.0`，`px4_msgs` 使用 `release/1.16`，状态订阅使用
`/px4_1/fmu/out/vehicle_status_v1`。

在仓库根目录依次执行：

```bash
cd "/home/ake/SJTU/graduation project/FPV-Intercept"

# 安装系统包，替换旧 Gazebo/ros_gz，并安装 CUDA 13 的 ONNX Runtime C++ 库。
sudo bash scripts/install_system_dependencies.sh

# 以普通用户安装源码依赖、编译消息及功能包、构建 PX4 并复制仿真资源。
bash scripts/setup_workspace.sh
```

`setup_workspace.sh` 默认使用两个编译任务，可用 `AID_BUILD_JOBS` 调整。
MicroXRCEAgent v2.4.3 安装到用户目录，其依赖库仅对 Agent 进程生效。
已有 ONNX 模型可直接运行，无需安装 PyTorch 或 TensorRT。

每个运行终端先在本仓库根目录执行 `source ./env.sh`。随后启动
`~/PX4-Autopilot/run_swarm.sh`，打开 QGroundControl 并等待两机连接，再启动目标机节点，并从无视觉 PNG、视觉 PNG、RL
中选择一个控制节点；视觉 PNG 与 RL 另需运行检测节点。三个控制节点不要同时启动。

安装脚本完成仅表示依赖与编译成功；还应按下文检查双机话题、相机图像、
模型加载日志及 CSV 输出。

### 1. 环境依赖
*   **ROS 2 Version**: Humble
*   **Gazebo**:Gazebo Harmonic（gz-sim 8.12.0）
*   **PX4 Firmware**: v1.16 (**Micro-XRCE-DDS-Agent**版本推荐v2.4.x)
*   **地面站**: QGroundControl 5.0.8（支持 Ubuntu 22.04），运行期间保持连接
*   **Dependencies**: OpenCV, onnx, colcon

### 2. 编译项目
```bash
# 在本仓库根目录（即工作空间）编译
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

视觉 PNG 的 `hover_ok` 依赖 PX4 悬停推力估计。`scripts/setup_workspace.sh`
会在编译 PX4 前自动导出该话题。已有安装若缺少此话题，先停止仿真，在
`~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` 的 `publications:`
下补充以下条目（已存在则无需重复添加），再单独重新编译 PX4：

```yaml
  - topic: /fmu/out/hover_thrust_estimate
    type: px4_msgs::msg::HoverThrustEstimate
```

```bash
make -C ~/PX4-Autopilot -j2 px4_sitl_default
```

重新启动仿真并起飞后，在已执行 `source ./env.sh` 的终端验证：

```bash
ros2 topic echo /px4_1/fmu/out/hover_thrust_estimate \
  px4_msgs/msg/HoverThrustEstimate --qos-reliability best_effort --once
```

### 3. 运行拦截任务 (步骤序列)
使用 RL 方案时，在本仓库根目录一键启动完整仿真：

```bash
./start.sh
# 可选：将参数传给 RL launch，例如使用 PNG 基线
./start.sh fallback_png:=true
```

脚本自动加载 `env.sh`，在 `~/PX4-Autopilot` 中启动 `run_swarm.sh`。
确认 Gazebo 状态、双机 PX4 消息及相机图像可用后，才启动 QGroundControl；
确认双机连接地面站、本地位置和速度估计有效后，依次启动目标机、视觉检测和 RL 制导。
后续分别检查目标机解锁并进入 Offboard、检测模型加载成功且发布检测消息、
RL 发布控制心跳，每一步通过后才继续。启动过程按实际状态推进。
已运行的 PX4 仿真和地面站会被复用。日志位于
`log/start/<时间戳-PID>/`，按 Ctrl+C 统一停止本次启动的程序。
每阶段失败超时上限默认 30 秒，可用 `START_TIMEOUT=180 ./start.sh` 调整；
条件满足即继续执行。
PX4 目录可通过 `PX4_DIR=/path/to/PX4-Autopilot ./start.sh` 指定。

地面站连接检查使用 `gcs_connection_lost`，Offboard 位置/速度控制检查使用
`VehicleLocalPosition` 的 `xy_valid`、`z_valid`、`v_xy_valid`、`v_z_valid`。
`pre_flight_checks_pass` 对应当前飞行模式；初始 Hold 模式要求全局定位，
不能用它作为本项目本地 Offboard 控制的启动条件。PX4 仍在切换模式和解锁时
执行对应检查，脚本随后确认目标机已解锁并进入 Offboard。

需要分别启动时，请按以下顺序在不同终端中运行各节点：


0. **启动仿真环境**:
   启动前，需将本仓库的仿真资源复制到 PX4-Autopilot 目录下：

   ```bash
   # 步骤1: 将本仓库的 run_swarm.sh 复制到 PX4-Autopilot 根目录（若尚未复制）
   cp ./run_swarm.sh ~/PX4-Autopilot/

   # 步骤2: 将自定义世界文件复制到 PX4 Gazebo 世界目录
   cp ./assets/gazebo_world/grass_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
   ```

   > 以上步骤只需执行一次。之后每次启动仿真，直接进入 PX4-Autopilot 目录运行启动脚本即可：
   ```bash
   cd ~/PX4-Autopilot
   ./run_swarm.sh
   ```

   该脚本会自动启动 Gazebo 仿真服务器、两架无人机（px4_1 拦截机 + px4_2 目标机）、MicroXRCEAgent 以及相机话题桥接。

   **启动目标节点前，打开 QGroundControl 并等待两架飞机通过 UDP 14550 自动连接。**
   本机安装位置为：

   ```bash
   ~/Applications/QGroundControl-v5.0.8-x86_64.AppImage
   ```

   Ubuntu 22.04 使用 [QGroundControl 5.0 系列](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/download_and_install.html)。
   当前 PX4 配置要求地面站连接；否则 `drone2.log` 会报告
   `Preflight Fail: No connection to the ground control station` 并拒绝解锁。
   `Arm command sent` 仅表示目标节点发送了请求。目标节点仅请求一次解锁，
   若连接地面站前已经启动过该节点，连接成功后应在该节点终端按 Ctrl+C 并重新运行。


1. **启动目标机仿真**:
   ```bash
   # 默认圆周运动
   ros2 run uav_target_sim uav_target_sim

   # 或通过参数选择运动模式和活动范围
   ros2 run uav_target_sim uav_target_sim --ros-args -p motion_mode:=sinusoidal -p max_range:=10.0

   # 也可使用 launch 文件
   ros2 launch uav_target_sim target_sim.launch.py motion_mode:=random_walk
   ```

   **支持的目标运动模式**：

   | 参数值 | 运动模式 | 运动学公式 | 特点 |
   | :--- | :--- | :--- | :--- |
   | `circle` | 匀速圆周 | R=5m, w=0.5rad/s | 默认模式，半径5m匀速圆周运动 |
   | `sinusoidal` | 正弦机动 | a=0.5*sin(0.5t) m/s<sup>2</sup> | 前向匀速1m/s + 侧向正弦加速度，周期性规避 |
   | `random_walk` | 随机游走 | v(t+dt) = v(t) + N(0, 0.2) | 速度随机扰动，限速3m/s，无规律逃逸 |

   > `max_range` 参数（默认10m）控制目标活动范围，防止目标飞出拦截机视野。

2. **启动视觉制导拦截**:
   ```bash
   # 推荐直接运行（使用 .hpp 中的默认参数）
   ros2 run uav_vision_png uav_vision_png

   # 或通过 launch 文件加载 config/params.yaml 中的参数
   ros2 launch uav_vision_png vision_png.launch.py
   ```

   > **参数配置说明**：`uav_vision_png` 节点的参数优先级为：
   >   **launch 命令行 > params.yaml > .hpp 成员初始值**
   >
   > - 使用 `ros2 run` 直接启动：参数取 `.hpp` 中的默认值，**`params.yaml` 不会自动加载**，需手动传参 `--ros-args -p 参数名:=值`
   > - 使用 `ros2 launch` 启动：launch 文件自动加载 `config/params.yaml`，YAML 中的值会覆盖 `.hpp` 默认值
   > - 调试时可用 `ros2 param get /uav_vision_png <参数名>` 查看当前实际生效的值
   > - 可调参数详见 `uav_vision_png/config/params.yaml`，含 PNG 增益、视场补偿、起飞高度等

2-备. **启动 RL 学习制导拦截 (替代方案)** 🆕:
   `uav_rl_guidance` 是基于强化学习（BC + PPO）训练的 GRU 策略制导节点，用于取代传统 PNG 算法。
   它与 `uav_vision_png` 保持相同的话题接口、状态机和 CSV 统计格式，可无缝替换。
   使用 ONNX Runtime 推理，零 PyTorch 依赖，纯 C++ 实现。

   ```bash
   # 方式1：launch 启动（自动加载 config/params.yaml）
   ros2 launch uav_rl_guidance rl_guidance.launch.py

   # 方式2：直接运行（使用代码默认参数）
   ros2 run uav_rl_guidance uav_rl_guidance

   # A/B 基线模式：全程使用内置 PNG，不加载策略模型（与 uav_vision_png 等价）
   ros2 launch uav_rl_guidance rl_guidance.launch.py fallback_png:=true

   # 台架调试：跳过起飞，直接进入 SEARCHING
   ros2 launch uav_rl_guidance rl_guidance.launch.py bench_test:=true
   ```

   > **架构说明**：RL 策略在 INTERCEPT 阶段输出速度指令；若策略出现异常（watchdog），自动回退到内置 PNG 控制器。
   > 策略模型（ONNX `.onnx`）存放在 `uav_rl_guidance/models/` 下，
   > 配套训练项目见同级目录 [End2End-FPVIntercept](../End2End-FPVIntercept/README.zh-CN.md)
   > （上游项目为 [AeroIntercept](https://github.com/Eaglewzw/AeroIntercept)）。
   > 当前节点使用 15 维观测的 GRU 策略，更新兼容的 `uav_rl_guidance/models/policy.pt` 后，
   > 在本仓库根目录执行 `python3 uav_rl_guidance/src/export_onnx.py`；端到端图像策略需使用其对应的部署接口。

3. **启动视觉检测**:
   ```bash
   ros2 run uav_vision_dectect uav_vision_dectect
   ```
---

---

> **项目维护者注**：如果您需要更多关于算法推导（PNG）的细节，请参考各功能包下的 `include/` 头文件或联系开发团队。
