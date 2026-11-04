# uav_yolo_track

ROS 2 YOLO + Ultralytics 内置跟踪器的新包，提供**单模型全图**和全局／局部两种入口，与 `uav_ard_mav` 并存。
ROS 消息、ONNX 检测和单目标选择用 C++；ID 关联调用 **Ultralytics 原版 BoT-SORT**，
默认启用原生稀疏光流相机运动补偿，关闭 ReID，不下载额外模型。

本包不使用原有的 `RelativeVelocity`、自写卡尔曼预测或 GLAD 运动检测／CNN 级联。
**保留 BoT-SORT 内部的卡尔曼滤波**，符合本次确认的需求；不是“完全没有卡尔曼”。

## 构建与启动

在 `FPV-Intercept` 目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to uav_yolo_track --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 新增：一个模型始终检测全图，再交给内置 track
ros2 launch uav_yolo_track single_model.launch.py

# 保留原来的全局／局部版本
ros2 launch uav_yolo_track yolo_track.launch.py

# 无桌面显示；仍可订阅调试图像
ros2 launch uav_yolo_track single_model.launch.py show:=false
# 自定义显示窗口大小（两个入口均支持）
ros2 launch uav_yolo_track single_model.launch.py show_width:=800 show_height:=450
# CUDA 只改变 C++ ONNX 推理后端
ros2 launch uav_yolo_track single_model.launch.py provider:=cuda
# 默认使用 CUDA；需要 CPU 时显式覆盖
ros2 launch uav_yolo_track single_model.launch.py provider:=cpu
```

两种入口默认均弹窗，显示原图、绿色目标框、置信度、内置跟踪器 ID 和漏检帧数。
窗口默认 `960×540`，保持图像比例，支持拖动边框缩放；`show_width`、`show_height`
只设置初始显示大小，检测、发布框坐标和调试图像仍使用原图尺寸。
窗口和调试图像左上角显示实际处理 FPS，每 0.5 秒更新一次，包含检测／关联、显示和等帧的影响，
与 `publish_rate` 的结果发布频率不同；首个统计窗口完成前显示 `FPS: --`。
单模型入口显示 `SINGLE`，没有白色 ROI；全局／局部入口保留白色局部 ROI。
直接运行：`ros2 run uav_yolo_track uav_yolo_single_track`（单模型），
或 `ros2 run uav_yolo_track uav_yolo_track`（全局／局部）。
原检测包继续保留；一次只启动一个向 `/camera_detect_result` 发布结果的检测节点。
PNG、RL、目标机和相机桥接的启动方式不变，本包不直接发送飞行控制指令。

## Python 跟踪后端

Ultralytics 的内置跟踪器是 Python 实现。本包使用独立 Python 工作进程调用
`ultralytics.trackers.bot_sort.BOTSORT.update()`，它也是
[`model.track()` 官方回调](https://github.com/ultralytics/ultralytics/blob/v8.4.6/ultralytics/trackers/track.py)
使用的入口。检测仍由 C++ 执行，因此没有调用高层 `model.track()` 重复跑 YOLO，
也没有自行移植／重写 BoT-SORT。

独立进程通过本机 socket 交换全图 BGR 图像、检测框和跟踪结果，避免 Python/OpenCV 与 ROS/OpenCV 的动态库冲突。
工作进程随节点启动、关闭；断开或超时会使当帧结果失效，后续帧可重新启动后端。
它不导入 `rclpy`，无需把 ROS 安装到 Conda 环境中。

Python 环境需要 `ultralytics`、`torch`、`numpy`、`opencv-python`、`PyYAML`、`lap`、`scipy`。
本机使用已有 `yolo` 环境的 Ultralytics 8.4.6，不自动升级环境。
构建时优先查找 `~/anaconda3/envs/yolo/bin/python3` 或 `~/miniconda3/envs/yolo/bin/python3`；
可显式指定：

```bash
colcon build --packages-up-to uav_yolo_track --cmake-args \
  -DULTRALYTICS_PYTHON_EXECUTABLE=/absolute/path/to/python
# 或运行时覆盖
ros2 launch uav_yolo_track yolo_track.launch.py python_executable:=/absolute/path/to/python
```

## 每帧流程

### 单模型入口：`single_model.launch.py`

1. 只加载 [`config/single_model.yaml`](config/single_model.yaml) 的 `model`，默认 `global_best.onnx`。
2. **每帧都用同一个模型检测完整图像**，按 `conf=0.3` 筛选候选，再更新唯一的内置跟踪器。
3. 初次从已确认轨迹中选择最高置信度目标，随后锁定该 ID；短暂丢失时继续全图检测。
4. 连续漏检 `target_patience=30` 帧后释放所选 ID，下一帧允许重新选目标。
   每个漏检帧输出无效框，不发布纯预测框；这个参数不会触发 ROI 或模型切换。
   等待期间也逐帧尝试恢复原 ID，无需等满 30 帧才重新检测。

该入口只创建一个 ONNX 推理实例，不读取 `global_model`、`local_model` 或局部窗口参数。
可将 `model` 改为其他兼容的 YOLO ONNX 模型绝对路径；原权重保留。

### 全局／局部入口：`yolo_track.launch.py`

1. 初始 GLOBAL：全图 YOLO，所有有效候选交给唯一的官方跟踪器；从已确认轨迹中选最高置信度目标。
2. 发现目标后，下一帧进入 LOCAL：以**最近一次有效跟踪框**为中心生成局部窗口，运行局部 YOLO。
3. 局部候选坐标先加回 ROI 偏移，再用**原图坐标与完整原图**更新同一个跟踪器。
   不给两个 YOLO 模型各建一个 track 实例；不将移动裁剪窗口的坐标直接用于关联。
4. LOCAL 锁定当前 ID，只发布该 ID 当帧实际匹配到检测的轨迹；短暂丢失不自动换成旁边的其他 ID。
5. 漏检时以最后有效框为中心扩大窗口：`crop_size + roi_growth * lost`。
   连续失败 `local_patience` 帧后，下一帧回到 GLOBAL，允许重新选目标。
   等待期间每帧都在扩大的局部窗口中尝试恢复原 ID，未找到时不进行全图搜索。
   不额外叠加应用侧速度预测；官方跟踪器内部仍执行自己的运动估计。

每个处理帧只运行一个 YOLO 模型，跟踪器更新一次（包括空检测）。
这两个等待参数按实际处理帧数计数，`30` 帧在 30 FPS 下约 1 秒、15 FPS 下约 2 秒；
不按 `publish_rate` 的结果发布频率计算。释放 ID／切回 GLOBAL 后，实际重新选中目标
还取决于检测结果和跟踪器确认，不保证恰好在下一帧成功。
BoT-SORT 的原生预测用于关联，但其未匹配的预测不会作为有效框发布。
ID 是内置跟踪器产生的真实轨迹 ID；长期丢失／后端重置后不保证 ID 不变。

## 模型和参数

单模型入口只使用 `uav_ard_mav` 安装的 `models/global_best.onnx`；
全局／局部入口额外使用 `models/local_multiscale_ft_20261030.onnx`，不复制、不覆盖旧权重。
只共享 C++ ONNX 推理库，原包的 GLAD／自写卡尔曼跟踪核心没有链接进本包。

单模型参数在 [`config/single_model.yaml`](config/single_model.yaml)：`model` 为唯一模型，`conf` 为检测阈值。
全局／局部参数在 [`config/params.yaml`](config/params.yaml)，包含两个模型、各自阈值和 ROI 设置。
相对模型名以 `uav_ard_mav` 的 models 目录为基准，绝对路径也可。
相对 `tracker_config` 以本包 share 目录为基准。参数在启动时读取，修改后重新构建／重启。

[`config/botsort.yaml`](config/botsort.yaml) 控制官方关联器：

- `track_high_thresh=0.25`：高分检测的第一阶段关联阈值。
- `track_low_thresh=0.05`：低分检测的恢复下限；仍受前置 YOLO 阈值限制。
- `new_track_thresh=0.25`：新轨迹创建阈值，低分观测只能维持已有轨迹。
- `track_buffer=30`：内置跟踪器保留丢失轨迹的处理帧数，使用标准 30 FPS 计数基准。
- `gmc_method=sparseOptFlow`：原生相机补偿；`with_reid=false`：不启用额外外观网络。

默认 YOLO 阈值：单模型 `conf=0.3`；全局／局部版本为全局 `0.3`、局部 `0.1`。这些与 track 的关联阈值分别生效。
也支持将配置中的 `tracker_type` 改为 `bytetrack`，使用原版 ByteTrack（没有相机运动补偿）。
两种跟踪器都保留自己的内部卡尔曼，见 [Ultralytics 跟踪说明](https://docs.ultralytics.com/modes/track/)。

## ROS 接口与失效处理

- 输入 `/camera/image`：`sensor_msgs/msg/Image`，SensorDataQoS，队列深度 1，仅处理最新图像。
- 输出 `/camera_detect_result`：原 `uav_common_msg/msg/RectMsg`，可靠发布，默认 60 Hz。
  无目标时 `x/y/width/height=-1`、`depth=0`；保留源图像 header。
- `/uav_yolo_track/target_id`：`std_msgs/msg/Int32`，无有效目标时为 `-1`。
- `/uav_yolo_track/confidence`：`std_msgs/msg/Float32`，无有效目标时为 `0`。
- `/uav_yolo_track/debug_image`：带 ROI、ID 和分数的 BGR 图像，仅有订阅者时生成。

单模型节点的以上三个私有话题前缀为 `/uav_yolo_single_track/`，其调试图像没有 ROI。
两种入口的输入与 `RectMsg` 输出话题相同，不能同时向同一结果话题发布。

结果定时器与后台推理分开，60 Hz 是结果发布频率，并非承诺 60 FPS 检测。
结果超过 `image_timeout=0.5` 秒会失效；帧间长间隔、时间戳回退、尺寸变化和异常输入会清除旧搜索状态。
`backend_timeout_ms` 限制跟踪 IPC 等待时间。

## 验证

```bash
colcon test --packages-select uav_ard_mav uav_yolo_track
colcon test-result --verbose
# 复用节点的全局/局部与原生关联流程；0 表示完整视频，不控制飞机
ros2 run uav_yolo_track yolo_track_video /path/to/video.mp4 300 /tmp/yolo_track.csv cpu
# 单模型全图流程；最后一个模型参数可选，也支持绝对路径
ros2 run uav_yolo_track yolo_single_track_video /path/to/video.mp4 300 /tmp/yolo_single_track.csv cpu global_best.onnx
```

自动测试调用真实官方跟踪器，检查低置信度恢复、短暂遮挡后 ID、空检测失效、后端失败、边缘 ROI，
以及单模型入口在尺寸变化后仍保持全图模式。
全局重新搜索可能选择其他无人机；单目标接口不提供跨长时间遮挡的身份保证。

本机 2026-11-01 单模型验证：本包 4 项自动测试通过；`phantom12` 前 120 帧全程 `SINGLE`，
没有 ROI，119 帧输出 ID=1，1 帧漏检后恢复原 ID。36 帧遮挡序列全程全图检测，并成功重新捕获。
原全局／局部入口的 120 帧 CSV 与改动前完全一致。
ROS 联调确认仅声明单模型参数、60.1 Hz 结果发布、best-effort 输入、源 header、ID／置信度、
调试图像、断流失效及异常图像失效处理。
这是功能检查，尚未评估完整数据集精度或 Gazebo 飞行闭环。
