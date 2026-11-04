# uav_ARD-MAV

`uav_vision_dectect` 的 ROS 2 / C++ 替代实现，迁移
[`ARD-MAV/tracking`](../../ultralytics/ARD-MAV/tracking/README.md) 的单目标搜索流程和三个已有权重。
目录名为 `uav_ARD-MAV`，ROS 包名、节点名、可执行文件名均为 `uav_ard_mav`，遵循 ROS 小写下划线命名。
运行时使用 OpenCV 和 ONNX Runtime，不调用 Python、不训练模型。

## 构建与运行

需要 ROS 2 Humble、OpenCV 开发包和 ONNX Runtime C++ 安装。
当前机器的 ONNX Runtime 在 `/usr/local`，其他安装位置通过 `-DONNXRUNTIME_ROOT=/path/to/onnxruntime` 指定。
只需链接 `libonnxruntime`；CPU 构建不要求 CUDA provider 库。

在 `FPV-Intercept` 下执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to uav_ard_mav --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch uav_ard_mav ard_mav.launch.py

# 默认使用 CUDA，需要匹配的 ONNX Runtime GPU、CUDA/cuDNN 安装
ros2 launch uav_ard_mav ard_mav.launch.py provider:=cuda
# 需要 CPU 时显式覆盖
ros2 launch uav_ard_mav ard_mav.launch.py provider:=cpu

# 默认弹出 ARD-MAV 窗口，显示检测框、搜索 ROI 和检测状态。
# 无桌面环境时关闭窗口，也可通过 /uav_ard_mav/debug_image 查看图像。
ros2 launch uav_ard_mav ard_mav.launch.py show:=false

# 窗口默认 960×540，支持拖动缩放；也可指定更小的初始尺寸。
ros2 launch uav_ard_mav ard_mav.launch.py show_width:=800 show_height:=450

# 直接运行时，代码默认值与 params.yaml 一致
ros2 run uav_ard_mav uav_ard_mav
```

与旧检测节点二选一运行；`uav_vision_png`、`uav_ibvs_control`、`uav_rl_guidance` 和 BC 录制器继续使用原检测话题。
本包只负责检测和搜索，飞行控制仍由原控制节点承担。

## ROS 接口

- 输入 `/camera/image`：`sensor_msgs/msg/Image`，SensorDataQoS、深度 1，兼容 best-effort 相机桥接。
- 输出 `/camera_detect_result`：`uav_common_msg/msg/RectMsg`，可靠发布，默认 60 Hz。
  `x,y` 为原图左上角，`width,height` 为尺寸，`depth=0`；无检测时四项均为 `-1`。
  检测框坐标向外取整并限制在图内，不发布 ROI／卡尔曼预测框。
  结果保留源图像的 `header.stamp` 和 `frame_id`；同一源帧会按发布频率重复发送。
- 调试 `/uav_ard_mav/debug_image`：`sensor_msgs/msg/Image`，仅有订阅者时生成；白框为搜索 ROI，橙点为漏检预测。

窗口与调试图像在目标框旁显示 `Drone 0.87`，顶部同时显示当前分支与 `conf`；无检测时为 `conf --`。
`show_width`、`show_height` 仅设置显示窗口的初始尺寸，画面保持比例；
检测、发布框坐标和调试图像仍使用原图尺寸。
窗口和调试图像左上角显示实际处理 FPS，每 0.5 秒更新一次，包含检测、显示和等帧的影响，
与 `publish_rate` 的结果发布频率不同；首个统计窗口完成前显示 `FPS: --`。
GAD/LAD 的分数来自 YOLO，筛选阈值分别为 `global_conf=0.5`、`local_conf=0.1`；
GMD/LMD 的分数为外观 CNN 的 MAV 概率，二分类选择 MAV（概率大于 0.5）后才接受候选，
还需通过运动筛选。两个模型的分数含义不同，显示值不代表经过校准的正确率。

后台线程仅处理最新图像，推理不阻塞 60 Hz 结果定时器。`image_timeout` 默认 0.5 秒，
以接收时刻的单调时钟判断结果过期，断流／推理过慢时发布无效框。
图像尺寸变化、时间戳回退、帧间时间超过此阈值或处理异常会重置搜索状态。
没有图像时也持续发布无效框。

所有参数见 [`config/params.yaml`](config/params.yaml)，启动时读取，修改后重启节点。
`imgsz` 须为不小于 160 的 32 倍数，以满足导出模型的固定 TopK=100。
`global_model`、`local_model`、`classifier_model` 的相对路径以安装后的包 share 目录为基准；可改用绝对路径。
自定义相机话题可通过参数或 ROS remap 设置。

```bash
ros2 run uav_ard_mav uav_ard_mav --ros-args -p image_topic:=/my_camera/image -p provider:=cuda
ros2 launch uav_ard_mav ard_mav.launch.py params_file:=/absolute/path/params.yaml
```

## 迁移的方法

1. 未锁定目标时，按处理帧交替执行 GLOBAL 与 LOCAL_MULTI，从 GLOBAL 开始。
   GLOBAL 使用全局 YOLO26（GAD，`global_conf`）；LOCAL_MULTI 使用 `local_model`
   （默认 `local_multiscale_ft_20261030.onnx`，LAD，`local_conf`）对整帧重识别，
   用于补充全局模型对近距离大目标的漏检。外观检测失败后均尝试全局运动检测（GMD）。
2. 检测成功后，下一帧进入 LOCAL。30×20 网格 LK 与 RANSAC 单应变换补偿相机运动；
   状态 `(vx,vy,ax,ay)` 的卡尔曼滤波器叠加相对速度预测。
3. LOCAL：窗口边长 `300 + 4 * lost_frames`，边界处平移至图内；
   先局部 YOLO26（LAD，阈值 0.1），失败再局部运动检测（LMD）。
4. 连续局部失败达到 `local_patience` 后，下一帧从 GLOBAL 开始逐帧交替 GLOBAL / LOCAL_MULTI
   搜索；任一分支检测成功后，下一帧恢复 LOCAL，并将下次重搜索的起点重置为 GLOBAL。
   节点重置或图像尺寸变化也从 GLOBAL 重新开始；每个搜索帧只使用一个外观模型。
5. 运动分支保留高斯滤波、相机补偿帧差、光照／背景运动阈值修正、形态学、区域合并、
   Shi-Tomasi/LK 速度与方向方差筛选，以及官方 32×32 外观 CNN。
   单应估计失败时拒绝运动候选；局部搜索仍可用相对速度预测。

YOLO 使用 RGB、[0,1]、stride=32 的动态矩形 letterbox，解析 YOLO26 的 `[1,N,6]`
端到端 `xyxy,score,class` 输出，不套用旧 YOLOv5 解码或额外 NMS。
全局取最高置信度，局部置信度相同时按预测距离排序。CNN 使用 BGR、[0,1]、类别 1 为 MAV；
局部运动候选按预测距离选择。卡尔曼只用真实检测更新。

参数默认值与原 `track_glad.yaml` 一致，包括未公开参数的实现选值。
这是现有 Python 方法的 C++ 迁移，不声称严格复现论文精度；OpenCV、推理后端和浮点差异可能影响边界候选。
原有模型与方法的训练、误判及漂移限制仍然适用，不提供多目标 ID。

## 模型来源与重新导出

`models/` 已包含 `global_best.onnx`、`local_best.onnx`、`Net_best.onnx`，
来源为原 tracking 的两个 YOLO26n `.pt` 和作者发布的 `Net_best.pth`。
2026-10-30 新增 `local_multiscale_ft_20261030.onnx`，来源为
`ultralytics/ARD-MAV/runs/yolo26n_local_multiscale_ft_50e2/weights/best.pt`。
这是该轮训练已保存的最佳检查点（`epoch=17`，从 0 计数，即第 18 轮）；导出时训练日志记录到第 37 轮。
ROS launch、直接运行节点和离线视频工具默认使用这个新局部模型。
原 `local_best.onnx`、全局模型和 CNN 均保留，源 `.pt` 未修改。
[`models/manifest.json`](models/manifest.json) 记录导出版本、源权重和产物 SHA-256。
仅在更新权重时需要带 PyTorch、Ultralytics、ONNX 的 Python 环境：

```bash
python uav_ARD-MAV/scripts/export_models.py
# 源项目在其他位置时：
python uav_ARD-MAV/scripts/export_models.py --tracking /path/to/ARD-MAV/tracking

# 只导出新局部权重，使用新的文件名保留旧模型；已存在的文件名会被拒绝
python uav_ARD-MAV/scripts/export_models.py \
  --local-weights ../ultralytics/ARD-MAV/runs/yolo26n_local_multiscale_ft_50e2/weights/best.pt \
  --local-name local_multiscale_ft_20261030.onnx
```

导出器复用源项目的 CNN 定义，YOLO 使用 float32、opset 17、动态尺寸、`max_det=100`、无额外 NMS。
源 `.pt/.pth` 不会被覆盖，更新后需要重新构建以安装模型。

回退原局部模型时，可将 `config/params.yaml` 的 `local_model` 改为 `models/local_best.onnx`，
重新构建并重启 launch；直接运行时也可临时覆盖：

```bash
ros2 run uav_ard_mav uav_ard_mav --ros-args -p local_model:=models/local_best.onnx
```

## 离线验证

```bash
colcon test --packages-select uav_ard_mav
colcon test-result --verbose

# 与 ROS 节点复用同一 C++ 跟踪器，无需相机或 PX4；0 表示完整视频
ros2 run uav_ard_mav ard_mav_video /path/to/video.mp4 \
  "$(ros2 pkg prefix uav_ard_mav)/share/uav_ard_mav/models" 300 /tmp/ard_mav_frames.csv cpu
```

离线工具使用源码默认参数，不自动读取 ROS YAML；输出每帧检测、模式、分支、ROI、预测点和运动级联统计。
不读取标注、不控制飞行器。测试覆盖滤波相机平移、边缘 ROI、单应估计、运动补偿失败拒绝、
模型真实推理、全图搜索模型逐帧交替、显式重置和分辨率变化。
重识别回归测试另用两个微型 ONNX 测试模型，模拟全局模型持续漏检、local-multi 成功识别，
验证多次丢失／重识别后恢复 ROI 跟踪；测试模型不用于节点推理。

本机迁移验证（2026-10-29）：Release 构建与 5 项自动测试通过；`phantom12`、`phantom08`
各前 300 帧的模式、检测分支、检测有无和丢失帧数均与原 Python 逐帧一致。
`phantom12` 检测框坐标最大绝对差约 0.00013 像素；`phantom08` 运动级联候选统计一致。
额外通过 34 帧遮挡／重捕获序列，以及禁用外观检测后的 100 帧运动分支对照（含 34 次 LMD 检出）。
ROS 联调确认 60 Hz 输出、best-effort 图像接收、源 header、调试图像、断流失效和异常图像处理；
CUDA 后端完成 5 帧推理试跑。以上是迁移功能检查，不是全数据集精度或实飞结果。

2026-10-30 新局部模型通过 ONNX 图检查、640×640／384×640／320×320 动态输入推理，
以及原 `.pt` 与 ONNX 的图像预测对照。C++ 在 `phantom12` 前 60 帧中的模式、分支和检出情况
与使用新 `.pt` 的 Python 一致（1 次 GAD、59 次 LAD）；框坐标最大差约 0.00013 像素。
旧模型哈希保持不变。此检查验证格式转换和接入，不用于判断新模型精度是否优于旧模型。
