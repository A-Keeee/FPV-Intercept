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

# 临时关闭大目标 LOCAL_FULL，跟踪时使用按目标框扩大后的 ROI
ros2 launch uav_ard_mav ard_mav.launch.py enable_local_full:=false

# 临时指定 LOCAL_FULL 的识别模型：global / local / local-multi
ros2 launch uav_ard_mav ard_mav.launch.py local_full_model:=global

# 直接运行时使用代码默认值，不自动读取 params.yaml
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

`enable_local_full=true` 时，大目标使用 `LOCAL_FULL`：选定模型直接对整张图识别，不裁剪 ROI。
当前帧处于 `LOCAL_FULL` 时，窗口和调试图像右上角单独显示带深色背景的醒目标识，退出该模式后自动消失。
`local_full_model` 独立选择 `global`、`local` 或 `local-multi`（默认），分别读取
`local_full_models` 下的全局、原始局部、多尺度局部权重路径：

```yaml
local_full_model: local-multi
local_full_models:
  global: models/global_best.onnx
  local: models/local_best.onnx
  local-multi: models/local_multiscale_ft_20261030.onnx
```

此选择只影响 LOCAL_FULL；GLOBAL、LOCAL 和 LOCAL_MULTI 继续使用原有的 `global_model/local_model`。
LOCAL_FULL 选择 `global` 时使用 `global_conf`，检测分支为 GAD；其余两项使用 `local_conf`，分支为 LAD。
节点启动日志输出选项、实际权重路径和置信度阈值。相同路径复用已有模型，仅在需要不同权重时额外加载一份；
关闭 LOCAL_FULL 时不加载额外权重，每帧仍只运行一个外观模型。
在 YAML 中将 `enable_local_full` 设为 `false` 可关闭此模式，跟踪阶段始终使用自适应 ROI；请使用布尔值 `true/false`。
首次发现及彻底丢失后的全图搜索不受此开关影响。ROI 可按尺寸自然扩大至全图，此时仍属于 LOCAL。
画面第三行显示 `FOV`、`ROI fill` 和下一帧模式，日志也输出两个比例。
`FOV = max(检测框宽/图像宽, 检测框高/图像高)`，不是面积占比。
`ROI fill = max(检测框宽, 检测框高) / min(crop_size, 图像宽, 图像高)`。
所有比例均使用原图坐标计算，局部检测框先转换回原图坐标。

窗口与调试图像在目标框旁显示 `Drone 0.87`，顶部同时显示当前分支与 `conf`；无检测时为 `conf --`。
`show_width`、`show_height` 仅设置显示窗口的初始尺寸，画面保持比例；
检测、发布框坐标和调试图像仍使用原图尺寸。
窗口和调试图像左上角显示实际处理 FPS，每 0.5 秒更新一次，包含检测、显示和等帧的影响，
与 `publish_rate` 的结果发布频率不同；首个统计窗口完成前显示 `FPS: --`。
GAD/LAD 的分数来自 YOLO，启动 YAML 中的筛选阈值分别为 `global_conf=0.3`、`local_conf=0.1`；
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

1. 未锁定目标时优先执行 LOCAL_MULTI，默认每 10 个全图搜索处理帧才执行一次 GLOBAL，
   即连续 9 帧 LOCAL_MULTI、1 帧 GLOBAL；`global_search_interval` 可调整周期，1 表示仅用 GLOBAL。
   GLOBAL 使用全局 YOLO26（GAD，`global_conf`）；LOCAL_MULTI 使用 `local_model`
   （默认 `local_multiscale_ft_20261030.onnx`，LAD，`local_conf`）对整帧重识别，
   用于补充全局模型对近距离大目标的漏检。
2. `enable_local_full=true` 时，全局或局部模型检测成功后，根据检测框大小选择下一帧模式：
   `FOV >= full_frame_enter_ratio=0.20` 或 `ROI fill >= roi_fill_enter_ratio=0.80` 时进入
   LOCAL_FULL，此后连续使用 `local_full_model` 选定的模型识别全图，不按搜索周期更换模型。
   只有同时满足 `FOV <= full_frame_exit_ratio=0.15` 和
   `ROI fill <= roi_fill_exit_ratio=0.60` 才恢复 LOCAL；中间区间维持原模式，减少反复切换。
   对 300 像素基础 ROI，长边达到 240 像素也会切全图，缩小到 180 像素以内才允许恢复，
   避免宽扁目标在达到画面比例阈值前被固定窗口截断。
   新阈值与模式从下一处理帧生效，每帧仍只运行一个外观模型。
   LOCAL 和 LOCAL_FULL 都保留 30×20 网格 LK/RANSAC 相机补偿及卡尔曼相对速度预测，
   预测点仅辅助搜索，不作为有效检测。
3. LOCAL：ROI 宽、高分别为 `max(crop_size, ceil(目标尺寸下限 * roi_padding)) + roi_growth * lost_frames`，
   默认最小 300×300、目标宽高各放大 1.20 倍、每漏检帧再增加 12 像素。
   宽、高分别限制在图像宽、高以内，边界处平移至图内；宽目标不再受到图像短边的尺寸限制。
   全局模型检测框的宽、高在本轮跟踪中始终作为下限，局部框变小也不会使 ROI 小于该全局框。
   ROI 内的局部检测只增大已记住的目标尺寸，避免裁剪后的残缺框导致下一帧 ROI 继续缩小；
   局部全图检测可更新目标尺寸，但仍保留本轮全局框下限。余量超出画面时以图像边界为限。
   此规则根据已检测的尺寸工作，目标下一帧突然变大或快速移出预测位置时仍可能漏检。
   使用局部 YOLO26（LAD，阈值 0.1），漏检期间保留相机补偿和卡尔曼预测。
4. LOCAL 或 LOCAL_FULL 连续失败达到 `local_patience=30` 个处理帧后，清除跟踪与大目标状态，
   下一帧优先 LOCAL_MULTI 全图重捕获，持续失败时才按上述周期插入 GLOBAL。
   任一分支检测成功后按上述目标尺寸条件选择 LOCAL/LOCAL_FULL，并重置搜索计数。
   彻底丢失、节点重置或图像尺寸变化也清除大目标状态和记住的目标尺寸，从 LOCAL_MULTI 重新开始。
   每个搜索帧只使用一个外观模型，预测框不会作为有效检测发布。
5. `motion_fallback=false` 默认关闭外观漏检后的运动级联，以减少丢失期间重复光流、
   帧差和 CNN 推理耗时。设为 `true` 可恢复 LAD 失败后 LMD、全图外观失败后 GMD。
   LOCAL_FULL 漏检时也可尝试全图 GMD。此开关不关闭相机运动补偿。
   运动分支保留高斯滤波、相机补偿帧差、光照／背景运动阈值修正、形态学、区域合并、
   Shi-Tomasi/LK 速度与方向方差筛选，以及官方 32×32 外观 CNN。
   单应估计失败时拒绝运动候选；局部搜索仍可用相对速度预测。

YOLO 使用 RGB、[0,1]、stride=32 的动态矩形 letterbox，解析 YOLO26 的 `[1,N,6]`
端到端 `xyxy,score,class` 输出，不套用旧 YOLOv5 解码或额外 NMS。
全局取最高置信度，局部置信度相同时按预测距离排序。CNN 使用 BGR、[0,1]、类别 1 为 MAV；
局部运动候选按预测距离选择。卡尔曼只用真实检测更新。

本版本在原 `track_glad.yaml` 迁移基础上使用局部优先搜索和可选运动级联。
离线工具使用 C++ 默认参数，ROS launch 使用 YAML（例如 ROI 增长和全局阈值以 YAML 为准）。
不声称严格复现论文精度；OpenCV、推理后端和浮点差异可能影响边界候选。
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
ROI 可以是矩形；CSV 新增 `roi_width,roi_height`，原 `roi_side` 列保留为宽度以兼容旧脚本。
不读取标注、不控制飞行器。测试覆盖滤波相机平移、边缘 ROI、单应估计、运动补偿失败拒绝、
模型真实推理、局部优先与周期全局搜索、显式重置和分辨率变化。
大目标测试使用可变尺寸的微型 ONNX 模型，覆盖全局/局部检出后的全图跟踪、
ROI 内目标变大、原图坐标比例、恢复 ROI、阈值回差及丢失/重置后的模式恢复。
另覆盖关闭 LOCAL_FULL 后的全局/局部捕获、目标尺寸保底、超出图像短边的目标、
漏检扩张及重置后清除尺寸下限，以及 `roi_padding >= 1` 的参数约束。
模型选择测试覆盖 LOCAL_FULL 三种选项的真实推理路由、全局/局部阈值、恢复 ROI 和周期搜索后的模型恢复，
以及非法选项拒绝、关闭模式时不加载额外权重。
重识别回归测试另用两个微型 ONNX 测试模型，模拟全局模型持续漏检、local-multi 成功识别，
验证多次丢失后立即使用 LOCAL_MULTI 重识别并恢复 ROI 跟踪；测试模型不用于节点推理。

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
