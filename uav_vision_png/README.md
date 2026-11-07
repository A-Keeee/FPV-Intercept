# 视觉 PNG 控制调试

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uav_ard_mav uav_vision_png --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 分别在两个终端运行
ros2 launch uav_ard_mav ard_mav.launch.py
ros2 launch uav_vision_png vision_png.launch.py
```

`vision_png.launch.py` 默认读取 `config/params.yaml`。只有显式传入的 `speed_cmd`、
`N_png`、`record_csv`、`csv_path` 才会覆盖 YAML；例如 `speed_cmd:=2.0`。也可通过
`params_file:=/absolute/path/params.yaml` 选择其他配置。修改后重启节点生效。

总速度上限由 `speed_cmd` 设置，当前为 **3.0 m/s**。水平速度矢量变化率由
`max_accel_xy` 设置（代码默认 0.8 m/s²，launch 时以 YAML 为准），
垂直变化率 `max_accel_z=0.6 m/s²`；偏航角速度上限 `max_yaw_rate=0.5 rad/s`，
变化率 `max_yaw_accel=0.5 rad/s²`。限制器从实测速度开始，跟踪、短暂丢失、减速、
重捕获和 DONE 共用同一状态，因此切换不会绕过指令平滑。
如果切换前实测速度已超限，会逐步减速回限值。

垂直视场补偿通过 `max_vertical_correction=0.8 m/s` 限幅，补偿叠加后还受
`max_vertical_speed=1.0 m/s` 和总速度上限约束。移除了原先至少 2 m/s 的硬下限。
这些参数约束发送给 PX4 的速度与偏航指令；实际机体加速度和倾角仍由 PX4 内环决定，
需要在仿真中观察姿态验证，不能把指令变化率当作实际倾角的硬限制。

ARD-MAV 重复发布同一源图像时，PNG 只消费一次该时间戳；无源时间戳的旧节点仍按消息更新。
短暂丢失期间保持实际已发布速度，收到 `coast_thresh=30` 条无效检测消息后平滑减速，
达到 `lost_thresh=90` 条后进入 TRACK_LOST 旋转搜索。ARD-MAV 发布 60 Hz 时分别约为
0.5 秒、1.5 秒。检测话题断流超过 `detection_timeout=0.5` 秒也进入减速搜索。
任何丢失恢复都会重新初始化 LOS，避免将丢失期间的角度变化作为一帧导引增量。

默认 `record_csv=false`，统计使用 ROS 原生日志。每秒记录一次 `[STATS] event=progress`，
包括状态、GPS 距离、最近接距离和有效/丢失源结果计数；成功时立即记录
`event=hit state=DONE`，正常退出时记录 `event=summary hit=true/false`。
`gps_distance_m=-1` 表示两机 GPS 数据尚未就绪。距离来自 GPS 换算，不能等同于 Gazebo 碰撞事件。

默认日志位于 `~/.ros/log/`，节点文件名包含进程号和时间戳，每次运行独立记录。
可将本次进程的 ROS 日志统一写到工作区目录：

```bash
ROS_LOG_DIR="$PWD/log/vpng" ros2 launch uav_vision_png vision_png.launch.py
```

成功判定在 200 Hz 控制循环中独立更新，INTERCEPT 的续飞/减速分支及 TRACK_LOST
都继续检查。它不会读取旧 CSV，也不再依赖记录文件是否能打开。
旧实现将 CSV 打开与命中检测放在同一函数，打开失败就提前返回；这会漏判。
旧文件正常可写时，每次启动会覆盖旧内容，删除文件本身不会清除程序中的判定状态。

如需兼容原绘图脚本，可以显式启用 10 Hz CSV 导出：

```bash
ros2 launch uav_vision_png vision_png.launch.py record_csv:=true csv_path:=/tmp/vpng_experiment.csv
```

CSV 打开或写入失败只输出一次错误并关闭该导出，本次运行仍检查成功条件并记录 ROS 日志。
记录定时器与控制回调在同一执行器中，启用 CSV 后慢磁盘仍可能拖延其他回调，默认关闭。

需要完整数据用于绘图或回放时，可另开终端录制 rosbag；本机默认使用 SQLite3，自动创建
时间戳目录。以下话题包含视觉输入、姿态、实际速度、指令速度、GPS 和状态日志：

```bash
ros2 bag record /rosout /camera_detect_result /vpng_data \
  /px4_1/fmu/out/vehicle_odometry /px4_1/fmu/out/vehicle_local_position \
  /px4_1/fmu/out/vehicle_gps_position /px4_2/fmu/out/vehicle_gps_position \
  /px4_1/fmu/in/trajectory_setpoint
```

可通过以下命令确认生效值：

```bash
ros2 param get /uav_vision_png speed_cmd
ros2 param get /uav_vision_png max_accel_xy
ros2 param get /uav_ard_mav global_search_interval
```

测试覆盖搜索降级与重捕获、指令限制，以及 CSV 不可写、默认不触碰旧 CSV、
日志落盘、视觉丢失/断流后的成功判定和可选 CSV 导出。
统计集成测试在独立 ROS domain 中发布模拟输入，不连接运行中的 PX4：

```bash
colcon test --packages-select uav_ard_mav uav_vision_png --ctest-args -R 'test_tracking|test_setpoint_limiter|test_statistics' --output-on-failure
colcon test-result --test-result-base build/uav_ard_mav --verbose
colcon test-result --test-result-base build/uav_vision_png --verbose
```
