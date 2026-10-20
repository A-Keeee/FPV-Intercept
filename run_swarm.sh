#!/bin/bash

# =======================================================
# PX4 双机仿真启动脚本
# Drone 1: gz_x500_depth (带深度相机, Instance 1)
# Drone 2: gz_x500       (普通版,     Instance 2)
# 世界: grass_world
# =======================================================

set -e
set -o pipefail

# 仅作为失败上限；每一步收到实际就绪信号后立即继续。
START_TIMEOUT="${START_TIMEOUT:-30}"
if [[ ! "$START_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "❌ START_TIMEOUT 必须是正整数（秒）"
    exit 1
fi

# -------------------------------------------------------
# 0. 检查工作目录 & 依赖工具
# -------------------------------------------------------
if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "❌ 错误：请在 PX4-Autopilot 根目录下运行此脚本"
    echo "   当前目录: $(pwd)"
    exit 1
fi

if ! command -v rotatelogs &>/dev/null; then
    echo "❌ 错误：未找到 rotatelogs，请安装 apache2-utils"
    echo "   sudo apt install apache2-utils"
    exit 1
fi
for command_name in MicroXRCEAgent ros2 gz ss pidof timeout; do
    if ! command -v "$command_name" &>/dev/null; then
        echo "❌ 错误：未找到 $command_name，请先加载 ROS / 工作区环境并安装依赖"
        exit 1
    fi
done

# 日志轮转配置：每个日志最大 5MB，保留最近 2 个文件
LOG_MAX_SIZE="5M"
LOG_MAX_FILES=2

PIDS=()
PID_AGENT="" PID1="" PID2="" PID_BRIDGE=""
ACTIVE_PROBE=""
READINESS_LOG=$(mktemp)
BRIDGE_CONFIG=$(mktemp --suffix=.yaml)

# 在任何子进程启动前注册清理，启动失败和 Ctrl+C 都会执行。
cleanup() {
    local exit_status=$?
    trap - EXIT SIGINT SIGTERM
    echo ""
    echo "🛑 正在关闭所有进程..."
    if [[ -n "$ACTIVE_PROBE" ]]; then
        kill "$ACTIVE_PROBE" 2>/dev/null || true
        wait "$ACTIVE_PROBE" 2>/dev/null || true
    fi
    if ((${#PIDS[@]})); then
        kill "${PIDS[@]}" 2>/dev/null || true
        # 这里只给退出清理一个宽限期，不用于判断启动就绪。
        sleep 1
    fi
    killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
    rm -f "$READINESS_LOG" "$BRIDGE_CONFIG"
    echo "✅ 清理完成"
    exit "$exit_status"
}
trap cleanup EXIT
trap 'exit 130' SIGINT
trap 'exit 143' SIGTERM

# 保存实际程序 PID，而不是日志轮转进程的 PID。
start_logged() {
    local pid_variable=$1 log_file=$2
    shift 2
    "$@" > >(rotatelogs -n "$LOG_MAX_FILES" "$log_file" "$LOG_MAX_SIZE") 2>&1 &
    printf -v "$pid_variable" '%s' "$!"
    PIDS+=("$!")
}

# 同时监督已启动的程序：它们提前退出时立即失败，不继续启动后续部分。
check_services_running() {
    local description=$1 pid
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
            echo "❌ 等待 $description 时，已启动的进程 $pid 提前退出，请检查运行日志"
            return 1
        fi
    done
}

wait_ready() {
    local description=$1 completed_pid="" probe_status
    shift
    check_services_running "$description" || exit 1
    echo "⏳ 等待 $description（上限 ${START_TIMEOUT} 秒，收到就绪信号即继续）..."
    timeout --kill-after=2 "$START_TIMEOUT" "$@" > "$READINESS_LOG" 2>&1 &
    ACTIVE_PROBE=$!
    if wait -n -p completed_pid "$ACTIVE_PROBE" "${PIDS[@]}"; then
        probe_status=0
    else
        probe_status=$?
    fi
    if [[ "$completed_pid" != "$ACTIVE_PROBE" ]]; then
        echo "❌ 等待 $description 时，已启动的进程 ${completed_pid:-未知} 提前退出，请检查运行日志"
        exit 1
    fi
    ACTIVE_PROBE=""
    if ((probe_status != 0)); then
        echo "❌ $description 未就绪（退出码 $probe_status），后续部分不会启动"
        cat "$READINESS_LOG"
        exit 1
    fi
    check_services_running "$description" || exit 1
    echo "   -> $description 已就绪"
}

wait_vehicle_status() {
    local instance=$1
    wait_ready "Drone $instance 的 ROS 状态消息" \
        ros2 topic echo "/px4_${instance}/fmu/out/vehicle_status_v1" \
        px4_msgs/msg/VehicleStatus --once --field timestamp \
        --qos-reliability best_effort
}

# -------------------------------------------------------
# 1. 彻底清理残留进程
# -------------------------------------------------------
echo "🧹 正在清理残留进程..."
killall -9 px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge 2>/dev/null || true
wait_ready "残留仿真进程退出" bash -c '
    while pidof px4 gz-sim gz-server gz-client MicroXRCEAgent parameter_bridge >/dev/null; do
        sleep 0.1
    done
'

# 清理旧日志
if ls ./*.log 1>/dev/null 2>&1; then
    rm -f ./*.log
    echo "   -> 已清除旧的运行日志 (*.log)"
fi

# 清理 PX4 仿真缓存
PX4_LOG_DIR="build/px4_sitl_default/tmp/rootfs/log"
if [ -d "$PX4_LOG_DIR" ]; then
    rm -rf "${PX4_LOG_DIR:?}"/*
    echo "   -> 已清理 PX4 仿真缓存日志"
fi

# -------------------------------------------------------
# 2. 启动 MicroXRCEAgent (ROS 2 通信桥)
# -------------------------------------------------------
echo ""
echo "📡 正在启动 MicroXRCEAgent..."
start_logged PID_AGENT agent.log MicroXRCEAgent udp4 -p 8888
echo "   -> Agent PID: $PID_AGENT (日志: agent.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
# $1 在子 shell 中表示 Agent PID，应保持字面量传入。
# shellcheck disable=SC2016
wait_ready "MicroXRCEAgent 绑定 UDP 8888" bash -c '
    until ss -H -lunp "sport = :8888" | grep -Fq "pid=$1,"; do
        sleep 0.1
    done
' _ "$PID_AGENT"

# -------------------------------------------------------
# 3. 启动第一架无人机 (带深度相机, Instance 1)
#    同时拉起 Gazebo 仿真服务器 + grass_world
# -------------------------------------------------------
echo ""
echo "🚀 正在启动 Drone 1 (gz_x500_depth, Instance 1)..."
start_logged PID1 drone1.log env PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4002 \
PX4_GZ_MODEL_POSE="0,0" \
PX4_SIM_MODEL=gz_x500_depth \
PX4_PARAM_EKF2_HGT_REF=2 \
    ./build/px4_sitl_default/bin/px4 -d -i 1
echo "   -> Drone 1 PID: $PID1 (日志: drone1.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"

wait_ready "Gazebo grass_world 世界发布状态" \
    gz topic -e -t /world/grass_world/stats -n 1
wait_vehicle_status 1

	# -------------------------------------------------------
	# 4. 启动 ROS 2 相机桥接 (YAML 内嵌，直接映射到 /camera/image)
	# -------------------------------------------------------
	echo ""
	echo "🌉 正在启动 ROS 2 相机桥接..."

	cat > "$BRIDGE_CONFIG" << 'YEOF'
- ros_topic_name: "/camera/image"
  gz_topic_name: "/world/grass_world/model/x500_depth_1/link/camera_link/sensor/IMX214/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
YEOF

	start_logged PID_BRIDGE bridge.log ros2 run ros_gz_bridge parameter_bridge \
	    --ros-args -p config_file:="$BRIDGE_CONFIG"
	echo "   -> Bridge PID: $PID_BRIDGE (bridge.log)"
	echo "   -> /camera/image"
	wait_ready "ROS 相机第一帧 /camera/image" \
	    ros2 topic echo /camera/image sensor_msgs/msg/Image --once --field header \
	    --qos-reliability best_effort
echo ""
echo "🚀 正在启动 Drone 2 (gz_x500, Instance 2)..."
start_logged PID2 drone2.log env PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=grass_world \
PX4_SYS_AUTOSTART=4001 \
PX4_GZ_MODEL_POSE="20,0" \
PX4_SIM_MODEL=gz_x500 \
    ./build/px4_sitl_default/bin/px4 -d -i 2
echo "   -> Drone 2 PID: $PID2 (日志: drone2.log, 最大 ${LOG_MAX_SIZE} x${LOG_MAX_FILES})"
wait_vehicle_status 2

# -------------------------------------------------------
# 6. 启动摘要
# -------------------------------------------------------
echo ""
echo "✅ 所有系统启动完成！"
echo "================================================"
echo "  MicroXRCEAgent : PID $PID_AGENT  (port 8888)"
echo "  Drone 1 (相机)  : PID $PID1      Instance 1  UXRCE_DDS_KEY=2  /px4_1/"
echo "  Drone 2 (普通)  : PID $PID2      Instance 2  UXRCE_DDS_KEY=3  /px4_2/"
echo "  Camera Bridge  : PID $PID_BRIDGE"
echo "------------------------------------------------"
echo "  日志限制        : 每个最大 ${LOG_MAX_SIZE}，保留 ${LOG_MAX_FILES} 个文件"
echo "  相机话题        : /camera/image"
echo "  世界地图        : grass_world"
echo "  Drone 1 位置   : (0,  0)"
echo "  Drone 2 位置   : (0, 20)"
echo "================================================"
echo ""
echo "  验证命令:"
echo "  ros2 topic list | grep camera"
echo "  ros2 topic hz /camera/image"
echo "  ros2 topic list | grep px4"
echo ""
echo "按 Ctrl+C 关闭所有仿真进程"

# -------------------------------------------------------
# 7. 监督所有子进程
# -------------------------------------------------------
# 任一核心进程退出就清理其余进程，避免留下不完整的仿真。
wait -n "${PIDS[@]}" || true
echo "❌ 核心仿真进程已退出，请检查 agent.log / drone1.log / drone2.log / bridge.log"
exit 1
