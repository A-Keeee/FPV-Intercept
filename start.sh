#!/usr/bin/env bash
# Start the PX4 simulation, QGroundControl, target, RL guidance, and detection.
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'EOF'
用法: ./start.sh [RL launch 参数...]
示例: ./start.sh fallback_png:=true

启动顺序: Gazebo/双机/相机就绪 → QGC 连接及本地估计有效 → 目标机 → 检测 → RL 制导。
每个阶段按实际消息或状态确认成功后，才启动下一阶段。
已运行的 PX4 仿真和 QGroundControl 会被复用。
日志保存在 log/start/，按 Ctrl+C 统一停止本脚本启动的程序。
可选环境变量: PX4_DIR（默认 ~/PX4-Autopilot）、QGC_APPIMAGE（地面站路径）、
START_TIMEOUT（每阶段失败超时上限，默认 30 秒；条件满足立即继续）。
EOF
    exit 0
fi

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
px4_dir=${PX4_DIR:-"$HOME/PX4-Autopilot"}
qgc_appimage=${QGC_APPIMAGE:-"$HOME/Applications/QGroundControl-v5.0.8-x86_64.AppImage"}
startup_timeout=${START_TIMEOUT:-30}

die() {
    printf '错误：%s\n' "$*" >&2
    exit 1
}

[[ -f $repo_dir/env.sh ]] || die "缺少 $repo_dir/env.sh，请先完成工作空间安装。"
[[ -x $qgc_appimage ]] || die "地面站不存在或不可执行：$qgc_appimage"
[[ $startup_timeout =~ ^[1-9][0-9]*$ ]] || die "START_TIMEOUT 必须是正整数。"

# ROS setup scripts do not support nounset in all environments.
# shellcheck source=env.sh
source "$repo_dir/env.sh"
set -u
cd -- "$repo_dir"
for command_name in ros2 gz setsid flock timeout pgrep; do
    command -v "$command_name" >/dev/null || die "未找到命令：$command_name"
done
for package_name in uav_target_sim uav_rl_guidance uav_vision_dectect; do
    ros2 pkg prefix "$package_name" >/dev/null || die "未找到 ROS 包：$package_name，请先编译工作空间。"
done

mkdir -p "$repo_dir/log/start"
exec 9>"$repo_dir/log/start/start.lock"
flock -n 9 || die "已有 start.sh 正在运行。"
log_dir="$repo_dir/log/start/$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$log_dir"
printf '日志目录：%s\n' "$log_dir"

declare -a pids=() names=()
last_pid=0

cleanup() {
    local exit_status=$? pid attempt running
    trap - EXIT INT TERM
    if ((${#pids[@]})); then
        printf '\n正在停止本次启动的程序…\n'
        for pid in "${pids[@]}"; do
            kill -INT -- "-$pid" 2>/dev/null || true
        done
        for ((attempt = 0; attempt < 30; attempt++)); do
            running=false
            for pid in "${pids[@]}"; do
                if kill -0 -- "-$pid" 2>/dev/null; then running=true; fi
            done
            if [[ $running == false ]]; then break; fi
            sleep 0.1
        done
        for pid in "${pids[@]}"; do
            kill -TERM -- "-$pid" 2>/dev/null || true
        done
        # run_swarm.sh needs one second to finish its own cleanup trap.
        sleep 1.5
        for pid in "${pids[@]}"; do
            kill -KILL -- "-$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        done
    fi
    exit "$exit_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_process() {
    local name=$1
    shift
    # Separate process groups also cover nodes spawned by ros2 launch.
    # Close the lock FD so children cannot keep the launcher locked after exit.
    setsid "$@" >"$log_dir/$name.log" 2>&1 9>&- &
    last_pid=$!
    pids+=("$last_pid")
    names+=("$name")
    printf '启动 %-20s PID=%s  日志：%s/%s.log\n' "$name" "$last_pid" "$log_dir" "$name"
}

check_services() {
    local count=${1:-${#pids[@]}} index exit_status
    for ((index = 0; index < count; index++)); do
        if ! kill -0 "${pids[index]}" 2>/dev/null; then
            exit_status=0
            wait "${pids[index]}" || exit_status=$?
            printf '%s 已退出（状态码 %s），日志：%s/%s.log\n' \
                "${names[index]}" "$exit_status" "$log_dir" "${names[index]}" >&2
            if ((exit_status == 0)); then exit_status=1; fi
            exit "$exit_status"
        fi
    done
}

wait_for_stage() {
    local stage=$1 service_count=${#pids[@]}
    shift
    printf '等待 %s 的就绪信号（超时上限 %ss）…\n' "$stage" "$startup_timeout"
    start_process "$stage" timeout --foreground --kill-after=5 "$startup_timeout" "$@"
    while kill -0 "$last_pid" 2>/dev/null; do
        check_services "$service_count"
        # Poll process status only; advancement requires the check to succeed.
        sleep 0.1
    done
    check_services "$service_count"
    if ! wait "$last_pid"; then
        die "$stage 未就绪。请查看 $log_dir/$stage.log 和对应程序日志。"
    fi
    unset 'pids[-1]' 'names[-1]'
    printf '已确认 %s 就绪。\n' "$stage"
}

wait_for_ros() {
    local stage=$1 topic=$2 message_type=$3
    shift 3
    wait_for_stage "$stage" ros2 topic echo "$topic" "$message_type" \
        --qos-reliability best_effort --qos-durability volatile --once "$@"
}

# Avoid starting another controller or duplicating nodes launched manually.
if ! node_list=$(timeout --foreground --kill-after=5 10 ros2 node list --no-daemon 9>&-); then
    die "无法查询 ROS 节点，请检查 env.sh 和 ROS 环境。"
fi
while IFS= read -r node_name; do
    case "$node_name" in
        /uav_target_sim|/uav_rl_guidance|/uav_vision_dectect|/uav_vision_png|\
        /png_interceptor|/uav_vehicle_controller|/drone_keyboard_control|/uav_ibvs_control|/uav_control)
            die "节点 $node_name 已在运行，请先停止已有节点再运行本脚本。" ;;
    esac
done <<< "$node_list"

if pgrep -u "$UID" -x px4 >/dev/null; then
    printf '检测到 PX4 已在运行，复用现有仿真。\n'
else
    [[ -f $px4_dir/run_swarm.sh ]] || die "缺少仿真脚本：$px4_dir/run_swarm.sh"
    [[ -x $px4_dir/build/px4_sitl_default/bin/px4 ]] || die "PX4 尚未编译：$px4_dir/build/px4_sitl_default/bin/px4"
    start_process simulation env --chdir="$px4_dir" START_TIMEOUT="$startup_timeout" bash ./run_swarm.sh
fi

# These checks must not require a ground station: QGC starts only afterwards.
wait_for_stage gazebo_ready gz topic --echo --topic /world/grass_world/stats --num 1
for instance in px4_1 px4_2; do
    wait_for_ros "${instance}_transport_ready" "/$instance/fmu/out/vehicle_status_v1" \
        px4_msgs/msg/VehicleStatus
done
wait_for_ros camera_ready /camera/image sensor_msgs/msg/Image --field header

if pgrep -u "$UID" -x QGroundControl >/dev/null; then
    printf '复用已运行的 QGroundControl。\n'
else
    start_process qgroundcontrol "$qgc_appimage"
fi

# QGC readiness is a link check. pre_flight_checks_pass refers to the current
# mode (initially AUTO_LOITER), whose global-position requirements differ from
# the local position/velocity control used by these Offboard nodes.
for instance in px4_1 px4_2; do
    wait_for_ros "${instance}_qgc_ready" "/$instance/fmu/out/vehicle_status_v1" \
        px4_msgs/msg/VehicleStatus \
        --filter 'not m.gcs_connection_lost'
done

# Both controllers start in local position control, then use velocity control.
# PX4 still performs mode-specific checks when they request Offboard and arm.
for instance in px4_1 px4_2; do
    wait_for_ros "${instance}_local_position_ready" "/$instance/fmu/out/vehicle_local_position" \
        px4_msgs/msg/VehicleLocalPosition \
        --filter 'm.xy_valid and m.z_valid and m.v_xy_valid and m.v_z_valid'
done

start_process uav_target_sim ros2 run uav_target_sim uav_target_sim
wait_for_ros target_heartbeat_ready /px4_2/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
    --filter 'm.position'
wait_for_ros target_ready /px4_2/fmu/out/vehicle_status_v1 px4_msgs/msg/VehicleStatus \
    --filter 'm.arming_state == m.ARMING_STATE_ARMED and m.nav_state == m.NAVIGATION_STATE_OFFBOARD'

start_process uav_vision_dectect ros2 run uav_vision_dectect uav_vision_dectect
wait_for_ros detector_ready /camera_detect_result uav_common_msg/msg/RectMsg
# The detector catches YOLO load failures and can still publish empty rectangles.
grep -qF 'YOLO Detector Initialized Successfully.' "$log_dir/uav_vision_dectect.log" \
    || die "检测模型未成功加载，请查看 $log_dir/uav_vision_dectect.log。"

start_process uav_rl_guidance ros2 launch uav_rl_guidance rl_guidance.launch.py "$@"
wait_for_ros guidance_ready /px4_1/fmu/in/offboard_control_mode px4_msgs/msg/OffboardControlMode \
    --filter 'm.position or m.velocity'

printf '\n仿真、地面站和三个 ROS 节点已启动。按 Ctrl+C 统一停止。\n'
printf '查看日志：tail -f "%s/uav_rl_guidance.log"\n' "$log_dir"

# Check already-exited children too: wait -n can miss an early startup failure.
while true; do
    check_services
    sleep 1
done
