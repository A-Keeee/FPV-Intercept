#!/usr/bin/env bash
# Resume user-side setup after install_system_dependencies.sh has completed.
set -euo pipefail

if [[ $EUID -eq 0 ]]; then
    echo "Run this script as your normal user, without sudo." >&2
    exit 1
fi

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace="$repo_dir"
px4_dir="$HOME/PX4-Autopilot"
agent_dir="$HOME/Micro-XRCE-DDS-Agent"
agent_prefix="$HOME/.local/opt/microxrce-agent-2.4.3"
workers=${AID_BUILD_JOBS:-2}

command -v rotatelogs >/dev/null
gz sim --versions | grep -E '^8\.' >/dev/null
test -f /usr/local/include/onnxruntime_cxx_api.h
test -f /usr/local/lib/libonnxruntime_providers_cuda.so
if ldd /usr/local/lib/libonnxruntime_providers_cuda.so | grep -q 'not found'; then
    echo "ONNX Runtime CUDA dependencies are missing. Complete system setup first." >&2
    exit 1
fi

mkdir -p "$HOME/.local/bin"

if [[ ! -f $workspace/px4_msgs/package.xml ]]; then
    git clone --depth 1 --branch release/1.16 \
        https://github.com/PX4/px4_msgs.git "$workspace/px4_msgs"
fi
if [[ $(git -C "$workspace/px4_msgs" branch --show-current) != release/1.16 ]]; then
    echo "px4_msgs must use release/1.16." >&2
    exit 1
fi

if [[ ! -e $px4_dir ]]; then
    git clone --depth 1 --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git "$px4_dir"
fi
if [[ $(git -C "$px4_dir" rev-parse HEAD) != $(git -C "$px4_dir" rev-parse 'v1.16.0^{commit}') ]]; then
    echo "$px4_dir must use v1.16.0; refusing to change an existing checkout." >&2
    exit 1
fi
git -C "$px4_dir" submodule update --init --recursive --depth 1 --jobs "$workers"

# PX4 v1.16 reads the NuttX tag list even for a POSIX SITL build. A shallow
# submodule clone omits these tags and otherwise fails in px_update_git_header.py.
nuttx_dir="$px4_dir/platforms/nuttx/NuttX/nuttx"
if ! git -C "$nuttx_dir" rev-parse --verify --quiet refs/tags/nuttx-12.12.0 >/dev/null; then
    git -C "$nuttx_dir" fetch --depth 1 origin tag nuttx-12.12.0
fi

# Keep NumPy compatible with the Ubuntu cv_bridge binary.
/usr/bin/python3 -m pip install --user -r "$px4_dir/Tools/setup/requirements.txt" 'numpy<2'

if [[ ! -e $agent_dir ]]; then
    git clone --depth 1 --branch v2.4.3 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$agent_dir"
fi
if [[ $(git -C "$agent_dir" rev-parse HEAD) != $(git -C "$agent_dir" rev-parse 'v2.4.3^{commit}') ]]; then
    echo "$agent_dir must use v2.4.3; refusing to change an existing checkout." >&2
    exit 1
fi
cmake -S "$agent_dir" -B "$agent_dir/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$agent_prefix"
cmake --build "$agent_dir/build" -j "$workers"
cmake --install "$agent_dir/build"

# Keep the Agent's Fast-DDS libraries out of the ROS nodes' library search path.
cat > "$HOME/.local/bin/MicroXRCEAgent" <<'EOF'
#!/usr/bin/env bash
agent_prefix="$HOME/.local/opt/microxrce-agent-2.4.3"
export LD_LIBRARY_PATH="$agent_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$agent_prefix/bin/MicroXRCEAgent" "$@"
EOF
chmod +x "$HOME/.local/bin/MicroXRCEAgent"

# Build against the system ROS underlay, without stale workspace/Conda paths.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="$workers"
export MAKEFLAGS="-j$workers"
cd "$workspace"
colcon build --symlink-install --packages-select px4_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
set +u
source "$workspace/install/setup.bash"
set -u
colcon build --symlink-install --parallel-workers "$workers" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

cp -- "$repo_dir/run_swarm.sh" "$px4_dir/run_swarm.sh"
cp -- "$repo_dir/assets/gazebo_world/grass_world.sdf" "$px4_dir/Tools/simulation/gz/worlds/"
chmod +x "$px4_dir/run_swarm.sh"

# The PNG controller waits for this estimator output before leaving TAKEOFF.
# Insert only the missing entry; YAML dumping would discard existing comments.
/usr/bin/python3 - "$px4_dir/src/modules/uxrce_dds_client/dds_topics.yaml" <<'PY'
from pathlib import Path
import sys
import yaml

path = Path(sys.argv[1])
source = path.read_bytes().decode("utf-8")
config = yaml.safe_load(source)
publications = config.get("publications") if isinstance(config, dict) else None
if not isinstance(publications, list):
    raise SystemExit(f"{path}: publications must be a YAML list")
topic = "/fmu/out/hover_thrust_estimate"
message_type = "px4_msgs::msg::HoverThrustEstimate"
matches = [entry for entry in publications
           if isinstance(entry, dict) and entry.get("topic") == topic]
if any(entry.get("type") != message_type for entry in matches):
    raise SystemExit(f"{path}: {topic} must use type {message_type}; refusing to overwrite")
if matches:
    print(f"DDS publication already configured: {topic}")
    sys.exit(0)

root = yaml.compose(source)
node = next(value for key, value in root.value if key.value == "publications")
if node.flow_style:
    index = node.start_mark.index + 1
    entry = f"{{topic: {topic}, type: {message_type}}}" + (", " if publications else "")
else:
    index = node.start_mark.index - node.start_mark.column
    indent = " " * node.start_mark.column
    newline = "\r\n" if "\r\n" in source else "\n"
    entry = f"{indent}- topic: {topic}{newline}{indent}  type: {message_type}{newline}{newline}"
path.write_bytes((source[:index] + entry + source[index:]).encode("utf-8"))
print(f"Added DDS publication: {topic}")
PY

# Reconfigure after installing Harmonic and discard cached Anaconda Qt/ZeroMQ
# paths. Build PX4 with the system tools, independently of an active Conda env.
px4_build_env=(env -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH -u PKG_CONFIG_PATH
    -u PYTHONPATH -u PYTHONHOME PATH=/usr/local/bin:/usr/bin:/bin)
if [[ -f $px4_dir/build/px4_sitl_default/CMakeCache.txt ]]; then
    "${px4_build_env[@]}" cmake -S "$px4_dir" -B "$px4_dir/build/px4_sitl_default" \
        -U 'Qt5*_DIR' -U ZeroMQ_DIR -DCMAKE_PREFIX_PATH=/usr \
        -DPYTHON_EXECUTABLE=/usr/bin/python3
fi
# v1.16's explicit gz_x500 target starts PX4 even when DONT_RUN is set.
# The default SITL build compiles the executable and Gazebo plugins only.
"${px4_build_env[@]}" make -C "$px4_dir" -j "$workers" px4_sitl_default

cat > "$workspace/env.sh" <<'EOF'
#!/usr/bin/env bash
# Source this file in every simulation terminal.
_aid_workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
source /opt/ros/humble/setup.bash
source "$_aid_workspace_dir/install/setup.bash"
export PATH="$HOME/.local/bin:$PATH"
unset _aid_workspace_dir
EOF
printf 'Workspace ready. Run: source %q\n' "$workspace/env.sh"
