#!/usr/bin/env bash
# System packages for Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic.
# Run this script with sudo; source downloads and workspace builds run as the user.
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "Run: sudo bash \"$0\"" >&2
    exit 1
fi

source /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 22.04 || $(dpkg --print-architecture) != amd64 ]]; then
    echo "This installer requires Ubuntu 22.04 amd64." >&2
    exit 1
fi

log_dir=$(mktemp -d /var/tmp/aid-system-setup.XXXXXX)
exec > >(tee "$log_dir/install.log") 2>&1
trap 'echo "Installation failed at line $LINENO; log: $log_dir/install.log" >&2' ERR
dpkg-query -W -f='${db:Status-Abbrev} ${Package} ${Version}\n' > "$log_dir/packages-before.txt"

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y \
    build-essential cmake ninja-build git curl wget gnupg ca-certificates \
    python3-pip python3-dev python3-venv python3-setuptools python3-wheel \
    python3-colcon-common-extensions python3-rosdep \
    python3-numpy python3-opencv \
    libopencv-dev libeigen3-dev libgeographic-dev \
    ros-humble-cv-bridge apache2-utils libunwind-dev libasio-dev libtinyxml2-dev \
    astyle ccache cppcheck file gdb lcov libssl-dev libxml2-dev libxml2-utils \
    rsync shellcheck unzip zip bc pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

curl -fsSL --retry 3 --retry-all-errors https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' \
    > /etc/apt/sources.list.d/gazebo-stable.list
apt-get update

# Download and resolve the entire replacement before uninstalling the old stack.
remove_packages=()
for package in \
    gazebo gazebo-common gazebo-plugin-base libgazebo-dev libgazebo11 \
    ros-humble-gazebo-dev ros-humble-gazebo-msgs ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces \
    ros-humble-ros-gz-sim; do
    if [[ $(dpkg-query -W -f='${db:Status-Abbrev}' "$package" 2>/dev/null || true) == ii* ]]; then
        remove_packages+=("${package}-")
    fi
done
apt-get install -y --download-only gz-harmonic ros-humble-ros-gzharmonic "${remove_packages[@]}"
apt-get install -y gz-harmonic ros-humble-ros-gzharmonic "${remove_packages[@]}"

# Install the C++ headers and CUDA provider together from one pinned release.
ort_name=onnxruntime-linux-x64-gpu_cuda13-1.27.0
ort_stage=$(mktemp -d /var/tmp/aid-onnxruntime.XXXXXX)
cache_home=$(getent passwd "${SUDO_USER:-root}" | cut -d: -f6)
cached_archive="$cache_home/Downloads/aid-deps/${ort_name}.tgz"
if [[ -f $cached_archive ]] && gzip -t "$cached_archive"; then
    cp -- "$cached_archive" "$ort_stage/ort.tgz"
else
    curl -fL --retry 5 --retry-all-errors --connect-timeout 20 \
        "https://github.com/microsoft/onnxruntime/releases/download/v1.27.0/${ort_name}.tgz" \
        -o "$ort_stage/ort.tgz"
fi
tar -xzf "$ort_stage/ort.tgz" --no-same-owner -C "$ort_stage"
cp -a "$ort_stage/$ort_name/include/." /usr/local/include/
cp -a "$ort_stage/$ort_name/lib/." /usr/local/lib/
ldconfig

ldd /usr/local/lib/libonnxruntime_providers_cuda.so | tee "$log_dir/onnxruntime-ldd.txt"
if grep -q 'not found' "$log_dir/onnxruntime-ldd.txt"; then
    echo "ONNX Runtime has missing runtime dependencies; see $log_dir/onnxruntime-ldd.txt" >&2
    exit 1
fi
gz sim --versions
echo "System dependencies installed. Log: $log_dir/install.log"
