#!/usr/bin/env bash
# Source this file in every simulation terminal.
_aid_workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
source /opt/ros/humble/setup.bash
source "$_aid_workspace_dir/install/setup.bash"
export PATH="$HOME/.local/bin:$PATH"
unset _aid_workspace_dir
