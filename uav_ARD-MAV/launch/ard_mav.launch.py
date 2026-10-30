"""Start the C++ tracker with the packaged parameters and models."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("uav_ard_mav"))

    def start_node(context):
        overrides = {}
        for name in ("provider", "show"):
            value = LaunchConfiguration(name).perform(context)
            if value:
                overrides[name] = (
                    ParameterValue(LaunchConfiguration(name), value_type=bool)
                    if name == "show"
                    else value
                )
        return [
            Node(
                package="uav_ard_mav",
                executable="uav_ard_mav",
                name="uav_ard_mav",
                output="screen",
                parameters=[LaunchConfiguration("params_file"), overrides],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config/params.yaml")
            ),
            DeclareLaunchArgument(
                "provider",
                default_value="",
                description="Override YAML cpu/cuda provider",
            ),
            DeclareLaunchArgument(
                "show", default_value="", description="Override YAML display setting"
            ),
            OpaqueFunction(function=start_node),
        ]
    )
