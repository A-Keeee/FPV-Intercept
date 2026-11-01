"""Run one full-frame C++ YOLO model with the official Ultralytics tracker."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("uav_yolo_track"))

    def start(context):
        overrides = {}
        for name in ("show", "provider", "python_executable"):
            if LaunchConfiguration(name).perform(context):
                overrides[name] = (
                    ParameterValue(LaunchConfiguration(name), value_type=bool)
                    if name == "show"
                    else LaunchConfiguration(name)
                )
        return [
            Node(
                package="uav_yolo_track",
                executable="uav_yolo_single_track",
                name="uav_yolo_single_track",
                output="screen",
                parameters=[LaunchConfiguration("params_file"), overrides],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config/single_model.yaml")
            ),
            DeclareLaunchArgument("show", default_value=""),
            DeclareLaunchArgument("provider", default_value=""),
            DeclareLaunchArgument("python_executable", default_value=""),
            OpaqueFunction(function=start),
        ]
    )
