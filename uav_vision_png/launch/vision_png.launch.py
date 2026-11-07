"""Launch visual PNG; only explicit launch arguments override the YAML."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_vision_png'))

    def start_node(context):
        overrides = {}
        for name, value_type in {
            'speed_cmd': float,
            'N_png': float,
            'csv_path': str,
            'record_csv': bool,
        }.items():
            if LaunchConfiguration(name).perform(context):
                overrides[name] = ParameterValue(
                    LaunchConfiguration(name), value_type=value_type)
        return [Node(
            package='uav_vision_png',
            executable='uav_vision_png',
            name='uav_vision_png',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), overrides],
            emulate_tty=True,
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config/params.yaml')),
        DeclareLaunchArgument(
            'speed_cmd', default_value='',
            description='Override YAML total speed limit (m/s)'),
        DeclareLaunchArgument(
            'N_png', default_value='',
            description='Override YAML navigation gain'),
        DeclareLaunchArgument(
            'csv_path', default_value='',
            description='Override YAML statistics CSV path'),
        DeclareLaunchArgument(
            'record_csv', default_value='',
            description='Enable optional CSV export; default uses ROS logs'),
        OpaqueFunction(function=start_node),
    ])
