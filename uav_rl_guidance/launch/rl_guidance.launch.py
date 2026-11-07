"""Launch RL guidance; only explicit launch arguments override the YAML."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_rl_guidance'))

    def start_node(context):
        overrides = {}
        for name, value_type in {
            'fallback_png': bool,
            'bench_test': bool,
            'csv_path': str,
            'record_csv': bool,
            'max_speed': float,
            'max_accel_xy': float,
            'max_accel_z': float,
            'max_vertical_speed': float,
            'max_yaw_rate': float,
            'max_yaw_accel': float,
        }.items():
            if LaunchConfiguration(name).perform(context):
                overrides[name] = ParameterValue(
                    LaunchConfiguration(name), value_type=value_type)
        return [Node(
            package='uav_rl_guidance',
            executable='uav_rl_guidance',
            name='uav_rl_guidance',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), overrides],
            emulate_tty=True,
        )]

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(share / 'config/params.yaml')),
        DeclareLaunchArgument(
            'fallback_png', default_value='',
            description='Override YAML: true uses built-in PNG baseline'),
        DeclareLaunchArgument(
            'bench_test', default_value='',
            description='Override YAML: true skips takeoff for bench testing'),
        DeclareLaunchArgument(
            'csv_path', default_value='',
            description='Override YAML optional CSV export path'),
        DeclareLaunchArgument(
            'record_csv', default_value='',
            description='Enable optional CSV export; default uses ROS logs'),
        DeclareLaunchArgument('max_speed', default_value='', description='Final total speed limit (m/s)'),
        DeclareLaunchArgument('max_accel_xy', default_value='', description='Horizontal command slew limit (m/s^2)'),
        DeclareLaunchArgument('max_accel_z', default_value='', description='Vertical command slew limit (m/s^2)'),
        DeclareLaunchArgument('max_vertical_speed', default_value='', description='Vertical speed limit (m/s)'),
        DeclareLaunchArgument('max_yaw_rate', default_value='', description='Yaw rate limit (rad/s)'),
        DeclareLaunchArgument('max_yaw_accel', default_value='', description='Yaw acceleration limit (rad/s^2)'),
        OpaqueFunction(function=start_node),
    ])
