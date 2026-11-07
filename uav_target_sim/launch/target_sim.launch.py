from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'motion_mode',
        default_value='circle',
        description='Target motion mode: hover, spin, circle, sinusoidal, random_walk'
    )
    numeric_args = [
        DeclareLaunchArgument('max_range', default_value='10.0',
                              description='Boundary radius for sinusoidal/random_walk (m)'),
        DeclareLaunchArgument('takeoff_height', default_value='5.0',
                              description='Height above the local NED origin (m, positive)'),
        DeclareLaunchArgument('hover_x', default_value='0.0',
                              description='Fixed local NED x for hover/spin (m)'),
        DeclareLaunchArgument('hover_y', default_value='0.0',
                              description='Fixed local NED y for hover/spin (m)'),
        DeclareLaunchArgument('yaw_rate', default_value='0.05',
                              description='Spin yaw rate (rad/s, signed)'),
    ]

    target_node = Node(
        package='uav_target_sim',
        executable='uav_target_sim',
        name='uav_target_sim',
        output='screen',
        parameters=[{
            'motion_mode': LaunchConfiguration('motion_mode'),
            **{
                arg.name: ParameterValue(LaunchConfiguration(arg.name), value_type=float)
                for arg in numeric_args
            },
        }],
    )

    return LaunchDescription([mode_arg, *numeric_args, target_node])
