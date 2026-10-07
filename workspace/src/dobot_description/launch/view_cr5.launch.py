from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = 'dobot_description'


def generate_launch_description() -> LaunchDescription:

    prefix = LaunchConfiguration('prefix')
    gui = LaunchConfiguration('gui')

    pkg = FindPackageShare(PACKAGE_NAME)

    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'cr5', 'cr5.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'view_cr5.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ',
                xacro_file, ' prefix:=', prefix]),
        value_type=str,
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='Joint/link prefix'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Use joint_state_publisher_gui'
        )
    )

    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(gui),
        )
    )

    ld.add_action(
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=UnlessCondition(gui),
        )
    )

    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
    )

    return ld
