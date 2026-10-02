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


def generate_launch_description() -> LaunchDescription:
    prefix = LaunchConfiguration("prefix")
    gui = LaunchConfiguration("gui")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("dobot_description"), "urdf",
         "cr5", "cr5.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("dobot_description"), "rviz", "view_cr5.rviz"]
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ",
                xacro_file, " prefix:=", prefix]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "prefix", default_value="",
                description="Joint/link prefix"),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Use joint_state_publisher_gui"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=UnlessCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
