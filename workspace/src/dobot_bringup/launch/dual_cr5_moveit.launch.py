import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup = get_package_share_directory("dobot_bringup")
    moveit_pkg = get_package_share_directory("dobot_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("dual_cr5", package_name="dobot_moveit_config")
        .robot_description(file_path=os.path.join(bringup, "urdf", "dual_cr5.urdf.xacro"))
        .robot_description_semantic(file_path="config/dual_cr5.srdf")
        .robot_description_kinematics(file_path="config/dual_kinematics.yaml")
        .joint_limits(file_path="config/dual_joint_limits.yaml")
        .trajectory_execution(file_path="config/dual_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    sim_time = {"use_sim_time": True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, "launch", "dual_cr5_sim.launch.py")),
        launch_arguments={"rviz": "false"}.items(),
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), sim_time],
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="log",
        arguments=[
            "-d", os.path.join(moveit_pkg, "config", "dual_moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_time,
        ],
    )
    return LaunchDescription([sim, move_group, rviz])
