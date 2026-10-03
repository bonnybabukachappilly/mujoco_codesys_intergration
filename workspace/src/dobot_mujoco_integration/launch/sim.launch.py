import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
import xacro
from ament_index_python.packages import get_package_share_directory


def setup(context, *args, **kwargs):
    pkg = get_package_share_directory("my_cell")
    from my_cell.build_scene import build   # or call the script via subprocess
    build(os.path.join(pkg, "scenes/main.xml"),
          os.path.join(pkg, "grippers/pgc_50_35/gripper.xml"),
          os.path.join(pkg, "scenes/main_with_gripper.xml"))

    robot_description = xacro.process_file(
        os.path.join(pkg, "urdf/cell.urdf.xacro")).toxml()
    controllers = os.path.join(pkg, "config/controllers.yaml")

    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="mujoco_ros2_control", executable="ros2_control_node",
             output="both",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}, controllers]),
        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller"]),
        Node(package="controller_manager", executable="spawner",
             arguments=["gripper_controller"]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=setup)])
