import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    moveit_config: str = get_package_share_directory('cell_moveit_config')
    # mujoco_share: str = get_package_share_directory('mujoco_system')

    with open(os.path.join(moveit_config, 'config', 'cell.yaml')) as f:
        cell = yaml.safe_load(f)['robots']

    controller_names: list[str] = [
        f'{r["name"]}_arm_controller' for r in cell] + \
        [f'{r["name"]}_gripper_controller' for r in cell if r['gripper']]

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    xacro_file: str = os.path.join(
        moveit_config, 'urdf', 'cell_mujoco.urdf.xacro')
    controllers_file: str = os.path.join(
        moveit_config, 'config', 'cell_controllers.yaml')
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('dobot_cr5_description'), 'rviz', 'view_cr5.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ], output='screen')

    control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        output='both',
        parameters=[
            {
                'use_sim_time': use_sim_time
            }, controllers_file],
        remappings=[('~/robot_description', '/robot_description')])

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'])

    others = Node(
        package='controller_manager',
        executable='spawner',
        arguments=controller_names + ['-c', '/controller_manager'])

    delay = RegisterEventHandler(OnProcessExit(
        target_action=jsb, on_exit=[others]))

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        rsp, control_node, jsb, delay, rviz_node,
    ])
