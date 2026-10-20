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
    moveit_config: str = get_package_share_directory('assembly_station')

    with open(os.path.join(moveit_config, 'config', 'station_configuration.yaml')) as f:
        config = yaml.safe_load(f)
        station = config['robots']
        mechanisms = config['mechanism']

    controller_names: list[str] = (
        [f'{r["id"]}_arm_controller' for r in station]
        + [f'{r["id"]}_gripper_controller' for r in station if r.get('eoat')]
        + [f'{r}_controller' for r in mechanisms]
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    mujoco_model = LaunchConfiguration('mujoco_model')

    xacro_file: str = os.path.join(
        moveit_config, 'urdf', 'mujoco_station.urdf.xacro')
    controllers_file: str = os.path.join(
        moveit_config, 'config', 'station_controllers.yaml')
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('assembly_station'), 'config', 'station_moveit.rviz'])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' mujoco_model:=', mujoco_model,
        ]),
        value_type=str)

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
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mujoco_system'), 'scenes', 'station.xml'])),
        rsp, control_node, jsb, delay, rviz_node,
    ])
