import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    moveit_cfg = get_package_share_directory('assembly_station')

    mujoco_model = os.path.join(
        get_package_share_directory('mujoco_system'), 'scenes', 'cell.xml')

    def cfg(f):
        return os.path.join(moveit_cfg, 'config', f)

    moveit_config = (
        MoveItConfigsBuilder('cell', package_name='assembly_station')
        .robot_description(
            file_path=os.path.join(
                moveit_cfg, 'urdf', 'station_mujoco.urdf.xacro'),
            mappings={'mujoco_model': mujoco_model})
        .robot_description_semantic(file_path=cfg('station.srdf'))
        .robot_description_kinematics(file_path=cfg('cell_kinematics.yaml'))
        .joint_limits(file_path=cfg('cell_joint_limits.yaml'))
        .trajectory_execution(file_path=cfg('cell_moveit_controllers.yaml'))
        .pilz_cartesian_limits(file_path=cfg('pilz_cartesian_limits.yaml'))
        .to_moveit_configs()
    )

    with open(cfg('cell_ompl_planning.yaml')) as f:
        ompl = yaml.safe_load(f)
    moveit_config.planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl,
    }
    sim_time = {'use_sim_time': True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_cfg, 'launch', 'cell_sim.launch.py')),
        launch_arguments={'rviz': 'false'}.items(),
    )
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim_time],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', cfg('cell_moveit.rviz')],
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
