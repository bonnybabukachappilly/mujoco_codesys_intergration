from collections.abc import Callable
from pathlib import PosixPath
from typing import cast

import yaml
from generator.models import Robot, StationConfigurations


class MoveitControllerGenerator:
    __slots__: tuple[str, ...] = (
        '_active_robot',
        '_config',
        '_controller',
        '_get_robot',
        '_output_path'
    )

    def __init__(
        self,  config: list[StationConfigurations],
            get_robot: Callable[[str], Robot], output_path: PosixPath) -> None:
        self._controller: dict = {
            'moveit_manage_controllers': False,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'moveit_simple_controller_manager': {
                'controller_names': []
            },
        }

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._output_path: PosixPath = output_path

        self._active_robot: Robot | None = None

    def _create_robot_controller(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        joint_names: list[str] = [joint.name for joint in robot.joints]

        _controller = {
            f'{prefix}_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    f'{prefix}_{joint}' for joint in joint_names
                ]
            }
        }

        self._controller['moveit_simple_controller_manager'] |= _controller

    def generate(self) -> None:
        _controller_names: list[str] = []

        for config in self._config:
            self._active_robot = self._get_robot(config.model)
            _controller_names.append(
                f'{config.prefix}_arm_controller'
            )

            self._create_robot_controller(config.prefix)

        self._controller['moveit_simple_controller_manager']['controller_names'] = _controller_names

        file: PosixPath = self._output_path / 'config' / 'cell_moveit_controllers.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._controller, f, sort_keys=False,
                default_flow_style=False
            )
