from collections.abc import Callable
from pathlib import PosixPath
from typing import cast

import yaml
from generator.models import Robot, StationConfigurations
from generator.utils import NoAliasDumper


class JointLimitGenerator:
    __slots__: tuple[str, ...] = (
        '_active_robot',
        '_config',
        '_get_robot',
        '_joint_limit',
        '_output_path'
    )

    def __init__(
        self,  config: list[StationConfigurations],
            get_robot: Callable[[str], Robot], output_path: PosixPath) -> None:
        self._joint_limit: dict = {
            'default_velocity_scaling_factor': 0.1,
            'default_acceleration_scaling_factor': 0.1,
            'joint_limits': {}
        }

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._output_path: PosixPath = output_path

        self._active_robot: Robot | None = None

    def _create_joint_limit(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        _joint_limit: dict = {}

        for joint in robot.joints:
            _limit: dict = {
                f'{prefix}_{joint.name}': {
                    'has_velocity_limits': True,
                    'max_velocity': joint.max_velocity,
                    'has_acceleration_limits': True,
                    'max_acceleration': joint.max_acceleration
                }
            }

            _joint_limit |= _limit

        self._joint_limit['joint_limits'] |= _joint_limit

    def generate(self) -> None:

        for config in self._config:
            self._active_robot = self._get_robot(config.model)
            self._create_joint_limit(config.prefix)

        file: PosixPath = self._output_path / 'config' / 'cell_joint_limits.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._joint_limit, f, sort_keys=False,
                default_flow_style=False, Dumper=NoAliasDumper)
