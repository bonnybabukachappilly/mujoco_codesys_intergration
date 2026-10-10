from collections.abc import Callable
from pathlib import PosixPath
from typing import cast

import yaml
from generator.models import EOAT, Robot, StationConfigurations

JTC: dict = {
    'command_interfaces': ['position'],
    'state_interfaces': ['position', 'velocity'],
    'state_publish_rate': 50.0,
    'action_monitor_rate': 20.0,
    'allow_partial_joints_goal': False,
    'allow_nonzero_velocity_at_trajectory_end': False,
    'constraints': {
        'stopped_velocity_tolerance': 0.01,
        'goal_time': 0.0,
    }
}

FCC: dict = {
    'interface_name': 'position'
}


def get_controller(name: str) -> dict:
    _controller: dict = {
        'JointTrajectoryController': JTC,
        'ForwardCommandController': FCC
    }

    _request: str = name.split('/')[-1]

    if _request not in _controller:
        raise RuntimeError(
            f'Unable to find controller: {name}, please add new controller in script')

    return _controller[_request]


class CellControllerGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
        '_active_robot',
        '_config',
        '_controller',
        '_get_eoat',
        '_get_robot',
        '_output_path',
    )

    def __init__(
            self, config: list[StationConfigurations],
            get_robot: Callable[[str], Robot],
            get_eoat: Callable[[str], EOAT], output_path: PosixPath) -> None:

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat
        self._output_path: PosixPath = output_path

        self._controller = {
            'controller_manager': {
                'ros__parameters': {
                    'update_rate': 500,
                    'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster'
                }
            }
        }

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None

    def _create_ros_params(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        _controller: dict[str, dict[str, str]] = {
            f'{prefix}_arm_controller': {
                'type': robot.controller_type
            }
        }

        self._controller['controller_manager']['ros__parameters'] |= _controller

    def _create_arm_controller(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)
        joint_names: list[str] = [joint.name for joint in robot.joints]

        _controller: dict = {
            f'{prefix}_arm_controller': {
                'ros__parameters': {
                    'joints': joint_names
                }
            }
        }

        _controller[f'{prefix}_arm_controller']['ros__parameters'] |= get_controller(
            robot.controller_type)

        self._controller |= _controller

    def _create_eoat_controller(self, prefix: str) -> None:
        eoat: EOAT = cast(EOAT, self._active_eoat)
        joint_names: list[str] = [joint.name for joint in eoat.joints]

        _controller: dict = {
            f'{prefix}_gripper_controller': {
                'ros__parameters': {
                    'joints': joint_names
                }
            }
        }

        _controller[f'{prefix}_gripper_controller']['ros__parameters'] |= get_controller(
            eoat.controller_type)

        self._controller |= _controller

    def generate(self) -> None:
        for config in self._config:
            self._active_robot = self._get_robot(config.model)

            has_eoat: bool = config.eoat is not None
            _prefix: str = config.prefix

            self._create_ros_params(_prefix)
            self._create_arm_controller(_prefix)

            if has_eoat:
                self._active_eoat = self._get_eoat(config.eoat)  # type: ignore
                self._create_eoat_controller(_prefix)

        file: PosixPath = self._output_path / 'config' / 'cell_controllers.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._controller, f, sort_keys=False,
                default_flow_style=False
            )
