from pathlib import PosixPath

import yaml
from generator.models import StationConfigurations
from generator.utils import NoAliasDumper


class KinematicsGenerator:
    __slots__: tuple[str, ...] = (
        '_config',
        '_kinematics',
        '_output_path'
    )

    def __init__(
        self,  config: list[StationConfigurations],
            output_path: PosixPath) -> None:
        self._kinematics: dict = {}

        self._config: list[StationConfigurations] = config
        self._output_path: PosixPath = output_path

    def _create_kinematics(self, prefix: str) -> None:
        _kinematics: dict = {
            f'{prefix}_arm': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.05
            }
        }

        self._kinematics |= _kinematics

    def generate(self) -> None:

        for config in self._config:
            self._create_kinematics(config.prefix)

        file: PosixPath = self._output_path / 'config' / 'cell_kinematics.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._kinematics, f, sort_keys=False,
                default_flow_style=False, Dumper=NoAliasDumper)
