from pathlib import PosixPath

import yaml
from generator.models import StationConfigurations

HUMBLE_ADAPTERS: list[str] = [
    'default_planner_request_adapters/AddTimeOptimalParameterization',
    'default_planner_request_adapters/ResolveConstraintFrames',
    'default_planner_request_adapters/FixWorkspaceBounds',
    'default_planner_request_adapters/FixStartStateBounds',
    'default_planner_request_adapters/FixStartStateCollision',
    'default_planner_request_adapters/FixStartStatePathConstraints',
]


class OMPLGenerator:
    __slots__: tuple[str, ...] = (
        '_active_robot',
        '_config',
        '_ompl',
        '_output_path',
    )

    def __init__(
            self, config: list[StationConfigurations], output_path: PosixPath) -> None:
        self._ompl: dict = {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': HUMBLE_ADAPTERS,
            'start_state_max_bounds_error': 0.1,
            'planner_configs': {
                'RRTConnectkConfigDefault': {
                    'type': 'geometric::RRTConnect',
                    'range': 0.0,
                }
            },
        }

        self._config: list[StationConfigurations] = config
        self._output_path: PosixPath = output_path

    def _create_ompl(self, prefix: str, shared_list: list) -> None:
        robot = {
            f'{prefix}_arm': {
                'default_planner_config': 'RRTConnectkConfigDefault',
                'planner_configs': shared_list,
            }
        }

        self._ompl |= robot

    def generate(self) -> None:
        shared_planner_configs: list[str] = ['RRTConnectkConfigDefault']

        for config in self._config:
            self._create_ompl(config.prefix, shared_planner_configs)

        self._ompl['all_arms'] = {
            'default_planner_config': 'RRTConnectkConfigDefault',
            'planner_configs': shared_planner_configs,
        }

        config_dir: PosixPath = self._output_path / 'config'
        config_dir.mkdir(parents=True, exist_ok=True)

        file: PosixPath = config_dir / 'cell_ompl_planning.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._ompl, f, sort_keys=False, default_flow_style=False
            )
