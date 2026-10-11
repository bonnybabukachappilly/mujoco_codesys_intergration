from pathlib import PosixPath

import yaml
from generator.utils import NoAliasDumper


class PlizGenerator:
    __slots__: tuple[str, ...] = (
        '_output_path',
        '_pliz'
    )

    def __init__(self, output_path: PosixPath) -> None:
        self._output_path: PosixPath = output_path

    def generate(self) -> None:
        self._pliz: dict[str, dict[str, float]] = {
            'cartesian_limits': {
                'max_trans_vel': 1.0,
                'max_trans_acc': 2.25,
                'max_trans_dec': -5.0,
                'max_rot_vel': 1.57
            }
        }

        file: PosixPath = self._output_path / 'config' / 'pilz_cartesian_limits.yaml'
        with open(file, 'w') as f:
            yaml.dump(
                self._pliz, f, sort_keys=False,
                default_flow_style=False, Dumper=NoAliasDumper)
