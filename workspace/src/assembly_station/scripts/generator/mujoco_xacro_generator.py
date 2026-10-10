from collections.abc import Callable
from pathlib import PosixPath
from typing import cast
from xml.etree.ElementTree import Element, ElementTree, SubElement, indent, register_namespace

from generator.models import EOAT, Robot, StationConfigurations


class SRDFGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
        '_added_includes',
        '_active_robot',
        '_config',
        '_get_eoat',
        '_get_robot',
        '_output_path',
        '_root'
    )

    def __init__(
            self, config: list[StationConfigurations],
            get_robot: Callable[[str], Robot],
            get_eoat: Callable[[str], EOAT], output_path: PosixPath) -> None:

        register_namespace("xacro", "http://www.ros.org/wiki/xacro")
        self._root: Element = Element('robot', {
            'name': 'cell',
            "xmlns:xacro": "http://www.ros.org/wiki/xacro"
        })

        SubElement(self._root, 'xacro:arg', name='mujoco_model', default='')

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat
        self._output_path: PosixPath = output_path

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None
        self._added_includes: set[str] = set()

    def _create_includes(self, model: str, model_type: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)
        eoat: EOAT = cast(EOAT, self._active_eoat)

        if model in self._added_includes:
            return

        self._added_includes.add(model)

        match model_type:
            case 'robot':
                _file_name = robot.xacro_file
                # SubElement(
                #     self._root, 'xacro:include',
                #     file_name=f'$(find {})')

    def generate(self) -> None:

        for config in self._config:
            self._active_robot = self._get_robot(config.model)

        file: PosixPath = self._output_path / 'config' / 'station.srdf'
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(self._root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=True)
