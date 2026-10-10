from collections.abc import Callable
from pathlib import PosixPath
from typing import cast
from xml.etree.ElementTree import (
    Element,
    ElementTree,
    SubElement,
    indent,
    register_namespace,
)

from generator.models import EOAT, Robot, StationConfigurations


class StationXacroGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
        '_active_robot',
        '_added_includes',
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

    def _create_includes(self, model: str, is_eoat: bool) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        if model in self._added_includes:
            return

        self._added_includes.add(model)

        if is_eoat:
            eoat: EOAT = cast(EOAT, self._active_eoat)
            _package: str = eoat.xacro_package
            _filepath: str = str(eoat.xacro_path).split(_package)[-1]
        else:
            _package = robot.xacro_package
            _filepath = str(robot.xacro_path).split(_package)[-1]

        SubElement(
            self._root, 'xacro:include',
            filename=f'$(find {_package}){_filepath}')

    def _create_robot(self, config: StationConfigurations) -> None:
        _prefix: str = config.prefix
        has_eoat: bool = config.eoat is not None

        robot: Robot = self._get_robot(config.model)

        SubElement(
            self._root, f'xacro:{robot.xacro_macro}', prefix=f'{_prefix}_')
        _joint: Element = SubElement(
            self._root, 'joint',
            name=f'{_prefix}_{robot.base}', type='fixed')
        SubElement(_joint, 'parent', link='world')
        SubElement(_joint, 'child', link=f'{_prefix}_{robot.base}')
        SubElement(
            _joint, 'origin', xyz=config.robot_rpy,
            rpy=config.robot_rpy)

        if has_eoat:
            eoat: EOAT = self._get_eoat(config.eoat)  # type: ignore
            _xacro: Element = SubElement(
                self._root, f'xacro:{eoat.xacro_macro}', {
                    'prefix': f'{_prefix}_',
                    'parent': f'{_prefix}_{robot.flange}'
                })
            SubElement(
                _xacro, 'origin', xyz=config.eoat_xyz,  # type: ignore
                rpy=config.eoat_rpy)  # type: ignore

    def generate(self) -> None:

        for config in self._config:
            _model: str = config.model
            self._active_robot = self._get_robot(_model)
            self._create_includes(_model, False)

            is_eoat: bool = config.eoat is not None
            if is_eoat:
                _model = config.eoat  # type: ignore
                self._active_eoat = self._get_eoat(_model)
                self._create_includes(_model, True)

        SubElement(self._root, 'link', name='world')

        for config in self._config:
            self._create_robot(config)

        file: PosixPath = self._output_path / 'urdf' / 'station.urdf.xacro'
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(self._root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=True)
