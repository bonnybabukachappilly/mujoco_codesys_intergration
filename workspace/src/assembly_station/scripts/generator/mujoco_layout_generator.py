from collections.abc import Callable
from pathlib import PosixPath
from typing import cast
from xml.etree.ElementTree import (
    Element,
    ElementTree,
    SubElement,
    indent,
)

from generator.models import EOAT, Robot, StationConfigurations

ROBOT_MOUNT_GEOM_CLASS = 'robot_support'
ROBOT_MOUNT_MATERIAL = 'support_robot_mat'
ROBOT_MOUNT_MESH = 'support_robot'
ROBOT_MOUNT = '0 0 0.595'  # wrt base, mostly height


class MujocoLayoutGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
        '_active_robot',
        '_added_includes',
        '_asset',
        '_config',
        '_get_eoat',
        '_get_robot',
        '_output_path',
        '_root',
        '_worldbody'
    )

    def __init__(
            self, config: list[StationConfigurations],
            get_robot: Callable[[str], Robot],
            get_eoat: Callable[[str], EOAT], output_path: PosixPath) -> None:

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None

        self._added_includes: set[str] = set()

        self._output_path: PosixPath = output_path

        self._root: Element = Element('mujoco', model='robot_layout')

        SubElement(self._root, 'compiler', angle='radian', eulerseq='XYZ')

        self._asset: Element = SubElement(self._root, 'asset')
        self._create_default()

        self._worldbody: Element = SubElement(self._root, 'worldbody')

    def _create_includes(self, model: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)
        if model in self._added_includes:
            return

        self._added_includes.add(model)

        SubElement(
            self._asset, 'model', name=model,
            file=str(robot.mujoco_path.parent / f'{model}.xml'))

    def _mount_robots(self, config: StationConfigurations, model: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)
        prefix: str = config.prefix

        _body: Element = SubElement(
            self._worldbody, 'body',
            name=f'support_{prefix}_robot',
            pos=config.robot_xyz,
            euler=config.robot_rpy)

        SubElement(_body, 'geom', {
            'class': ROBOT_MOUNT_GEOM_CLASS
        })

        _frame: Element = SubElement(
            _body, 'frame', name=f'{prefix}_mount', pos=ROBOT_MOUNT)

        SubElement(
            _frame, 'attach', model=model,
            body=robot.base, prefix=f'{prefix}_')

    def _create_default(self) -> None:
        _default: Element = SubElement(self._root, 'default')
        _def: Element = SubElement(_default, 'default', {
            'class': ROBOT_MOUNT_GEOM_CLASS
        })
        SubElement(
            _def, 'geom', type='mesh',
            material=ROBOT_MOUNT_MATERIAL, mesh=ROBOT_MOUNT_MESH)

    def generate(self) -> None:

        for config in self._config:
            self._active_robot = self._get_robot(config.model)
            _model: str = self._active_robot.model

            has_eoat: bool = config.eoat is not None

            if has_eoat:
                self._active_eoat = self._get_eoat(config.eoat)  # type: ignore
                _model += f'_{config.eoat}'

            self._create_includes(_model)
            self._mount_robots(config, _model)

        file: PosixPath = self._output_path / 'models' / 'robot.xml'
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(self._root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=False)
