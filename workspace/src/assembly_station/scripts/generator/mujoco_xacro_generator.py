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


class MujocoXacroGenerator:
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
        SubElement(
            self._root, 'xacro:include',
            filename='$(find assembly_station)/urdf/station.urdf.xacro')

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat
        self._output_path: PosixPath = output_path

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None
        self._added_includes: set[str] = set()

    def _create_includes(self, model: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        if model in self._added_includes:
            return

        self._added_includes.add(model)

        SubElement(
            self._root, 'xacro:include',
            file_name=f'$(find {robot.xacro_package})/{robot.ros_2_control}')

    def _create_ros_control(self, root: Element) -> None:
        _hardware: Element = SubElement(root, 'hardware')
        _plugin: Element = SubElement(_hardware, 'plugin')
        _plugin.text = 'mujoco_ros2_control/MujocoSystemInterface'

        _model: Element = SubElement(_hardware, 'param', name='mujoco_model')
        _model.text = '$(arg mujoco_model)'

        _param: Element = SubElement(
            _hardware, 'param', name='auto_register_cameras')
        _param.text = 'false'

    def _add_joints(self, root: Element, prefix: str, has_eoat: bool) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        joint_names: list[str] = [joint.name for joint in robot.joints]

        for joint in joint_names:
            SubElement(
                root, f'xacro:{robot.ros_2_control_macro}',
                joint_name=f'{prefix}_{joint}')

        if not has_eoat:
            return

        eoat: EOAT = cast(EOAT, self._active_eoat)

        for joint in eoat.control_joint:
            SubElement(
                root, f'xacro:{robot.ros_2_control_macro}',
                joint_name=f'{prefix}_{joint}')

    def generate(self) -> None:
        for config in self._config:
            self._active_robot = self._get_robot(config.model)

            self._create_includes(config.model)

        _control: Element = SubElement(
            self._root, 'ros2_control', name='station_system', type='system')

        self._create_ros_control(_control)

        for config in self._config:
            self._active_robot = self._get_robot(config.model)

            has_eoat: bool = config.eoat is not None

            if has_eoat:
                self._active_eoat = self._get_eoat(config.eoat)  # type: ignore

            self._add_joints(_control, config.prefix, has_eoat)

        file: PosixPath = self._output_path / 'urdf' / 'station_mujoco.urdf.xacro'
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(self._root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=True)
