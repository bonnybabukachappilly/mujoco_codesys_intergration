import copy
import os
from collections.abc import Callable
from pathlib import Path, PosixPath
from typing import cast
from xml.etree.ElementTree import (
    Element,
    ElementTree,
    SubElement,
    indent,
    parse,
)

from generator.models import EOAT, Robot, StationConfigurations


class MujocoXmlGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
        '_active_robot',
        '_config',
        '_generated_robots',
        '_get_eoat',
        '_get_robot',
    )

    def __init__(
            self, config: list[StationConfigurations],
            get_robot: Callable[[str], Robot],
            get_eoat: Callable[[str], EOAT]) -> None:

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None

        self._generated_robots: set[str] = set()

    def _resolve_include(self, parent: Element, dir: Path, checked: set[Path]) -> None:
        for _child in list(parent):
            if _child.tag == "include" and "file" in _child.attrib:
                _include_path: Path = (dir / _child.attrib["file"]).resolve()

                if _include_path in checked:
                    parent.remove(_child)
                    continue

                if _include_path.exists():
                    checked.add(_include_path)
                    included_root: Element = parse(_include_path).getroot()

                    index: int = list(parent).index(_child)
                    parent.remove(_child)

                    elements_to_insert: list[Element] = (
                        list(included_root)
                        if included_root.tag == "mujoco"
                        else [included_root]
                    )

                    for offset, sub_element in enumerate(elements_to_insert):
                        parent.insert(index + offset, sub_element)
                        self._resolve_include(
                            sub_element, _include_path.parent, checked)
            else:
                self._resolve_include(_child, dir, checked)

    def _save_xml_file(self, file_name: str, path: PosixPath, root: Element) -> None:
        file: PosixPath = path / file_name
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=True)

    def _flatten_xml(self, root: ElementTree, dir: PosixPath) -> Element:
        _root: Element = copy.deepcopy(root.getroot())  # type: ignore
        _dir: PosixPath = dir.parent
        _checked: set[Path] = {dir.resolve()}

        self._resolve_include(_root, _dir, _checked)

        return _root

    def _create_robot_xml(self, has_eoat: bool) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        file_name: str = f'{robot.model}.xml'

        if has_eoat:
            eoat: EOAT = cast(EOAT, self._active_eoat)
            file_name = f'{robot.model}_{eoat.model}.xml'

        if file_name in self._generated_robots:
            return

        self._generated_robots.add(file_name)

        _root: Element = self._flatten_xml(
            robot.mujoco_file, robot.mujoco_path)

        if not has_eoat:
            print(f"HELLOOO: {file_name}")
            self._save_xml_file(file_name, robot.mujoco_path.parent,  _root)
            return

        _asset: Element | None = _root.find(".//asset")
        _tool: Element | None = _root.find(".//frame[@name='tool_mount']")
        if _asset is None or _tool is None:
            raise RuntimeError(
                'Unable to find xml tag in robot xml, Please check the xml data')

        _rel_path: str = os.path.relpath(
            eoat.mujoco_path, robot.mujoco_path.parent)  # type: ignore

        SubElement(
            _asset,
            'model',
            {
                'name': 'eoat_tool',
                'file': _rel_path,
            },
        )

        SubElement(
            _tool,
            'attach',
            {
                'model': 'eoat_tool',
                'body': eoat.base,  # type: ignore
                'prefix': ''
            },
        )

        print(f"HII: {file_name}")
        self._save_xml_file(file_name, robot.mujoco_path.parent, _root)

    def generate(self) -> None:

        for config in self._config:
            self._active_robot = self._get_robot(config.model)

            has_eoat: bool = config.eoat is not None

            if has_eoat:
                self._active_eoat = self._get_eoat(config.eoat)  # type: ignore

            self._create_robot_xml(has_eoat)
