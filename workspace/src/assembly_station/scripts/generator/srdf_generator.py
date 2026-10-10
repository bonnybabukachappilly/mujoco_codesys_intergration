from collections.abc import Callable
from pathlib import PosixPath
from typing import cast
from xml.etree.ElementTree import Element, ElementTree, SubElement, indent

from generator.models import EOAT, Robot, StationConfigurations


class SRDFGenerator:
    __slots__: tuple[str, ...] = (
        '_active_eoat',
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
        self._root: Element = Element('robot', name='cell')

        self._config: list[StationConfigurations] = config
        self._get_robot: Callable[[str], Robot] = get_robot
        self._get_eoat: Callable[[str], EOAT] = get_eoat
        self._output_path: PosixPath = output_path

        self._active_robot: Robot | None = None
        self._active_eoat: EOAT | None = None

    def _create_chain(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        group: Element = SubElement(self._root, 'group', name=f'{prefix}_arm')
        SubElement(
            group, 'chain', base_link=f'{prefix}_{robot.base}',
            tip_link=f'{prefix}_{robot.flange}')

    def _create_default_poses(self, prefix: str) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        for pose in robot.poses:
            group: Element = SubElement(
                self._root, 'group_state', name=pose.name, group=f'{prefix}_arm')

            joint_names = [joint.name for joint in robot.joints]
            for joint, value in zip(joint_names, pose.pose):
                SubElement(
                    group, 'joint', name=f'{prefix}_{joint}', value=str(value))

    def _disable_collision_pair(self, prefix: str, has_eoat: bool) -> None:
        robot: Robot = cast(Robot, self._active_robot)

        for pair in robot.collision_disabled_pair:
            SubElement(
                self._root, 'disable_collisions', link1=f'{prefix}_{pair.link_1}',
                link2=f'{prefix}_{pair.link_2}', reason=pair.reason)

        if not has_eoat:
            return

        eoat: EOAT = cast(EOAT, self._active_eoat)

        for link in robot.tool_adjacent:
            SubElement(
                self._root, 'disable_collisions', link1=f'{prefix}_{link}',
                link2=f'{prefix}_{eoat.base}', reason='Adjacent')

        for pair in eoat.collision_disabled_pair:
            SubElement(
                self._root, 'disable_collisions', link1=f'{prefix}_{pair.link_1}',
                link2=f'{prefix}_{pair.link_2}', reason=pair.reason)

    def _create_all_arm(self, all_robots: dict[str, Robot]) -> None:
        group: Element = SubElement(self._root, 'group', name='all_arm')

        for prefix in all_robots:
            SubElement(group, 'group', name=f'{prefix}_arm')

        pose_names: set[str] = {
            pose.name for robot in all_robots.values() for pose in robot.poses}

        for pose in pose_names:
            group_state: Element = SubElement(
                self._root, 'group_state', name=pose, group='all_arms')

            for prefix, robot in all_robots.items():
                joint_names: list[str] = [joint.name for joint in robot.joints]
                _current_pose: list[float] = next(
                    robot_pose.pose for robot_pose in robot.poses if robot_pose.name == pose)

                for joint, value in zip(joint_names, _current_pose):
                    SubElement(
                        group_state, 'joint', name=f'{prefix}_{joint}', value=str(value))

    def generate(self) -> None:  # sourcery skip: extract-method

        all_robots: dict[str, Robot] = {}

        for config in self._config:
            self._active_robot = self._get_robot(config.model)

            all_robots[config.prefix] = self._active_robot

            has_eoat = config.eoat is not None

            if has_eoat:
                self._active_eoat = self._get_eoat(config.eoat)  # type: ignore

            self._create_chain(config.prefix)
            self._create_default_poses(config.prefix)

            self._disable_collision_pair(config.prefix, has_eoat)

        self._active_robot = None

        self._create_all_arm(all_robots)

        file: PosixPath = self._output_path / 'config' / 'station.srdf'
        file.parent.mkdir(parents=True, exist_ok=True)

        tree: ElementTree = ElementTree(self._root)
        indent(tree, space='  ', level=0)
        tree.write(file, encoding='utf-8', xml_declaration=True)
