from __future__ import annotations

import logging
from pathlib import Path
from typing import ClassVar, cast

from lxml.etree import Element, SubElement, _Element

from station_generator.emit.base import Artifact, register_emitter
from station_generator.model.eoat import EoatKind, EoatModel
from station_generator.model.robot import RobotModel
from station_generator.model.station import StationModel
from station_generator.state import station_components
from station_generator.utils.naming import arm_group, joint, link

log: logging.Logger = logging.getLogger("generator")

OUTPUT = Path("config/station.srdf")


@register_emitter
class SrdfEmitter:
    name: ClassVar[str] = 'srdf'

    def emit(self, station: StationModel) -> list[Artifact]:
        root: _Element = Element('robot', name=station.name)

        self._all_arm_grp: _Element = SubElement(root, 'group', name='all_arm')
        self._all_arm: _Element | None = None

        for robot in station.robots:
            _id: str = robot.id
            _model: RobotModel = cast(
                RobotModel, station_components.fetch(robot.model))

            _gripper: EoatModel | None = None

            if _eoat := robot.eoat:
                _gripper = cast(
                    EoatModel, station_components.fetch(_eoat))

                if _gripper.kind != EoatKind.PARALLEL_GRIPPER:
                    _gripper = None

            SubElement(self._all_arm_grp, 'group', name=arm_group(_id))

            self._arm_group(root, _id, _model)
            self._arm_group_state(root, _id, _model)
            self._arm_collision(root, _id, _model, _gripper)

        return [Artifact(OUTPUT, root, self.name)]

    def _arm_group(self, root: _Element, id: str, robot: RobotModel) -> None:
        group: _Element = SubElement(root, 'group', name=arm_group(id))
        SubElement(group, 'chain', base_link=link(id, robot.base_link))

    def _arm_group_state(self, root: _Element, id: str, robot: RobotModel) -> None:
        for pose in robot.named_poses:
            group: _Element = SubElement(
                root, 'group_state', name=pose.name, group=arm_group(id))
            joints: list[str] = [joint.name for joint in robot.joints]

            if self._all_arm is None:
                self._all_arm = SubElement(
                    root, 'group_state', name=pose.name, group='all_arms')

            assert isinstance(pose.joints, list)
            for _joint, _value in zip(joints, pose.joints):
                SubElement(group, 'joint', name=joint(
                    id, _joint), value=str(_value))
                SubElement(self._all_arm, 'joint', name=joint(
                    id, _joint), value=str(_value))

            self._all_arm = None

    def _arm_collision(self, root: _Element,  id: str, robot: RobotModel, gripper: EoatModel | None) -> None:
        for collision in robot.collision.disabled_pairs:
            SubElement(
                root, 'disable_collisions',
                link1=link(id, collision.link1),
                link2=link(id, collision.link2),
                reason=collision.reason)

        if not gripper:
            return

        assert gripper.collision is not None
        for collision in gripper.collision.disabled_pairs:
            SubElement(
                root, 'disable_collisions',
                link1=link(id, collision.link1),
                link2=link(id, collision.link2),
                reason=collision.reason)
