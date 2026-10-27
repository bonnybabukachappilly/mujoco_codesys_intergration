from __future__ import annotations

from typing import Literal

from pydantic import Field, model_validator

from .base import ComponentModel, ComponentType, RootModel, register_component
from .common import CollisionModel, NamedPoseModel


class JointModel(RootModel):
    name: str
    max_velocity: float
    max_acceleration: float


class XacroModel(RootModel):
    file: str
    macro: str
    control: str
    control_macro: str


class MujocoModel(RootModel):
    package: str
    file: str


@register_component
class RobotModel(ComponentModel):
    schema_version: Literal[1]
    name: str
    type: ComponentType = Field(default=ComponentType.ROBOT)
    controller_type: str

    xacro: XacroModel
    mujoco: MujocoModel

    links: list[str]
    base_link: str
    flange_link: str

    joints: list[JointModel]
    named_poses: list[NamedPoseModel]

    tool_adjacent_links: list[str] = Field(default_factory=list)
    collision: CollisionModel = Field(default_factory=CollisionModel)

    @model_validator(mode='after')
    def _validate(self) -> RobotModel:
        links: set[str] = set(self.links)

        problems: list[str] = [
            f'link {ref} is not listed in links.'
            for ref in (
                self.base_link,
                self.flange_link,
                *self.tool_adjacent_links,
            )
            if ref not in links
        ]

        for pair in self.collision.disabled_pairs:
            for ref in (pair.link1, pair.link2):
                if ref not in links:
                    problems.append(f'collision pair uses unknown link: {ref}')

        joints: list[str] = [joint.name for joint in self.joints]
        if len(joints) != len(set(joints)):
            problems.append("duplicate joint names")

        for pose in self.named_poses:
            if not isinstance(pose.joints, list):
                problems.append('Expected list[float] not float.')
            elif len(pose.joints) != len(joints):
                problems.append(
                    f"pose '{pose.name}' has {len(pose.joints)} values, robot has {len(joints)} joints")

        if problems:
            raise ValueError("; ".join(problems))

        return self
