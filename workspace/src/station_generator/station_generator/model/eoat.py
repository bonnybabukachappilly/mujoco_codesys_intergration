from enum import Enum
from typing import Literal

from pydantic import Field, field_validator

from .base import ComponentModel, ComponentType, RootModel, register_component
from .common import CollisionModel, NamedPoseModel
from .ros2_controllers import ROS2Controllers


class EoatKind(Enum):
    PARALLEL_GRIPPER = 'parallel_gripper'
    STATIC = 'static'


class GripperJointType(Enum):
    ACTUATED = 'actuated'
    MIMIC = 'mimic'


class EoatXacro(RootModel):
    file: str
    macro: str
    package: str


class EoatMujoco(RootModel):
    file: str
    package: str


class EoatJoint(RootModel):
    name: str
    type: GripperJointType
    mimic: str | None = Field(default=None)
    lower: float
    upper: float


@register_component
class EoatModel(ComponentModel):
    schema_version: Literal[1]

    name: str
    type: ComponentType = Field(default=ComponentType.EOAT)
    kind: EoatKind
    controller_type: ROS2Controllers | None = Field(default=None)

    xacro: EoatXacro
    mujoco: EoatMujoco

    links: list[str] | str
    base_link: str

    # For parallel gripper
    control_joint: list[str] | None = Field(default=None)
    joints: list[EoatJoint] | None = Field(default=None)
    named_poses: list[NamedPoseModel] | None = Field(default=None)
    collision: CollisionModel | None = Field(default=None)
