from dataclasses import dataclass
from pathlib import PosixPath
from typing import NamedTuple
from xml.etree.ElementTree import ElementTree


class PackageShare(NamedTuple):
    dobot: PosixPath
    eoat: PosixPath
    mujoco: PosixPath


@dataclass
class StationModel:
    name: str
    make: str
    type: str
    ros_pkg: str
    ros_descriptor: str
    mujoco_pkg: str
    mujoco_descriptor: str


@dataclass
class StationConfigurations:
    model: str
    prefix: str
    robot_xyz: str
    robot_rpy: str
    eoat: str | None
    eoat_xyz: str | None
    eoat_rpy: str | None


@dataclass
class CollisionDisabled:
    link_1: str
    link_2: str
    reason: str


@dataclass
class EOATJoint:
    name: str
    type: str
    mimic: str | None
    lower: float
    upper: float


@dataclass
class EOATPose:
    name: str
    pose: float


@dataclass
class EOAT:
    model: str

    xacro_file: ElementTree
    xacro_macro: str
    mujoco_file: ElementTree

    controller_type: str

    links: list[str]
    base: str
    joints: list[EOATJoint]
    poses: list[EOATPose]
    collision_disabled_pair: list[CollisionDisabled]


@dataclass
class RobotJoint:
    name: str
    max_velocity: float
    max_acceleration: float


@dataclass
class RobotPose:
    name: str
    pose: list[float]


@dataclass
class Robot:
    model: str

    xacro_file: ElementTree
    xacro_macro: str
    mujoco_file: ElementTree

    controller_type: str

    links: list[str]
    base: str
    flange: str
    joints: list[RobotJoint]
    poses: list[RobotPose]
    tool_adjacent: list[str]
    collision_disabled_pair: list[CollisionDisabled]
