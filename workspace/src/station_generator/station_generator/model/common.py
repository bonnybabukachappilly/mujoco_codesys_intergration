from pydantic import Field

from .base import RootModel


class NamedPoseModel(RootModel):
    name: str
    joints: float | list[float]


class CollisionPairModel(RootModel):
    link1: str
    link2: str
    reason: str


class CollisionModel(RootModel):
    disabled_pairs: list[CollisionPairModel] = Field(default_factory=list)
