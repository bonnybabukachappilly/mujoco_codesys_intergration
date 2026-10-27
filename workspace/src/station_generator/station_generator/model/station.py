from typing import Any, Literal

from pydantic import Field, field_validator, model_validator

from .base import ComponentType, RootModel


class CoOrdinateModel(RootModel):
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


class StnComponentModel(RootModel):
    type: ComponentType
    make: str
    descriptor: str
    package: str


class StationRobotModel(RootModel):
    id: str = Field(pattern=r"^[a-z][a-z0-9]*$")
    model: str
    support: CoOrdinateModel
    eoat: str | None = None
    mount: CoOrdinateModel | None = None
    base: str | None = None


class StationModel(RootModel):
    schema_version: Literal[1]
    name: str
    components: dict[str, StnComponentModel]
    layout_origin: CoOrdinateModel
    static_support_height: float
    robots: list[StationRobotModel]

    @model_validator(mode='before')
    @classmethod
    def _resolve_references(cls, data: Any) -> Any:
        if not isinstance(data, dict):
            return data

        components = data.get('components')
        robots = data.get('robots')

        if not isinstance(components, dict) or not isinstance(robots, list):
            return data

        resolved = []

        for robot in robots:
            if not isinstance(robot, dict):
                resolved.append(robot)
                continue

            robot = dict(robot)
            rid = robot.get('id', '?')

            model = robot.get('model')
            if isinstance(model, str) and model not in components:
                raise ValueError(f'robot {rid}: unknown model {model!r}')

            resolved.append(robot)

        return {**data, 'robots': resolved}
