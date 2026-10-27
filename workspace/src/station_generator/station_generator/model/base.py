from enum import Enum
from typing import TypeVar

from pydantic import BaseModel, ConfigDict

from station_generator.errors import ConfigError


class ComponentType(Enum):
    ROBOT = 'robot'
    EOAT = 'eoat'
    STATION_MECH = 'station_mech'
    BASE_MECH = 'base_mech'


class RootModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ComponentModel(RootModel):
    type: ComponentType


T = TypeVar("T", bound=ComponentModel)
_COMPONENTS: dict[ComponentType, type[ComponentModel]] = {}


def register_component(cls: type[T]) -> type[T]:
    key = cls.model_fields["type"].default
    if not isinstance(key, ComponentType):
        raise TypeError(
            f"{cls.__name__}.type must default to a ComponentType")
    if key in _COMPONENTS:
        raise ValueError(f"duplicate component type: {key.value}")
    _COMPONENTS[key] = cls
    return cls


def get_component(ctype: ComponentType | str) -> type[ComponentModel]:
    try:
        return _COMPONENTS[ComponentType(ctype)]
    except (KeyError, ValueError) as e:
        raise ConfigError(
            f"No component registered for type {ctype!r}.") from e


def parse_component(data: dict) -> ComponentModel:
    return get_component(data["type"]).model_validate(data)
