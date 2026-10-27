from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar, Protocol, TypeVar

from station_generator.model.station import StationModel


@dataclass(frozen=True, slots=True)
class Artifact:
    path: Path
    content: str
    producer: str


class Emitter(Protocol):
    name: ClassVar[str]

    # requires: ClassVar[tuple[str, ...]]
    # backend: ClassVar[str | None]

    def emit(self, station: StationModel) -> list[Artifact]: ...


E = TypeVar("E", bound=Emitter)

_EMITTERS: dict[str, type[Emitter]] = {}


def register_emitter(cls: type[E]) -> type[E]:
    if cls.name in _EMITTERS:
        raise ValueError(f"duplicate emitter name '{cls.name}'")
    _EMITTERS[cls.name] = cls
    return cls


def all_emitters() -> dict[str, type[Emitter]]:
    return dict(_EMITTERS)
