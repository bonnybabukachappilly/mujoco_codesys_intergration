from pathlib import Path
from typing import ClassVar, Protocol, TypeVar

from station_generator.errors import ConfigError


class BaseLoader(Protocol):
    name: ClassVar[str]

    def load(self, path: Path) -> object:
        ...


T = TypeVar("T", bound=BaseLoader)
_LOADERS: dict[str, type[BaseLoader]] = {}


def register_loaders(cls: type[T]) -> type[T]:
    if cls.name in _LOADERS:
        raise ValueError(f'duplicate loader name: {cls.name}')
    _LOADERS[cls.name] = cls
    return cls


def get_loaders(ext: str) -> BaseLoader:
    try:
        loader_cls = _LOADERS[ext]
    except KeyError as e:
        raise ConfigError(
            f'No loader registered for extension {ext!r}.') from e
    return loader_cls()
