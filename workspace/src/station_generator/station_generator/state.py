from pathlib import Path

from station_generator.model.base import ComponentModel


class Packages:
    __slot__: str = (
        '_data'
    )

    def __init__(self) -> None:
        self._data: dict[str, Path] = {}

    def register(self, key: str, value: Path) -> None:
        if key in self._data:
            raise KeyError(f'Key {key} already exists. No duplicate allowed')
        self._data[key] = value

    def fetch(self, key: str) -> Path:
        return self._data[key]

    @property
    def packages(self) -> dict[str, Path]:
        return self._data.copy()


class Components:
    __slot__: str = (
        '_data'
    )

    def __init__(self) -> None:
        self._data: dict[str, ComponentModel] = {}

    def register(self, key: str, value: ComponentModel) -> None:
        if key in self._data:
            raise KeyError(f'Key {key} already exists. No duplicate allowed')
        self._data[key] = value

    def fetch(self, key: str) -> ComponentModel:
        return self._data[key]

    @property
    def components(self) -> dict[str, ComponentModel]:
        return self._data.copy()


ros_packages = Packages()
station_components = Components()
