from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable

from rclpy.node import Node

from opcua_server.opcua_server import OPCUAServer

_REGISTRY: dict[str, type[BridgeModule]] = {}


def register(name: str) -> Callable[[type[BridgeModule]], type[BridgeModule]]:
    def decorator(cls: type[BridgeModule]) -> type[BridgeModule]:
        _REGISTRY[name] = cls
        return cls
    return decorator


def build_modules(cfg: dict[str, dict]) -> list[BridgeModule]:
    if (unknown := set(cfg) - set(_REGISTRY)):
        raise KeyError(f'Unknown modules in config: {sorted(unknown)}')
    return [_REGISTRY[name](cfg) for name, cfg in cfg.items()]


class BridgeModule(ABC):
    __slots__: tuple[str, ...] = (
        '_cfg',
        '_node'
    )

    def __init__(self, cfg: dict[str, dict]) -> None:
        self._cfg: dict[str, dict] = cfg
        self._node: Node | None = None

    def setup_ros(self, node: Node) -> None:
        self._node = node

    @abstractmethod
    async def setup_opcua(self, server: OPCUAServer) -> None:
        ...

    async def publish(self) -> None:
        ...

    async def shutdown(self) -> None:
        ...
