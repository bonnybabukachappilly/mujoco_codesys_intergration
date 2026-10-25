from asyncua import Server  # type: ignore
from asyncua.common.node import Node
from asyncua.common.subscription import Subscription
from asyncua.ua import Variant, VariantType

from opcua_server.config import ServerConfig


class OPCUAServer:
    __slots__: tuple[str, ...] = (
        '_cfg',
        '_idx',
        '_root',
        '_server'
    )

    def __init__(self, cfg: ServerConfig) -> None:
        self._cfg: ServerConfig = cfg
        self._server = Server()
        self._idx: int = 0
        self._root: Node | None = None

    async def init(self) -> None:
        await self._server.init()
        self._server.set_endpoint(self._cfg.endpoint)
        self._server.set_server_name(self._cfg.server_name)

        self._idx = await self._server.register_namespace(self._cfg.namespace_uri)
        self._root = await self._server.nodes.objects.add_object(self._idx, 'Station')

    async def start(self) -> None:
        await self._server.start()

    async def stop(self) -> None:
        await self._server.stop()

    async def add_folder(self, parent: Node, name: str) -> Node:
        return await parent.add_folder(self._idx, name)

    async def add_object(self, parent: Node, name: str) -> Node:
        return await parent.add_object(self._idx, name)

    async def add_variable(
            self, parent: Node, name: str, initial=0.0,
            vtype: VariantType = VariantType.Double, writable: bool = False) -> Node:
        var: Node = await parent.add_variable(self._idx, name, Variant(initial, vtype))

        if writable:
            await var.set_writable()

        return var

    async def subscribe(self, handler, period_ms: int = 100) -> Subscription:
        return await self._server.create_subscription(period_ms, handler)


# nmcli connection modify "BBK_S23" ipv4.dns "" ipv4.ignore-auto-dns no
# nmcli connection modify "BBK_S23" ipv6.dns "" ipv6.ignore-auto-dns no
# nmcli connection up "BBK_S23"