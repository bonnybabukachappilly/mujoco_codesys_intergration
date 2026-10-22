from asyncio import run, sleep
from threading import Event, Thread

from rclpy.impl.rcutils_logger import RcutilsLogger

from opcua_server.config import ServerConfig
from opcua_server.modules.base import BridgeModule
from opcua_server.opcua_server import OPCUAServer


class OPCUARunner:
    __slots__ = (
        '_cfg',
        '_log',
        '_modules',
        '_stop',
        '_thread'
    )

    def __init__(self, cfg: ServerConfig, modules: list[BridgeModule], logger: RcutilsLogger) -> None:
        self._cfg: ServerConfig = cfg
        self._modules: list[BridgeModule] = modules
        self._log: RcutilsLogger = logger

        self._stop = Event()
        self._thread = Thread(
            target=lambda: run(self._main()),
            name='opcua',
            daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)

    async def _main(self) -> None:
        server = OPCUAServer(self._cfg)

        await server.init()

        for mod in self._modules:
            await mod.setup_opcua(server)

        await server.start()

        self._log.info(f'OPC UA Server listening on {self._cfg.endpoint}')

        period: float = 1.0 / self._cfg.publish_rate

        try:
            while not self._stop.is_set():
                for mod in self._modules:
                    try:
                        await mod.publish()

                    except Exception as e:  # noqa: BLE001
                        self._log.error(
                            f'{type(mod).__name__}. Publish failed: {e}')
                await sleep(period)
        finally:
            for mod in self._modules:
                await mod.shutdown()
            await server.stop()
