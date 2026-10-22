from rclpy.node import Node

from opcua_server.config import BridgeConfig, load_config
from opcua_server.modules import joint_states  # noqa: F401
from opcua_server.modules.base import BridgeModule, build_modules
from opcua_server.runner import OPCUARunner


class OPCUABridgeNode(Node):
    def __init__(self) -> None:
        super().__init__(node_name='opcua_station')

        self.declare_parameter('config_file', '')
        cfg: BridgeConfig = load_config(self.get_parameter(
            'config_file').value)  # type: ignore

        self._modules: list[BridgeModule] = build_modules(cfg.modules)
        for mod in self._modules:
            mod.setup_ros(self)

        self._runner = OPCUARunner(
            cfg.server, self._modules, self.get_logger())
        self._runner.start()

    def destroy_node(self):
        self._runner.stop()
        return super().destroy_node()
