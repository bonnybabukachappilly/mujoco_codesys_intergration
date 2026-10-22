# main.py
from contextlib import suppress

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import MultiThreadedExecutor

from opcua_server.node import OPCUABridgeNode


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OPCUABridgeNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        with suppress(RCLError):
            rclpy.shutdown()
