from _thread import LockType
from dataclasses import dataclass
from threading import Lock
from typing import cast

from asyncua.common.node import Node as UaNode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from opcua_server.modules.base import BridgeModule, register
from opcua_server.opcua_server import OPCUAServer


@dataclass
class _Binding:
    joint: str
    field: str
    node: object
    last: float | None = None


@register('joint_states')
class JointStateModule(BridgeModule):
    __slots__: tuple[str, ...] = (
        '_binding',
        '_deadband',
        '_lock',
        '_pos',
        '_vel'
    )

    def __init__(self, cfg) -> None:
        super().__init__(cfg)

        self._lock: LockType = Lock()
        self._pos: dict[str, float] = {}
        self._vel: dict[str, float] = {}
        self._binding: list[_Binding] = []
        self._deadband: float = float(
            cfg.get('deadband', 1e-4))  # type: ignore

    def setup_ros(self, node: Node) -> None:
        super().setup_ros(node)

        node.create_subscription(
            msg_type=JointState,
            topic=self._cfg.get('jsp_topic', '/joint_states'),  # type: ignore
            callback=self._on_msg,
            qos_profile=qos_profile_sensor_data
        )

    def _on_msg(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._pos[name] = msg.position[i]
                if i < len(msg.velocity):
                    self._vel[name] = msg.velocity[i]

    async def setup_opcua(self, server: OPCUAServer) -> None:
        root: UaNode = cast(UaNode, server._root)

        if _lift_cfg := self._cfg.get('lifts'):
            lifts: UaNode = await server.add_folder(root, 'Lifts')
            for lift in _lift_cfg:
                obj: UaNode = await server.add_object(lifts, lift['name'])

                await self._bind(server, obj, 'Position', lift['joint'], 'position')
                await self._bind(server, obj, 'Velocity', lift['joint'], 'velocity')

        if _plate_cfg := self._cfg.get('plates'):
            plates: UaNode = await server.add_folder(root, 'plates')
            for n in range(1, int(_plate_cfg['count']) + 1):
                obj = await server.add_object(plates, f'P{n}')
                for axis in _plate_cfg.get('axes', ['x', 'y', 'z']):
                    joint = _plate_cfg['joint_pattern'].format(n=n, axis=axis)
                    await self._bind(server, obj, axis.upper(), joint, 'position')

    async def _bind(self, server: OPCUAServer, parent: UaNode, name: str, joint: str, field: str) -> None:
        var: UaNode = await server.add_variable(parent, name, 0.0)
        self._binding.append(_Binding(joint, field, var))

    async def publish(self) -> None:
        with self._lock:
            pos, vel = dict(self._pos), dict(self._vel)

        for binding in self._binding:
            value: float | None = (
                pos if binding.field == 'position' else vel).get(binding.joint)
            if value is None:
                continue

            if binding.last is None or abs(value - binding.last) > self._deadband:
                await binding.node.write_value(float(value))  # type: ignore
                binding.last = value
