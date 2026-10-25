from _thread import LockType
from enum import Enum, auto
from threading import Lock

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_Goal
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.service import Service
from std_srvs.srv import Empty
from std_srvs.srv._empty import Empty_Request, Empty_Response
from trajectory_msgs.msg import JointTrajectoryPoint


class Position(Enum):
    TOP = auto()
    BOTTOM = auto()


class PlateActionClient:
    __slots__ = (
        '_client',
        '_goal_handle',
        '_joints',
        '_lock',
        '_name',
        '_node',
        '_on_done',
        '_position',
        '_x_cord',
        '_y_cord',
        '_z_cord'

    )

    def __init__(self, node: Node, name: str) -> None:
        self._name: str = name
        self._node: Node = node
        self._joints: list[str] = [f'{name}_plate_x', f'{name}_plate_y']

        self._goal_handle = None
        self._on_done = None

        self._lock: LockType = Lock()

        self._x_cord: float = 0.0
        self._y_cord: float = 0.0
        self._z_cord: float = 0.0
        self._position: Position = Position.BOTTOM

        self._client: ActionClient = ActionClient(
            node, FollowJointTrajectory,
            f'/{name}_controller/follow_joint_trajectory')

    def __repr__(self) -> str:
        return f'Plate {self._name} at [{self._x_cord}, {self._y_cord}, {self._z_cord}]'

    def update(self, key: str, value: float) -> None:
        with self._lock:
            match key:
                case 'x':
                    self._x_cord = round(value, 1)

                case 'y':
                    self._y_cord = round(value, 1)

                case 'z':
                    self._z_cord = round(value, 1)
                    self._position = (
                        Position.TOP if round(value) > 0.5 else Position.BOTTOM
                    )
                case _:
                    raise ValueError(
                        f"Invalid coordinate key: '{key}'. Expected 'x', 'y', or 'z'."
                    )

    @property
    def x_cord(self) -> float:
        return self._x_cord

    @property
    def y_cord(self) -> float:
        return self._y_cord

    @property
    def z_cord(self) -> float:
        return self._z_cord

    def send_goal(self, position: list[float], duration: float, on_done) -> None:
        self._on_done = on_done

        if not self._client.server_is_ready():
            self._finish(False, 'action server not available')
            return

        goal: FollowJointTrajectory_Goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[float(p) for p in position],
            velocities=[0.0, 0.0],
            time_from_start=Duration(
                sec=int(duration), nanosec=int((duration % 1) * 1e9)))]

        self._client.send_goal_async(
            goal).add_done_callback(self._goal_response)

    def cancel(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # ---- callbacks ----
    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._finish(False, 'goal rejected')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future) -> None:
        code = future.result().result.error_code
        self._finish(
            code == FollowJointTrajectory.Result.SUCCESSFUL,
            f'error_code={code}')

    def _finish(self, ok: bool, why: str = '') -> None:
        self._goal_handle = None
        if not ok:
            self._node.get_logger().error(f'{self._name}: {why}')
        cb, self._on_done = self._on_done, None
        if cb:
            cb(self._name, ok)


class PlateController:
    MAX_X: float = 4.0
    MIN_X: float = 0.0

    MAX_Y: float = 0.0
    MIN_Y: float = -2.0

    MAX_Z: float = 1.5
    MIN_Z: float = 0.0

    def __init__(self, node: Node) -> None:
        self._node: Node = node

        self._log: RcutilsLogger = self._node.get_logger()

        self._lock: LockType = Lock()
        self._plate_position: dict[str, PlateActionClient] = {}

        # self._bottom_plates: dict[str, PlateActionClient] = {}
        # self._top_plates: dict[str, PlateActionClient] = {}

        srv_top_cb_group = ReentrantCallbackGroup()
        srv_btm_cb_group = ReentrantCallbackGroup()

        self._pending: set[str] = set()
        self._ok = True
        self._on_done = None

        self._srv_top: Service = self._node.create_service(
            srv_type=Empty,
            srv_name='top',
            callback=self._cb_top,
            callback_group=srv_top_cb_group
        )

        self._srv_btm: Service = self._node.create_service(
            srv_type=Empty,
            srv_name='bottom',
            callback=self._cb_btm,
            callback_group=srv_btm_cb_group
        )

    def get_plate_position(self, key: str) -> PlateActionClient:
        if s_ac := self._plate_position.get(key):
            return s_ac

        s_ac = PlateActionClient(self._node, key)
        self._plate_position[key] = s_ac
        return s_ac

    def _plate_done(self, name: str, ok: bool) -> None:
        self._pending.discard(name)
        if not ok and self._ok:
            self._ok = False
            for n in self._pending:
                self._plate_position[n].cancel()

        if not self._pending and self._on_done:
            cb, self._on_done = self._on_done, None
            cb(self._ok)

    def _cb_btm(self, req: Empty_Request, resp: Empty_Response) -> Empty_Response:

        if not self._plate_position:
            return resp

        self._log.info('****************')
        for action in self._plate_position.values():
            if action._position == Position.TOP:
                continue

            target: list[float] = [0.0, 0.0]

            if action.x_cord == 0.0 and action.y_cord == 0.0:
                target[0] = 1.0

            elif action.x_cord > 0.0:
                target[0] = action.x_cord + 1.0

            elif action.y_cord < 0.0:
                target[1] = action.y_cord + 1.0

            action.send_goal(target, 3.0, self._plate_done)

            self._log.info(f'{action}: {target}')
        self._log.info('****************')

        return resp

    def _cb_top(self, req: Empty_Request, resp: Empty_Response) -> Empty_Response:

        if not self._plate_position:
            return resp


        self._log.info('****************')
        for action in self._plate_position.values():
            if action._position == Position.BOTTOM:
                continue

            target: list[float] = [0.0, 0.0]

            if action.x_cord == 0.0 and action.y_cord == 0.0:
                target[1] = -1.0

            elif action.x_cord > 0.0:
                target[0] = action.x_cord - 1.0

            elif action.y_cord < 0.0:
                target[1] = action.y_cord - 1.0

            action.send_goal(target, 3.0, self._plate_done)

            self._log.info(f'{action}: {target}')
        self._log.info('****************')

        return resp
