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


class LiftActionClient:
    __slots__ = (
        '_client',
        '_goal_handle',
        '_joints',
        '_left_lift',
        '_lock',
        '_node',
        '_on_done',
        '_position',
        '_right_lift',

    )

    def __init__(self, node: Node) -> None:
        self._node: Node = node
        self._joints: list[str] = ['left_lift_slide', 'right_lift_slide']

        self._goal_handle = None
        self._on_done = None

        self._lock: LockType = Lock()

        self._left_lift: float = 0.0
        self._right_lift: float = 0.0
        self._position: Position = Position.BOTTOM

        self._client: ActionClient = ActionClient(
            node, FollowJointTrajectory,
            '/lift_controller/follow_joint_trajectory')

    def __repr__(self) -> str:
        return f'Lift at [{self._left_lift}, {self._right_lift}]'

    def update(self, key: str, value: float) -> None:
        with self._lock:
            match key:
                case 'left':
                    self._left_lift = round(value, 3)

                case 'right':
                    self._right_lift = round(value, 3)
                case _:
                    raise ValueError(
                        f"Invalid coordinate key: '{key}'. Expected 'left' or 'right'."
                    )

    @property
    def left(self) -> float:
        return self._left_lift

    @property
    def right(self) -> float:
        return self._right_lift

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
            self._node.get_logger().error(f'Lift: {why}')
        cb, self._on_done = self._on_done, None
        if cb:
            cb('lift', ok)


class LiftController:
    Lift_MIN = 0.0
    Lift_MAX = 1.5

    def __init__(self, node: Node) -> None:
        self._node: Node = node
        self._log: RcutilsLogger = node.get_logger()

        self._lock: LockType = Lock()
        self._lift_position: LiftActionClient = LiftActionClient(node)

        srv_up_cb_group = ReentrantCallbackGroup()
        srv_down_cb_group = ReentrantCallbackGroup()

        self._srv_up: Service = self._node.create_service(
            srv_type=Empty,
            srv_name='up',
            callback=self._cb_up,
            callback_group=srv_up_cb_group
        )

        self._srv_down: Service = self._node.create_service(
            srv_type=Empty,
            srv_name='down',
            callback=self._cb_down,
            callback_group=srv_down_cb_group
        )

        self._pending: set[str] = set()
        self._ok = True
        self._on_done = None

    def get_lift_position(self) -> LiftActionClient:
        return self._lift_position

    def _plate_done(self, name: str, ok: bool) -> None:
        self._pending.discard(name)
        if not ok and self._ok:
            self._ok = False
            self._lift_position.cancel()

        if not self._pending and self._on_done:
            cb, self._on_done = self._on_done, None
            cb(self._ok)

    def _cb_up(self, req: Empty_Request, resp: Empty_Response) -> Empty_Response:
        self._log.info('****************')

        target: list[float] = [1.5, 1.5]

        self._lift_position.send_goal(target, 3.0, self._plate_done)

        self._log.info(f'{self._lift_position}: {target}')
        self._log.info('****************')

        return resp

    def _cb_down(self, req: Empty_Request, resp: Empty_Response) -> Empty_Response:
        self._log.info('****************')

        target: list[float] = [0, 0]

        self._lift_position.send_goal(target, 3.0, self._plate_done)

        self._log.info(f'{self._lift_position}: {target}')
        self._log.info('****************')

        return resp
