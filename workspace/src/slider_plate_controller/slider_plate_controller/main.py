from contextlib import suppress
from re import Pattern, compile

from rclpy import init, shutdown
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from slider_plate_controller.lift import LiftActionClient, LiftController
from slider_plate_controller.plate import PlateActionClient, PlateController


class SliderPlateController(Node):
    def __init__(self) -> None:
        super().__init__(node_name='slider_plate_controller')

        subscriber_cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            msg_type=JointState,
            topic='/joint_states',
            callback=self._callback,
            qos_profile=qos_profile_sensor_data,
            callback_group=subscriber_cb_group
        )

        self._lift = LiftController(self)
        self._plate = PlateController(self)

        self.get_logger().info('Starting Slider Plate Controller')

    def _callback(self, msg: JointState) -> None:
        plate_pattern: Pattern[str] = compile(r'^p\d{1,2}_plate_[xyz]$')
        lift_pattern: Pattern[str] = compile(r'^(left|right)_lift_slide$')

        for i, name in enumerate(msg.name):
            if plate_pattern.match(name):
                _key: str = name.split('_')[0]
                _model_plate: PlateActionClient = self._plate.get_plate_position(
                    _key)
                _model_plate.update(name[-1], msg.position[i])

            if lift_pattern.match(name):
                _key = name.split('_')[0]
                _model_sld: LiftActionClient = self._lift.get_lift_position()
                _model_sld.update(_key, msg.position[i])


def main(args=None) -> None:
    init(args=args)

    controller = SliderPlateController()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        controller.destroy_node()
        with suppress(RCLError):
            shutdown()
