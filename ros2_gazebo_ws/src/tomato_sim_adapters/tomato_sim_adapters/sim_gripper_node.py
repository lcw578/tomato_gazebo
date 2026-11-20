"""Drive the simulated cutter and return actual position in OEM pulse units."""
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from xc_interfaces.srv import GripperControl

OPEN_RAD, CLOSED_RAD = -0.45, 0.4
OPEN_PULSES, CLOSED_PULSES = 600, 2702


class SimGripper(Node):
    def __init__(self):
        super().__init__('sim_gripper_node')
        self.samples = threading.Condition()
        self.position = None
        self.state_received = 0.0
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.command = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)
        self.completed = self.create_publisher(Bool, '/sim/harvest/gripper_closed', 10)
        self.create_subscription(JointState, '/joint_states', self.update, qos_profile_sensor_data)
        self.create_service(GripperControl, '/gripper/control', self.move, callback_group=self.service_group)

    def update(self, message):
        if 'gripper_joint' not in message.name:
            return
        with self.samples:
            self.position = message.position[message.name.index('gripper_joint')]
            self.state_received = time.monotonic()
            self.samples.notify_all()

    def move(self, request, response):
        command = request.command.lower()
        if command not in ('open', 'close'):
            response.message = 'Invalid command'
            return response
        target = OPEN_RAD if command == 'open' else CLOSED_RAD
        if not self.command.get_subscription_count():
            response.message = 'Gripper controller is unavailable'
            return response
        start = time.monotonic()
        deadline = start + 5.0
        self.command.publish(Float64MultiArray(data=[target]))
        with self.samples:
            while self.context.ok() and time.monotonic() < deadline:
                if (self.state_received > start and self.position is not None
                        and abs(self.position - target) < 0.002):
                    response.success = True
                    break
                self.samples.wait(0.05)
            if self.position is not None:
                fraction = (self.position - OPEN_RAD) / (CLOSED_RAD - OPEN_RAD)
                response.current_position = round(OPEN_PULSES + max(0, min(1, fraction))
                                                  * (CLOSED_PULSES - OPEN_PULSES))
        response.message = 'Reached target' if response.success else 'Timeout'
        if response.success:
            self.completed.publish(Bool(data=command == 'close'))
        return response


def main():
    rclpy.init()
    node = SimGripper()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
