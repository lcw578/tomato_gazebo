"""Implement the AGV service and patrol parameter with the rail controller."""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from xc_interfaces.msg import AGVStatus
from xc_interfaces.srv import ControlAGV


class SimAgv(Node):
    def __init__(self):
        super().__init__('agv_always_go_straght_node')
        self.running = self.declare_parameter('is_continue_run', False).value
        self.positive_end = self.declare_parameter('positive_rfid_m', 2.4).value
        self.negative_end = self.declare_parameter('negative_rfid_m', -2.4).value
        if not -2.5 < self.negative_end < 0 < self.positive_end < 2.5:
            raise ValueError('RFID positions must be inside the rail limits')
        self.position = None
        self.speed = 0.0
        self.command = 0.0
        self.direction = 1
        self.manual_command_active = False
        self.initial_stop_sent = False
        self.card = 0
        self.pending_card = 0
        self.zone_samples = 0
        self.lifted = False
        self.command_pub = self.create_publisher(
            Float64MultiArray, '/slide_velocity_controller/commands', 10)
        self.status_pub = self.create_publisher(AGVStatus, '/agv_status', 10)
        self.create_subscription(JointState, '/joint_states', self.update, qos_profile_sensor_data)
        self.create_service(ControlAGV, '/control_agv', self.control)
        self.create_timer(0.1, self.tick)
        self.add_on_set_parameters_callback(self.parameters)

    def parameters(self, parameters):
        for param in parameters:
            if param.name == 'is_continue_run':
                self.running = param.value
                if not self.running:
                    self.manual_command_active = False
                    self.publish_speed(0.0)
                else:
                    self.manual_command_active = False
        return SetParametersResult(successful=True)

    def publish_speed(self, speed):
        self.command = speed
        self.command_pub.publish(Float64MultiArray(data=[speed]))

    def update(self, message):
        if 'machine_slide' not in message.name:
            return
        index = message.name.index('machine_slide')
        self.position = message.position[index]
        self.speed = message.velocity[index]
        if self.card == 13 and self.position < self.positive_end - 0.04:
            self.card = 0
        if self.card == 23 and self.position > self.negative_end + 0.04:
            self.card = 0
        card = 0
        if self.position >= self.positive_end and self.speed > 0.005:
            card = 13
        elif self.position <= self.negative_end and self.speed < -0.005:
            card = 23
        self.zone_samples = self.zone_samples + 1 if self.pending_card == card else 1
        self.pending_card = card
        if card and self.zone_samples >= 3 and self.card != card:
            self.card = card
            self.direction = -1 if card == 13 else 1
            if not self.running:
                self.publish_speed(0.0)

    def status(self):
        speed = max(-32768, min(32767, round(self.speed * 1000)))
        return AGVStatus(white_card_id=self.card, left_wheel_speed=speed,
                         right_wheel_speed=speed, front_magnetic_sensor=True,
                         rear_magnetic_sensor=True, magnetic_sensor_lift_status=self.lifted,
                         battery_voltage=24.0)

    def control(self, request, response):
        self.lifted = request.lift_magnetic_sensor
        speed = request.linear_speed / 1000.0
        if (self.position is not None and self.command_pub.get_subscription_count()
                and request.angular_speed == 0 and abs(speed) <= 0.5):
            if (self.card == 13 and speed > 0) or (self.card == 23 and speed < 0):
                speed = 0.0
            self.manual_command_active = True
            self.publish_speed(speed)
            response.control_cmd_success = True
        response.status_cmd_success = self.position is not None
        response.agv_status = self.status()
        return response

    def tick(self):
        if self.position is not None:
            if self.running:
                self.publish_speed(self.direction * 0.1)
            elif self.manual_command_active:
                self.publish_speed(self.command)
            elif not self.initial_stop_sent and self.command_pub.get_subscription_count():
                self.publish_speed(0.0)
                self.initial_stop_sent = True
            self.status_pub.publish(self.status())


def main():
    rclpy.init()
    node = SimAgv()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
