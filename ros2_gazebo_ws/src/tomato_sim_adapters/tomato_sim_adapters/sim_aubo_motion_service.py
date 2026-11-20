"""Use the AUBO hardware service contract to move the Gazebo arm."""
import math
import threading
import time

import numpy as np
import PyKDL as kdl
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt8
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformException, TransformListener
from xc_interfaces.msg import ArmStatus, MidpointReached
from xc_interfaces.srv import MoveJoints, MoveToPose

from .arm_kinematics import ArmKinematics, BOUND, JOINTS, OEM_HOME


def parse_box_waypoints(values):
    if values is None:
        return np.empty((0, 6))
    points = np.asarray(values, dtype=float)
    if points.ndim != 1 or points.size % 6:
        raise ValueError('box_waypoints_rad must contain six joint angles per waypoint')
    if not np.isfinite(points).all() or np.any(np.abs(points) > BOUND):
        raise ValueError('box_waypoints_rad must contain finite joint angles within AUBO limits')
    return points.reshape((-1, 6))


def waypoints(request, gripper_length=0.0):
    xyz = np.array([request.x, request.y, request.z])
    angles = np.array([request.rx, request.ry, request.rz, request.mid2target_angle])
    if not np.isfinite(xyz).all() or not np.isfinite(angles).all():
        raise ValueError('Pose must be finite')
    rx, ry, rz, angle = angles
    angle = math.remainder(angle, math.tau)
    sign = 1 if request.y > 0.0 else -1
    from_above = math.radians(70) < -sign * angle < math.radians(110)
    if from_above:
        offset = [0.0, -sign * 0.05, math.sqrt(3) * 0.05]
        rotation = kdl.Rotation.RPY(math.radians(150), 0.0, math.pi if sign > 0 else 0.0)
    else:
        angle = math.remainder(angle, math.pi)
        offset = [sign * 0.1 * math.sin(angle), -sign * 0.1 * math.cos(angle), 0.0]
        rotation = kdl.Rotation.RPY(rx, ry, rz + angle)
    # The hardware main controller offsets its SDK request away from the
    # observed cut point along the approach. Recover the cutter TCP target.
    target = xyz - np.asarray(offset) * (gripper_length / 0.1)
    return target + offset, target, rotation


class SimArm(Node):
    def __init__(self):
        super().__init__('sim_aubo_motion_service')
        self.kin = ArmKinematics()
        self.action_timeout_scale = float(
            self.declare_parameter('action_timeout_scale', 1.0).value)
        if self.action_timeout_scale < 1.0:
            raise ValueError('action_timeout_scale must be at least 1.0')
        box = self.declare_parameter('box_joints_rad', rclpy.Parameter.Type.DOUBLE_ARRAY).value
        self.box = np.asarray(box) if box is not None else None
        route = self.declare_parameter(
            'box_waypoints_rad', rclpy.Parameter.Type.DOUBLE_ARRAY).value
        self.box_waypoints = parse_box_waypoints(route)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.gripper_length = self.declare_parameter('request_gripper_length', 0.0).value
        if not math.isfinite(self.gripper_length) or self.gripper_length < 0.0:
            raise ValueError('request_gripper_length must be finite and nonnegative')
        self.state_lock = threading.Condition()
        self.q = None
        self.velocity = None
        self.state_time = 0.0
        self.approaching = False
        self.x1 = False
        self.midpoint_q = None
        self.approach_q = []
        self.callbacks = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.trajectory = ActionClient(
            self, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callbacks)
        self.arm_status = self.create_publisher(ArmStatus, '/arm_status', 10)
        self.midpoint = self.create_publisher(MidpointReached, '/midpoint_reached', 10)
        self.sensor = self.create_publisher(UInt8, '/gripper/input_x1_status', 10)
        self.approach = self.create_publisher(Bool, '/sim/harvest/approaching', 10)
        self.create_subscription(JointState, '/joint_states', self.update, qos_profile_sensor_data)
        self.create_subscription(Bool, '/sim/harvest/input_x1', self.update_sensor, 10)
        self.create_service(MoveJoints, '/move_joints', self.move_joints, callback_group=self.service_group)
        self.create_service(MoveToPose, '/move_to_pose', self.move_to_pose, callback_group=self.service_group)
        self.create_service(MoveToPose, '/target_return_midpoint', self.return_midpoint,
                            callback_group=self.service_group)
        self.create_service(MoveToPose, '/return_to_box', self.return_box,
                            callback_group=self.service_group)
        self.create_timer(0.05, self.publish_status)

    def update(self, message):
        if not set(JOINTS) <= set(message.name):
            return
        index = [message.name.index(name) for name in JOINTS]
        with self.state_lock:
            self.q = np.array([message.position[i] for i in index])
            self.velocity = np.array([message.velocity[i] for i in index])
            self.state_time = time.monotonic()
            self.state_lock.notify_all()

    def update_sensor(self, message):
        self.x1 = message.data

    def publish_status(self):
        self.sensor.publish(UInt8(data=int(self.approaching and self.x1)))
        with self.state_lock:
            q = self.q
        if q is None:
            return
        actual = self.kin.fk_tcp(q)
        status = ArmStatus(joints=np.rad2deg(q).tolist())
        status.position.x, status.position.y, status.position.z = actual.p
        (status.orientation.x, status.orientation.y,
         status.orientation.z, status.orientation.w) = actual.M.GetQuaternion()
        status.rx, status.ry, status.rz = actual.M.GetRPY()
        self.arm_status.publish(status)

    def current(self):
        with self.state_lock:
            if self.q is None or time.monotonic() - self.state_time > 1.0:
                raise RuntimeError('Fresh joint states are unavailable')
            return self.q.copy()

    def wait_action(self, future, timeout):
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout):
            raise RuntimeError('Controller action timed out')
        return future.result()

    def execute(self, points, max_speed=0.8):
        previous = self.current()
        if not self.trajectory.wait_for_server(timeout_sec=2.0):
            raise RuntimeError('Arm trajectory controller is unavailable')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        elapsed = 0.0
        for point in points:
            point = np.asarray(point)
            if point.shape != (6,) or not np.isfinite(point).all() or np.any(np.abs(point) > BOUND):
                raise ValueError('Target joint angle is outside the AUBO simulation limits')
            elapsed += max(float(np.max(np.abs(point - previous))) * 1.5 / max_speed, 0.25)
            nanoseconds = round(elapsed * 1e9)
            goal.trajectory.points.append(JointTrajectoryPoint(
                positions=point.tolist(), velocities=[0.0] * 6,
                time_from_start=Duration(sec=nanoseconds // 10**9, nanosec=nanoseconds % 10**9)))
            previous = point
        accepted = self.wait_action(self.trajectory.send_goal_async(goal),
                                    4.0 * self.action_timeout_scale)
        if not accepted.accepted:
            raise RuntimeError('Controller rejected the trajectory')
        try:
            result = self.wait_action(
                accepted.get_result_async(), (3 * elapsed + 10) * self.action_timeout_scale)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                raise RuntimeError(f'Arm action failed: {result.result.error_string}')
            settle_start = time.monotonic()
            settle_deadline = settle_start + 30.0
            stable_since = None
            with self.state_lock:
                while self.context.ok() and time.monotonic() < settle_deadline:
                    settled = (self.state_time > settle_start
                               and np.max(np.abs(self.q - previous)) < 0.003
                               and np.max(np.abs(self.velocity)) < 0.12)
                    if settled:
                        stable_since = stable_since or time.monotonic()
                        if time.monotonic() - stable_since >= 0.25:
                            return
                    else:
                        stable_since = None
                    self.state_lock.wait(0.05)
            raise RuntimeError(
                f'Measured joints did not settle: error={self.q - previous}, velocity={self.velocity}')
        except RuntimeError:
            accepted.cancel_goal_async()
            raise

    def move_joints(self, request, response):
        try:
            self.end_approach()
            self.execute([request.joints])
            self.midpoint_q = None
            self.approach_q = []
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(str(error))
            response.result = 30001
        return response

    def move_to_pose(self, request, response):
        if request.start2mid and request.mid_to_target:
            response.result = 30001
            return response
        try:
            midpoint, target, rotation = waypoints(request, self.gripper_length)
            q = self.current()
            if not request.mid_to_target:
                q = self.kin.ik_tcp(midpoint, rotation, q, alternatives=True)
            if not request.start2mid:
                start = midpoint
                if request.mid_to_target:
                    if self.midpoint_q is None:
                        raise ValueError('The arm must first move to a midpoint')
                    pose = self.kin.fk_tcp(q)
                    start = np.array(list(pose.p))
                span = np.linalg.norm(target - start)
                count = max(1, math.ceil(span / 0.02))
                points = []
                seed = q
                for index in range(1, count + 1):
                    xyz = start + (target - start) * index / count
                    next_q = self.kin.ik_tcp(xyz, rotation, seed)
                    if np.max(np.abs(next_q - seed)) > 0.65:
                        raise ValueError('Approach IK changed joint branches')
                    points.append(next_q)
                    seed = next_q
            if not request.mid_to_target:
                self.approaching = False
                self.x1 = False
                self.midpoint.publish(MidpointReached(reached=False))
                self.execute([q])
                self.midpoint_q = q
                self.approach_q = []
                self.midpoint.publish(MidpointReached(reached=True))
                deadline = time.monotonic() + 0.5
                with self.state_lock:
                    while self.context.ok() and time.monotonic() < deadline:
                        self.state_lock.wait(max(0.0, deadline - time.monotonic()))
                self.midpoint.publish(MidpointReached(reached=True))
            if not request.start2mid:
                self.approaching = True
                self.approach.publish(Bool(data=True))
                try:
                    self.execute(points, max_speed=0.32)
                    self.approach_q = points
                except RuntimeError:
                    self.end_approach()
                    raise
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(str(error))
            response.result = 30003
        return response

    def end_approach(self):
        self.approaching = False
        self.x1 = False
        self.midpoint.publish(MidpointReached(reached=False))
        self.approach.publish(Bool(data=False))

    def retreat(self):
        if self.midpoint_q is None:
            raise ValueError('No saved midpoint for retreat')
        if self.approach_q:
            self.execute(list(reversed(self.approach_q[:-1])) + [self.midpoint_q], max_speed=0.45)
        else:
            self.execute([self.midpoint_q])
        self.approach_q = []

    def return_midpoint(self, _request, response):
        self.end_approach()
        try:
            self.retreat()
        except (ValueError, RuntimeError) as error:
            self.get_logger().warning(str(error))
            response.result = 30006
        return response

    def return_box(self, _request, response):
        self.end_approach()
        try:
            if self.approach_q:
                self.retreat()
            points = self.box_waypoints
            if not len(points):
                target = self.box
                if target is None:
                    transform = self.tf.lookup_transform(
                        'arm_base_link', 'basket_link', rclpy.time.Time()).transform
                    p, q = transform.translation, transform.rotation
                    base_from_basket = kdl.Frame(
                        kdl.Rotation.Quaternion(q.x, q.y, q.z, q.w), kdl.Vector(p.x, p.y, p.z))
                    target = self.kin.ik_basket(base_from_basket, OEM_HOME)
                points = [OEM_HOME, target]
            self.execute(points)
            self.midpoint_q = None
        except (ValueError, RuntimeError, TransformException) as error:
            self.get_logger().warning(str(error))
            response.result = 30005
        return response


def main():
    rclpy.init()
    node = SimArm()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
