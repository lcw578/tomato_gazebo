"""MuJoCo-compatible cut, rigid hold, release, and stem-contact arbitration."""
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import rclpy
from gazebo_msgs.msg import ContactsState, EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt8
from tf2_ros import Buffer, TransformException, TransformListener
from xc_interfaces.msg import MidpointReached, TestResult

CUT_SLOT_Y = (-0.0075, 0.0025)
CUT_SLOT_Z = (-0.094, -0.052)
CLOSED_BLADE_X = -0.004897326065632642
TCP_CUT_TOLERANCE = 0.005
MAX_NATURAL_CROSSING_DEG = 30.0


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def qrotate(q, v):
    r = qmul(qmul(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return r[:3]


def pose_compose(a, b):
    offset = qrotate(a[1], b[0])
    return (tuple(a[0][i] + offset[i] for i in range(3)), qmul(a[1], b[1]))


def pose_inverse(p):
    q = (-p[1][0], -p[1][1], -p[1][2], p[1][3])
    v = qrotate(q, tuple(-x for x in p[0]))
    return v, q


def pose_from_msg(msg):
    return ((msg.position.x, msg.position.y, msg.position.z),
            (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))


def pose_msg(value):
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = value[0]
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = value[1]
    return msg


class HarvestPhysicsArbiter(Node):
    def __init__(self):
        super().__init__('harvest_physics_arbiter')
        self.declare_parameter('scene_file', 'validation/phase3/phase3_scene.json')
        self.declare_parameter('plant_model', '')
        self.declare_parameter('basket_frame', 'basket_link')
        self.declare_parameter('evidence_file', 'validation/phase3_runtime.json')
        scene_path = Path(self.get_parameter('scene_file').value)
        self.scene = json.loads(scene_path.read_text())
        self.model = self.get_parameter('plant_model').value or self.scene['target_model']
        self.release_sdf = scene_path.parent / self.scene.get(
            'release_sdf', 'models/phase3_fruit_truss_9_1/model.sdf')
        self.basket_frame = self.get_parameter('basket_frame').value
        self.evidence_path = Path(self.get_parameter('evidence_file').value)
        self.evidence = {
            'state': 'ATTACHED_TO_STEM', 'cut_point_world_m': self.scene['cut_point_world'],
            'baseline_id': self.scene.get('baseline_id'),
            'source_world': self.scene.get('source_world'),
            'legacy': self.scene['legacy_parameters'], 'cut_attempts': [],
            'main_stem_contacts': 0, 'transport_contacts': [], 'camera': {},
        }
        self.cut_point = tuple(self.scene['cut_point_world'])
        self.midpoint_reached = False
        self.approaching = False
        self.input_x1 = False
        self.feedback_sent = False
        self.main_stem_contact_active = False
        self.stem_contacts_by_topic = {}
        self.blade_peduncle_contact = False
        self.payload_contacts_active = set()
        self.held = False
        self.pending_model_request = False
        self.tcp_at_grasp = None
        self.fruit_at_grasp = None
        self.latest_tcp = None
        self.last_feedback = 0
        self.hold_error = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sensor_pub = self.create_publisher(Bool, '/sim/harvest/input_x1', 10)
        self.x1_pub = self.create_publisher(UInt8, '/gripper/input_x1_status', 10)
        self.feedback_pub = self.create_publisher(TestResult, '/test_result', 10)
        self.create_subscription(MidpointReached, '/midpoint_reached', self.on_midpoint, 10)
        self.create_subscription(Bool, '/sim/harvest/approaching', self.on_approach, 10)
        self.create_subscription(Bool, '/sim/harvest/gripper_closed', self.on_gripper, 10)
        for topic in self.scene.get('main_stem_contact_topics', ['/sim/harvest/main_stem_contacts']):
            self.create_subscription(ContactsState, topic,
                                     lambda msg, source=topic: self.on_stem_contacts(msg, source),
                                     qos_profile_sensor_data)
        self.create_subscription(ContactsState, '/sim/harvest/blade_contacts',
                                 self.on_blade_contacts, qos_profile_sensor_data)
        self.create_subscription(ContactsState, '/sim/harvest/payload_contacts',
                                 self.on_payload_contacts, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/color/image_raw',
                                 lambda msg: self.note_image('rgb', msg), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw',
                                 lambda msg: self.note_image('depth', msg), qos_profile_sensor_data)
        self.set_state = self.create_client(SetEntityState, '/set_entity_state')
        self.get_state = self.create_client(GetEntityState, '/get_entity_state')
        self.delete_model = self.create_client(DeleteEntity, '/delete_entity')
        self.spawn_model = self.create_client(SpawnEntity, '/spawn_entity')
        self.timer = self.create_timer(0.02, self.tick)
        self.flush()

    def flush(self):
        self.evidence_path.parent.mkdir(parents=True, exist_ok=True)
        self.evidence_path.write_text(json.dumps(self.evidence, indent=2) + '\n')

    def note_image(self, key, message):
        self.evidence['camera'][key] = {
            'width': message.width, 'height': message.height,
            'encoding': message.encoding,
            'stamp_ns': message.header.stamp.sec * 10**9 + message.header.stamp.nanosec,
        }

    def on_midpoint(self, message):
        self.midpoint_reached = bool(message.reached)

    def on_approach(self, message):
        self.approaching = bool(message.data)
        if not self.approaching:
            self.midpoint_reached = False
            self.input_x1 = False
            self.feedback_sent = False
            self.blade_peduncle_contact = False
            self.last_feedback = 0
            self.sensor_pub.publish(Bool(data=False))
            self.x1_pub.publish(UInt8(data=0))

    @staticmethod
    def transform_point(transform, point):
        q = transform.transform.rotation
        translation = transform.transform.translation
        inverse = (-q.x, -q.y, -q.z, q.w)
        return qrotate(inverse, (point[0] - translation.x,
                                 point[1] - translation.y,
                                 point[2] - translation.z))

    def valid_blade_contact(self, state):
        names = (state.collision1_name, state.collision2_name)
        blade_name = next((name for name in names if 'mujoco_contact_blade' in name), None)
        stalk_name = next((name for name in names if 'contact_stalk_' in name), None)
        if blade_name is None or stalk_name is None or not state.contact_positions:
            return None
        if not self.approaching or not self.midpoint_reached or self.latest_tcp is None:
            return None
        tcp_error = math.dist(self.latest_tcp[0], self.cut_point)
        if tcp_error > TCP_CUT_TOLERANCE:
            return None
        try:
            base_tf = self.buffer.lookup_transform('cutter_base_link', 'world', rclpy.time.Time())
        except TransformException:
            return None
        tangent = self.transform_point(base_tf, self.scene['peduncle_tangent_world'])
        tangent_norm = math.sqrt(sum(component * component for component in tangent))
        if tangent_norm < 1e-9:
            return None
        crossing_angle = math.degrees(math.acos(min(1.0, abs(tangent[0]) / tangent_norm)))
        if crossing_angle > MAX_NATURAL_CROSSING_DEG:
            return None
        for position in state.contact_positions:
            local = self.transform_point(base_tf, (position.x, position.y, position.z))
            in_slot = (CUT_SLOT_Y[0] < local[1] < CUT_SLOT_Y[1]
                       and CUT_SLOT_Z[0] < local[2] < CUT_SLOT_Z[1])
            on_blade_plane = abs(local[0] - CLOSED_BLADE_X) <= 0.0005
            if in_slot and on_blade_plane:
                return {'collision': stalk_name, 'contact_tool_m': local,
                        'tcp_cutpoint_error_m': tcp_error,
                        'crossing_angle_deg': crossing_angle}
        return None

    def on_stem_contacts(self, message, source):
        names = [(state.collision1_name, state.collision2_name) for state in message.states]
        names = [pair for pair in names if any('tomato_picker::' in n for n in pair)]
        # Each v4 plant has its own topic. An empty observation from
        # another plant must not clear an ongoing main-stem contact.
        was_active = self.main_stem_contact_active
        self.stem_contacts_by_topic[source] = bool(names)
        self.main_stem_contact_active = any(self.stem_contacts_by_topic.values())
        if self.main_stem_contact_active and not was_active:
            self.evidence['main_stem_contacts'] += 1
            self.evidence.setdefault('stem_contact_pairs', []).extend([list(x) for x in names])
            self.evidence['cut_attempts'].append({'event': 'MAIN_STEM_CONTACT', 'pairs': names})
            self.flush()

    def on_blade_contacts(self, message):
        for state in message.states:
            accepted = self.valid_blade_contact(state)
            if accepted:
                self.blade_peduncle_contact = True
                self.evidence['latest_cut_contact'] = accepted

    def on_payload_contacts(self, message):
        active = set()
        state_name = self.evidence['state']
        if state_name not in ('HELD_BY_GRIPPER', 'RELEASED', 'IN_BASKET', 'DROPPED'):
            self.payload_contacts_active.clear()
            return
        for state in message.states:
            pair = (state.collision1_name, state.collision2_name)
            if not any('fruit_truss_9_1' in name for name in pair):
                continue
            key = tuple(sorted(pair))
            active.add(key)
            if key in self.payload_contacts_active:
                continue
            basket_contact = any('basket_link' in name for name in pair)
            self.evidence.setdefault('transport_contacts', []).append({
                'pair': list(pair), 'phase': state_name,
                'classification': 'basket_contact' if basket_contact else 'transport_contact',
                'sim_time_s': self.get_clock().now().nanoseconds / 1e9,
            })
            self.flush()
        self.payload_contacts_active = active

    def on_gripper(self, message):
        if not message.data:
            self.release_payload()
            return
        if not self.input_x1 or self.main_stem_contact_active or not self.blade_peduncle_contact:
            self.evidence['cut_attempts'].append({
                'event': 'CUT_MISS', 'reason': 'no_contact_in_closed_throat',
                'peduncle_contact': self.blade_peduncle_contact,
            })
            self.evidence['state'] = 'ATTACHED_TO_STEM'
            self.flush()
            return
        if self.latest_tcp is None:
            self.evidence['cut_attempts'].append({'event': 'CUT_MISS', 'reason': 'tcp_unavailable'})
            self.flush()
            return
        distance = math.dist(self.latest_tcp[0], self.cut_point)
        if distance > TCP_CUT_TOLERANCE:
            self.evidence['cut_attempts'].append({
                'event': 'CUT_MISS', 'reason': 'tcp_cutpoint_error',
                'tcp_error_m': distance,
            })
            self.flush()
            return
        self.call_model_state(self.get_state, self.capture_grasp)

    def call_model_state(self, client, callback):
        if not client.wait_for_service(timeout_sec=0.2):
            self.hold_error = f'{client.srv_name} unavailable'
            return
        request = GetEntityState.Request()
        request.name = self.model
        request.reference_frame = 'world'
        future = client.call_async(request)
        future.add_done_callback(callback)

    def capture_grasp(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.status_message)
            self.fruit_at_grasp = pose_from_msg(response.state.pose)
            self.tcp_at_grasp = self.latest_tcp
            self.held = True
            self.evidence['state'] = 'HELD_BY_GRIPPER'
            self.evidence['cut_attempts'].append({
                'event': 'CUT_SUCCESS', 'tcp_cutpoint_error_m': math.dist(self.latest_tcp[0], self.cut_point),
                'blade_peduncle_contact': self.blade_peduncle_contact,
            })
            self.flush()
        except Exception as error:
            self.hold_error = f'grasp attach failed: {error}'
            self.flush()

    def tick(self):
        try:
            transform = self.buffer.lookup_transform('world', 'tool_tcp', rclpy.time.Time())
        except TransformException:
            return
        msg = transform.transform
        self.latest_tcp = ((msg.translation.x, msg.translation.y, msg.translation.z),
                           (msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w))
        if self.approaching and self.midpoint_reached:
            distance = math.dist(self.latest_tcp[0], self.cut_point)
            stem_contact = self.main_stem_contact_active
            if stem_contact:
                result = 3
                self.input_x1 = False
            elif not self.feedback_sent and distance <= TCP_CUT_TOLERANCE:
                result = 2
                self.input_x1 = True
            else:
                result = 0
            self.sensor_pub.publish(Bool(data=self.input_x1))
            self.x1_pub.publish(UInt8(data=int(self.input_x1)))
            if result and (not self.feedback_sent or result != self.last_feedback):
                self.feedback_pub.publish(TestResult(result=result))
                self.feedback_sent = True
                self.last_feedback = result
                self.evidence['cut_attempts'].append({
                    'event': 'FEEDBACK', 'test_result': result,
                    'tcp_error_m': distance, 'main_stem_contact': stem_contact,
                })
                self.flush()
        if self.held and not self.pending_model_request:
            self.follow_grasp()

    def follow_grasp(self):
        if not self.set_state.service_is_ready() or self.latest_tcp is None:
            return
        relative = pose_compose(pose_inverse(self.tcp_at_grasp), self.fruit_at_grasp)
        target = pose_compose(self.latest_tcp, relative)
        state = EntityState()
        state.name = self.model
        state.pose = pose_msg(target)
        state.twist = Twist()
        state.reference_frame = 'world'
        if self.pending_model_request:
            return
        self.pending_model_request = True
        future = self.set_state.call_async(SetEntityState.Request(state=state))
        future.add_done_callback(self.follow_done)

    def follow_done(self, future):
        self.pending_model_request = False
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(getattr(result, 'status_message', 'SetEntityState failed'))
        except Exception as error:
            self.hold_error = f'fruit follow failed: {error}'
        else:
            self.evidence['state'] = 'HELD_BY_GRIPPER'

    def release_payload(self):
        if not self.held or self.pending_model_request or self.latest_tcp is None:
            return
        if (not self.get_state.service_is_ready() or not self.delete_model.service_is_ready()
                or not self.spawn_model.service_is_ready()):
            self.hold_error = 'Gazebo spawn/delete services unavailable for dynamic release'
            self.flush()
            return
        self.pending_model_request = True
        get = self.get_state.call_async(GetEntityState.Request(
            name=self.model, reference_frame='world'))
        get.add_done_callback(self.release_pose_ready)

    def release_pose_ready(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(getattr(response, 'status_message', 'GetEntityState failed'))
            drop_pose = pose_from_msg(response.state.pose)
            delete = self.delete_model.call_async(DeleteEntity.Request(name=self.model))
            delete.add_done_callback(lambda result: self.spawn_dropped(result, drop_pose))
        except Exception as error:
            self.pending_model_request = False
            self.hold_error = f'fruit release pose failed: {error}'

    def spawn_dropped(self, future, drop_pose):
        try:
            deleted = future.result()
            if not deleted.success:
                raise RuntimeError(getattr(deleted, 'status_message', 'DeleteEntity failed'))
            root = ET.parse(self.release_sdf).getroot()
            root.find('model').set('name', self.model)
            link = root.find('.//link')
            link.find('kinematic').text = 'false'
            link.find('gravity').text = 'true'
            sdf = ET.tostring(root, encoding='unicode')
            request = SpawnEntity.Request()
            request.name = self.model
            request.xml = sdf
            request.robot_namespace = ''
            request.initial_pose = pose_msg(drop_pose)
            request.reference_frame = 'world'
            spawned = self.spawn_model.call_async(request)
            spawned.add_done_callback(self.release_done)
        except Exception as error:
            self.pending_model_request = False
            self.hold_error = f'fruit dynamic respawn failed: {error}'

    def release_done(self, future):
        self.pending_model_request = False
        try:
            result = future.result()
            if not result.success:
                raise RuntimeError(getattr(result, 'status_message', 'SpawnEntity failed'))
            self.held = False
            self.evidence['state'] = 'RELEASED'
            self.evidence['release'] = {'dynamics_enabled': True, 'sim_time_s': self.get_clock().now().nanoseconds / 1e9}
            self.flush()
        except Exception as error:
            self.hold_error = f'fruit spawn failed: {error}'
            self.flush()


def main():
    rclpy.init()
    node = HarvestPhysicsArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
