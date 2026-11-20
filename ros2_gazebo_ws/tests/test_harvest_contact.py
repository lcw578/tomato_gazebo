"""Check contact geometry in the cutter frame."""
import math
from concurrent.futures import Future
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
import xml.etree.ElementTree as ET
from std_msgs.msg import Bool

from gazebo_msgs.msg import ContactState, ContactsState, EntityState
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
from geometry_msgs.msg import TransformStamped

from tomato_sim_adapters.harvest_physics_arbiter import HarvestPhysicsArbiter
from tomato_sim_adapters import harvest_physics_arbiter as harvest_module


class HarvestContactTest(unittest.TestCase):
    def setUp(self):
        self.transform = TransformStamped()
        self.transform.transform.translation.x = -1.0
        self.transform.transform.rotation.w = 1.0

    def test_world_point_uses_target_from_source_transform(self):
        actual = HarvestPhysicsArbiter.transform_point(self.transform, (1.0, 0.0, 0.0))
        for value, expected in zip(actual, (0.0, 0.0, 0.0)):
            self.assertAlmostEqual(value, expected)

    def test_tangent_rotation_ignores_translation_and_normalizes(self):
        self.transform.transform.rotation.z = math.sin(math.pi / 4)
        self.transform.transform.rotation.w = math.cos(math.pi / 4)
        actual = HarvestPhysicsArbiter.transform_direction(self.transform, (2.0, 0.0, 0.0))
        for value, expected in zip(actual, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(value, expected)

    def test_reversed_tangent_accepts_same_zero_force_blade_contact(self):
        arbiter = object.__new__(HarvestPhysicsArbiter)
        arbiter.approaching = True
        arbiter.midpoint_reached = True
        arbiter.latest_tcp = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        arbiter.cut_point = (1.0, 0.0, 0.0)
        arbiter.scene = {'peduncle_tangent_world': [-1.0, 0.0, 0.0]}
        arbiter.evidence = {}
        arbiter.buffer = SimpleNamespace(lookup_transform=lambda *args: self.transform)
        state = SimpleNamespace(
            collision1_name='tomato_picker::mujoco_contact_blade',
            collision2_name='fruit_truss_9_1::contact_stalk_test',
            contact_positions=[SimpleNamespace(x=1.0 - 0.004897326065632642,
                                               y=-0.0044, z=-0.085)])
        accepted = arbiter.valid_blade_contact(state)
        self.assertIsNotNone(accepted)
        self.assertAlmostEqual(accepted['crossing_angle_deg'], 0.0)

    def test_open_event_is_retained_during_pending_follow(self):
        arbiter = object.__new__(HarvestPhysicsArbiter)
        arbiter.held = True
        arbiter.pending_model_request = True
        arbiter.latest_tcp = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        arbiter.release_requested = False
        arbiter.pending_close_until_ns = None
        arbiter.on_gripper(Bool(data=False))
        self.assertTrue(arbiter.release_requested)

    def test_closed_event_waits_for_asynchronous_contact_sensor(self):
        arbiter = object.__new__(HarvestPhysicsArbiter)
        arbiter.held = False
        arbiter.pending_model_request = False
        arbiter.blade_peduncle_contact = False
        arbiter.pending_close_until_ns = None
        arbiter.evidence = {'cut_attempts': []}
        arbiter.input_x1 = True
        arbiter.main_stem_contact_active = False
        arbiter.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100))
        arbiter.flush = lambda: None
        arbiter.on_gripper(Bool(data=True))
        self.assertEqual(arbiter.evidence['cut_attempts'], [])
        self.assertIsNotNone(arbiter.pending_close_until_ns)


class ServiceClient:
    """Record requests while letting tests deliver Gazebo replies in order."""
    def __init__(self):
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future


class HarvestStateTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        arbiter = object.__new__(HarvestPhysicsArbiter)
        self.arbiter = arbiter
        arbiter.model = 'fruit_truss_9_1'
        arbiter.release_model = 'fruit_truss_9_1_released'
        arbiter.held = True
        arbiter.pending_model_request = False
        arbiter.release_requested = False
        arbiter.drop_pose = None
        arbiter.pending_close_until_ns = None
        arbiter.latest_tcp = ((1., 0., 0.), (0., 0., 0., 1.))
        arbiter.tcp_at_grasp = arbiter.latest_tcp
        arbiter.fruit_at_grasp = arbiter.latest_tcp
        arbiter.cut_point = (1., 0., 0.)
        arbiter.blade_peduncle_contact = True
        arbiter.approaching = False
        arbiter.hold_error = None
        arbiter.evidence = {'state': 'HELD_BY_GRIPPER', 'cut_attempts': []}
        arbiter.flush = lambda: None
        arbiter.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100))
        transform = TransformStamped()
        transform.transform.translation.x = 1.
        transform.transform.rotation.w = 1.
        arbiter.buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
        arbiter.get_state = ServiceClient()
        arbiter.set_state = ServiceClient()
        arbiter.delete_model = ServiceClient()
        arbiter.spawn_model = ServiceClient()
        arbiter.release_sdf = Path(self.directory.name) / 'fruit.sdf'
        arbiter.release_sdf.write_text('''<sdf><model name="fruit"><link name="fruit">
            <kinematic>true</kinematic><gravity>false</gravity>
            <collision name="fruit_body"><geometry><sphere><radius>.02</radius>
            </sphere></geometry><surface><contact><collide_without_contact>true
            </collide_without_contact></contact></surface></collision>
            </link></model></sdf>''')

    def test_open_during_capture_releases_after_grasp_reply(self):
        arbiter = self.arbiter
        arbiter.held = False
        arbiter.pending_model_request = True
        arbiter.on_gripper(Bool(data=False))
        self.assertTrue(arbiter.release_requested)
        future = Future()
        state = EntityState()
        state.pose.orientation.w = 1.
        future.set_result(GetEntityState.Response(success=True, state=state))
        arbiter.capture_grasp(future)
        arbiter.tick()
        self.assertEqual(len(arbiter.get_state.requests), 1)
        self.assertEqual(len(arbiter.set_state.requests), 0)

    def test_close_cancels_open_queued_behind_follow(self):
        arbiter = self.arbiter
        arbiter.pending_model_request = True
        arbiter.on_gripper(Bool(data=False))
        arbiter.on_gripper(Bool(data=True))
        self.assertFalse(arbiter.release_requested)
        arbiter.pending_model_request = False
        arbiter.tick()
        self.assertEqual(len(arbiter.get_state.requests), 0)
        self.assertEqual(len(arbiter.set_state.requests), 1)

    def test_spawn_failure_retries_saved_release_pose_after_delete(self):
        arbiter = self.arbiter
        arbiter.on_gripper(Bool(data=False))
        state = EntityState()
        state.pose.position.x = .75
        state.pose.orientation.w = 1.
        arbiter.get_state.futures[0].set_result(
            GetEntityState.Response(success=True, state=state))
        arbiter.delete_model.futures[0].set_result(DeleteEntity.Response(success=True))
        arbiter.spawn_model.futures[0].set_result(SpawnEntity.Response(success=False))
        arbiter.on_gripper(Bool(data=True))
        arbiter.tick()
        self.assertEqual(len(arbiter.get_state.requests), 1)
        self.assertEqual(len(arbiter.delete_model.requests), 1)
        self.assertEqual(len(arbiter.spawn_model.requests), 2)
        self.assertEqual(arbiter.spawn_model.requests[1].initial_pose.position.x, .75)
        self.assertEqual(len(arbiter.set_state.requests), 0)
        arbiter.spawn_model.futures[1].set_result(SpawnEntity.Response(success=True))
        self.assertEqual(arbiter.evidence['state'], 'RELEASED')
        self.assertIsNone(arbiter.drop_pose)
        self.assertIsNone(arbiter.hold_error)

    def test_dynamic_spawn_adds_missing_legacy_link_flags(self):
        arbiter = self.arbiter
        tree = ET.parse(arbiter.release_sdf)
        link = tree.getroot().find('model/link')
        link.remove(link.find('kinematic'))
        link.remove(link.find('gravity'))
        tree.write(arbiter.release_sdf)
        arbiter.drop_pose = arbiter.latest_tcp
        arbiter.spawn_dropped()
        self.assertEqual(len(arbiter.spawn_model.requests), 1)
        spawned_link = ET.fromstring(arbiter.spawn_model.requests[0].xml).find('model/link')
        self.assertEqual(spawned_link.findtext('kinematic'), 'false')
        self.assertEqual(spawned_link.findtext('gravity'), 'true')

    def test_dynamic_spawn_uses_name_outside_original_world_pose_graph(self):
        arbiter = self.arbiter
        arbiter.drop_pose = arbiter.latest_tcp
        arbiter.spawn_dropped()
        request = arbiter.spawn_model.requests[0]
        self.assertEqual(request.name, arbiter.release_model)
        self.assertEqual(ET.fromstring(request.xml).find('model').get('name'),
                         arbiter.release_model)
        arbiter.spawn_model.futures[0].set_result(SpawnEntity.Response(success=True))
        self.assertEqual(arbiter.evidence['release']['model_name'], arbiter.release_model)

    def test_basket_contact_survives_fixed_joint_collision_renaming(self):
        arbiter = self.arbiter
        arbiter.evidence['state'] = 'RELEASED'
        arbiter.reported_payload_contacts = set()
        message = ContactsState(states=[ContactState(
            collision1_name='fruit_truss_9_1_released::fruit::g_tr37_s0',
            collision2_name=('tomato_picker::robot_carriage::'
                             'robot_carriage_fixed_joint_lump__crate_col_0_collision_1'))])
        arbiter.on_payload_contacts(message)
        self.assertEqual(arbiter.evidence['transport_contacts'][0]['classification'],
                         'basket_contact')


class BasketContainmentTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.sdf = Path(self.directory.name) / 'fruit.sdf'
        self.check = getattr(harvest_module, 'fruit_body_containment', None)
        self.assertIsNotNone(self.check, 'Fruit collision geometry must determine basket containment')

    def write_sphere(self, collision_pose, link_pose='0 0 0 0 0 0'):
        self.sdf.write_text(f'''<sdf><model name="fruit"><link name="fruit">
            <pose>{link_pose}</pose><collision name="g_tr37_s0">
            <pose>{collision_pose}</pose><geometry><sphere><radius>.02</radius>
            </sphere></geometry></collision><collision name="contact_stalk_cap">
            <geometry><sphere><radius>10</radius></sphere></geometry></collision>
            </link></model></sdf>''')

    def test_link_collision_and_settled_rotation_transform_actual_sphere_bounds(self):
        self.write_sphere('.02 0 0 0 0 0', '0 .05 0 0 0 1.5707963267948966')
        pose = ((.01, .02, .08), (0., 0., math.sqrt(.5), math.sqrt(.5)))
        result = self.check(self.sdf, pose)
        self.assertTrue(result['inside'])
        self.assertEqual(result['sphere_count'], 1)
        for actual, expected in zip(result['body_bounds_min_m'], (-.08, 0., .06)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(result['body_bounds_max_m'], (-.04, .04, .10)):
            self.assertAlmostEqual(actual, expected)

    def test_origin_inside_cannot_hide_rotated_fruit_beyond_wall_or_floor(self):
        self.write_sphere('0 0 -.265 0 0 0')
        for angle in (math.pi / 2, math.pi):
            with self.subTest(angle=angle):
                pose = ((0., 0., .08), (0., math.sin(angle / 2), 0., math.cos(angle / 2)))
                result = self.check(self.sdf, pose)
                self.assertFalse(result['inside'])
                self.assertGreater(max(result['outside_by_axis_m']), .1)

    def test_two_millimetre_tolerance_applies_to_sphere_surface(self):
        for position, expected in [
                ((.1315, 0., .08), True), ((.1325, 0., .08), False),
                ((0., .1865, .08), True), ((0., .1875, .08), False),
                ((0., 0., .135879), True), ((0., 0., .136879), False),
                ((0., 0., .043879), True), ((0., 0., .042879), False)]:
            with self.subTest(position=position):
                self.write_sphere('0 0 0 0 0 0')
                self.assertEqual(self.check(self.sdf, (position, (0., 0., 0., 1.)))['inside'],
                                 expected)

    def test_missing_fruit_body_cannot_pass_empty_geometry(self):
        self.sdf.write_text('''<sdf><model name="fruit"><link name="fruit">
            <collision name="contact_stalk_cap"><geometry><sphere><radius>.01</radius>
            </sphere></geometry></collision></link></model></sdf>''')
        with self.assertRaises(ValueError):
            self.check(self.sdf, ((0., 0., .08), (0., 0., 0., 1.)))
