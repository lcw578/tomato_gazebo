"""Start the verification scene and the ROS 2 hardware adapters."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from tomato_sim_config import assembly_launch_defaults, load_configuration


def generate_launch_description():
    share = Path(get_package_share_directory('tomato_sim_gazebo'))
    defaults = assembly_launch_defaults(load_configuration())
    use_time = {'use_sim_time': True}
    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/phase1.launch.py')),
                                     launch_arguments={
                                         'gui': LaunchConfiguration('gui'),
                                         'camera_angle_deg': LaunchConfiguration('camera_angle_deg'),
                                         'camera_hfov_deg': LaunchConfiguration('camera_hfov_deg'),
                                         'camera_tilt_deg': LaunchConfiguration('camera_tilt_deg'),
                                         'self_collide': LaunchConfiguration('self_collide'),
                                         'left_camera': LaunchConfiguration('left_camera'),
                                         'right_camera': LaunchConfiguration('right_camera'),
                                         'hand_camera': LaunchConfiguration('hand_camera'),
                                         'camera_rate': LaunchConfiguration('camera_rate'),
                                         'world_record': LaunchConfiguration('world_record'),
                                         'world': LaunchConfiguration('world'),
                                         'slide_initial': LaunchConfiguration('slide_initial'),
                                         'lift_height': LaunchConfiguration('lift_height'),
                                         'arm_base_yaw_deg': LaunchConfiguration('arm_base_yaw_deg'),
                                     }.items())
    support = [
        Node(package='tomato_sim_adapters', executable='sim_camera_mux_relay',
             parameters=[use_time, {
                 'active_side': LaunchConfiguration('active_camera_side'),
                 'remote_depth_units_per_metre': ParameterValue(
                     LaunchConfiguration('remote_depth_units_per_metre'), value_type=int),
             }],
             output='screen'),
        Node(package='tomato_sim_adapters', executable='sim_gripper_node',
             parameters=[use_time], output='screen'),
    ]
    def motion_service(context):
        parameters = [use_time, {
            'action_timeout_scale': 10.0,
            'request_gripper_length': 0.0,
            'pose_reference': 'tcp',
            'compensate_native_tool_offset': False,
        }]
        config = LaunchConfiguration('motion_config').perform(context)
        if config:
            parameters.append(config)
        overrides = {}
        for name, value_type in (
                ('action_timeout_scale', float), ('request_gripper_length', float),
                ('pose_reference', str), ('compensate_native_tool_offset', bool)):
            value = LaunchConfiguration(name).perform(context)
            if value:
                overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
        parameters.append(overrides)
        return [Node(package='tomato_sim_adapters', executable='sim_aubo_motion_service',
                     parameters=parameters, output='screen')]

    support.append(OpaqueFunction(function=motion_service))
    agv = Node(package='tomato_sim_adapters', executable='sim_agv_control_node',
               parameters=[use_time, {
                   'positive_rfid_m': ParameterValue(LaunchConfiguration('positive_rfid_m'), value_type=float),
                   'negative_rfid_m': ParameterValue(LaunchConfiguration('negative_rfid_m'), value_type=float),
               }], output='screen')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('self_collide', default_value='true'),
        DeclareLaunchArgument('left_camera', default_value='true'),
        DeclareLaunchArgument('right_camera', default_value='true'),
        DeclareLaunchArgument('hand_camera', default_value='true'),
        DeclareLaunchArgument('camera_rate', default_value='15'),
        DeclareLaunchArgument('action_timeout_scale', default_value=''),
        DeclareLaunchArgument('slide_initial', default_value='0.0'),
        DeclareLaunchArgument('active_camera_side', default_value='left'),
        DeclareLaunchArgument('remote_depth_units_per_metre', default_value='4000'),
        DeclareLaunchArgument('request_gripper_length', default_value=''),
        DeclareLaunchArgument('pose_reference', default_value=''),
        DeclareLaunchArgument('compensate_native_tool_offset', default_value=''),
        DeclareLaunchArgument('motion_config', default_value=''),
        DeclareLaunchArgument('world_record', default_value=''),
        DeclareLaunchArgument('positive_rfid_m', default_value='2.4'),
        DeclareLaunchArgument('negative_rfid_m', default_value='-1.85'),
        base, agv, *support,
    ])
