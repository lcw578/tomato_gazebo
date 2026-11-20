"""Run the fixed-station harvest world with contact-based harvest arbitration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from tomato_sim_config import harvest_launch_defaults, load_configuration, workspace


def generate_launch_description():
    share = Path(get_package_share_directory('tomato_sim_gazebo'))
    config = load_configuration()
    defaults = harvest_launch_defaults(config)
    phase2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/phase2.launch.py')),
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
            'action_timeout_scale': LaunchConfiguration('action_timeout_scale'),
            'world': LaunchConfiguration('world'),
            'slide_initial': LaunchConfiguration('slide_initial'),
            'lift_height': LaunchConfiguration('lift_height'),
            'active_camera_side': LaunchConfiguration('active_camera_side'),
            'remote_depth_units_per_metre': LaunchConfiguration('remote_depth_units_per_metre'),
            'request_gripper_length': LaunchConfiguration('request_gripper_length'),
            'arm_base_yaw_deg': LaunchConfiguration('arm_base_yaw_deg'),
            'positive_rfid_m': LaunchConfiguration('positive_rfid_m'),
            'negative_rfid_m': LaunchConfiguration('negative_rfid_m'),
        }.items())
    arbiter = Node(
        package='tomato_sim_adapters', executable='harvest_physics_arbiter',
        parameters=[{
            'use_sim_time': True,
            'scene_file': str(workspace() / 'validation/phase3/phase3_scene.json'),
            'basket_frame': 'basket_link',
            'evidence_file': LaunchConfiguration('evidence_file'),
        }], output='screen')
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('self_collide', default_value='true'),
        DeclareLaunchArgument('left_camera', default_value='false'),
        DeclareLaunchArgument('right_camera', default_value='true'),
        DeclareLaunchArgument('hand_camera', default_value='false'),
        DeclareLaunchArgument('camera_rate', default_value='5'),
        DeclareLaunchArgument('action_timeout_scale', default_value='10.0'),
        DeclareLaunchArgument('active_camera_side', default_value='right'),
        DeclareLaunchArgument('remote_depth_units_per_metre', default_value='4000'),
        DeclareLaunchArgument('request_gripper_length', default_value='0.0'),
        DeclareLaunchArgument(
            'evidence_file', default_value=str(workspace() / 'validation/phase3_runtime.json')),
        DeclareLaunchArgument(
            'positive_rfid_m', default_value='2.4'),
        DeclareLaunchArgument('negative_rfid_m', default_value='-1.85'),
        phase2, arbiter,
    ])
