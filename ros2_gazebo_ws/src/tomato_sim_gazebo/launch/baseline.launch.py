"""Launch the user's current v4 map and 180-degree arm installation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from tomato_sim_config import assembly_launch_defaults, load_configuration


def generate_launch_description():
    share = Path(get_package_share_directory('tomato_sim_gazebo'))
    baseline = load_configuration()
    values = {
        **assembly_launch_defaults(baseline),
        'slide_initial': str(baseline['slide_initial_m']),
        'world_record': '',
        'gui': 'false',
        'camera_rate': '3',
        'left_camera': 'false',
        'right_camera': 'true',
        'hand_camera': 'false',
        'active_camera_side': 'right',
        'remote_depth_units_per_metre': '1000',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in values.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/phase2.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in values}.items()),
    ])
