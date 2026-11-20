"""Launch the Classic verification world with the robot and its controllers."""
import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from tomato_sim_config import (assembly_launch_defaults, load_configuration, read_world,
                               sha256_file, workspace)


def generate_launch_description():
    config = load_configuration()
    defaults = assembly_launch_defaults(config)
    migration = Path(config['asset_root'])
    description = Path(get_package_share_directory('tomato_sim_description'))
    aubo = Path(get_package_share_directory('aubo_description'))
    model_path = [migration/'assets/models', migration/'assets',
                  description.parent, aubo.parent,
                  workspace() / 'validation/phase3/models',
                  Path('/usr/share/gazebo-11/models')]
    model_path = os.pathsep.join(str(p) for p in model_path)
    urdf = ParameterValue(Command([
        'xacro ', str(description/'urdf/tomato_picker_full.urdf.xacro'),
        ' lift_height:=', LaunchConfiguration('lift_height'),
        ' slide_initial:=', LaunchConfiguration('slide_initial'),
        ' arm_base_yaw_deg:=', LaunchConfiguration('arm_base_yaw_deg'),
        ' camera_angle_deg:=', LaunchConfiguration('camera_angle_deg'),
        ' camera_hfov_deg:=', LaunchConfiguration('camera_hfov_deg'),
        ' camera_tilt_deg:=', LaunchConfiguration('camera_tilt_deg'),
        ' self_collide:=', LaunchConfiguration('self_collide'),
        ' cameras:=', LaunchConfiguration('cameras'),
        ' left_camera:=', LaunchConfiguration('left_camera'),
        ' right_camera:=', LaunchConfiguration('right_camera'),
        ' hand_camera:=', LaunchConfiguration('hand_camera'),
        ' camera_rate:=', LaunchConfiguration('camera_rate'),
    ]), value_type=str)
    # gazebo_ros_state is a WorldPlugin, not a gzserver -s SystemPlugin.
    # Add it to a runtime copy while preserving the user's source map.
    def start_server(context):
        source = Path(LaunchConfiguration('world').perform(context)).resolve()
        tree, world = read_world(source)
        existing = world.findall("plugin[@filename='libgazebo_ros_state.so']")
        if not existing:
            plugin = ET.SubElement(world, 'plugin', {
                'name': 'gazebo_ros_state', 'filename': 'libgazebo_ros_state.so'})
            ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/'
            ET.SubElement(plugin, 'update_rate').text = '50'
        folder = workspace() / 'validation/runtime_worlds'
        folder.mkdir(parents=True, exist_ok=True)
        target = folder / f'{source.stem}_ros_{os.getpid()}.world'
        tree.write(target, encoding='utf-8', xml_declaration=True)
        report = {
            'source_world': str(source), 'source_sha256': sha256_file(source),
            'arm_base_yaw_deg': float(LaunchConfiguration('arm_base_yaw_deg').perform(context)),
            'runtime_world': str(target), 'world_name': world.get('name'),
            'change': 'gazebo_ros_state WorldPlugin only; map layout unchanged',
        }
        contents = json.dumps(report, indent=2) + '\n'
        target.with_suffix('.json').write_text(contents)
        record = LaunchConfiguration('world_record').perform(context)
        if record:
            path = Path(record)
            temporary_record = path.with_name(f'.{path.name}.{os.getpid()}')
            temporary_record.write_text(contents)
            temporary_record.replace(path)
        return [LogInfo(msg=f"Gazebo world: {world.get('name')} ({source}); "
                            f"arm installation yaw: {report['arm_base_yaw_deg']} deg"),
                ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                                    '-s', 'libgazebo_ros_factory.so', str(target)],
                               output='screen', on_exit=[Shutdown()])]
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'use_sim_time': True, 'robot_description': urdf}], output='screen')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'tomato_picker', '-topic', 'robot_description',
                            '-timeout', '35'])
    # Full v4 contact and camera initialization can exceed the default wait.
    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'slide_velocity_controller',
                                  'drive_wheel_velocity_controller',
                                  'arm_trajectory_controller', 'gripper_controller',
                                  '--controller-manager-timeout', '60', '--switch-timeout', '30'])

    def after_spawn(event, _context):
        if event.returncode:
            return [LogInfo(msg='Failed to spawn the tomato picker.'), Shutdown()]
        return [controllers]

    def after_spawners(event, _context):
        if event.returncode:
            return [LogInfo(msg='Failed to activate controllers.'), Shutdown()]
        return []

    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        DeclareLaunchArgument('world_record', default_value=''),
        DeclareLaunchArgument('slide_initial', default_value='0.0'),
        DeclareLaunchArgument('self_collide', default_value='true'),
        DeclareLaunchArgument('cameras', default_value='true'),
        DeclareLaunchArgument('left_camera', default_value='true'),
        DeclareLaunchArgument('right_camera', default_value='true'),
        DeclareLaunchArgument('hand_camera', default_value='true'),
        DeclareLaunchArgument('camera_rate', default_value='15'),
        DeclareLaunchArgument('gui', default_value='false'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(migration/'assets') + ':/usr/share/gazebo-11'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=controllers, on_exit=after_spawners)),
        OpaqueFunction(function=start_server),
        publisher,
        spawn,
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui'))),
    ])
