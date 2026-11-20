from setuptools import setup

name = 'tomato_sim_adapters'
setup(
    name=name,
    version='0.1.0',
    packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'sim_camera_mux_relay = tomato_sim_adapters.sim_camera_mux_relay:main',
        'sim_agv_control_node = tomato_sim_adapters.sim_agv_control_node:main',
        'sim_gripper_node = tomato_sim_adapters.sim_gripper_node:main',
        'sim_aubo_motion_service = tomato_sim_adapters.sim_aubo_motion_service:main',
        'harvest_physics_arbiter = tomato_sim_adapters.harvest_physics_arbiter:main',
        'phase4_yolo = tomato_sim_adapters.phase4_yolo:main',
        'phase4_main_control = tomato_sim_adapters.phase4_main_control:main',
    ]},
)
