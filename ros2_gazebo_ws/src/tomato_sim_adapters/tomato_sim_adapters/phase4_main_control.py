"""Run the real controller with Humble image bindings and a pinhole fallback."""

from .ros_vision_compat import install_realsense_pinhole


def main():
    try:
        import pyrealsense2  # noqa: F401
    except ModuleNotFoundError as error:
        if error.name != 'pyrealsense2':
            raise
        install_realsense_pinhole()
    from xc_robot_main_control import xc_robot_main_control_node as control

    control.main()
