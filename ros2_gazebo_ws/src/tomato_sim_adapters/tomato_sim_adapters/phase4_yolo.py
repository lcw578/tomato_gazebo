"""Run the project's YOLO service with the simulation's Conda-compatible bridge."""
from .ros_vision_compat import install_cv_bridge


def main():
    install_cv_bridge()
    from xc_robot_main_control import yolov8_service
    yolov8_service.main()
