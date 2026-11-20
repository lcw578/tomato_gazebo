#!/usr/bin/env python3
"""Query the controller manager directly in the current ROS domain."""
import argparse

from controller_manager_msgs.srv import ListControllers
import rclpy


CONTROLLERS = {'joint_state_broadcaster', 'slide_velocity_controller',
               'arm_trajectory_controller', 'gripper_controller'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--wait', type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('tomato_verify_controllers')
    try:
        client = node.create_client(ListControllers, '/controller_manager/list_controllers')
        if not client.wait_for_service(timeout_sec=args.wait):
            return 1
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.wait)
        response = future.result()
        if response is None:
            return 1
        for controller in response.controller:
            print(controller.name, controller.type, controller.state)
        active = {controller.name for controller in response.controller if controller.state == 'active'}
        return 0 if CONTROLLERS <= active else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
