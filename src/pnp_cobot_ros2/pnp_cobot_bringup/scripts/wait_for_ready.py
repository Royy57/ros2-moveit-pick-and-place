#!/usr/bin/env python3
"""
Block until parts of the ROS 2 system are ready, then exit.

Launch files start this as a process and use its exit (OnProcessExit) to start the next
stage, instead of sleeping for a fixed time. Exits 0 when everything is ready, 1 on timeout.

Examples:
    wait_for_ready.py --controllers joint_state_broadcaster arm_controller
    wait_for_ready.py --action /execute_task_solution --timeout 120

:author: Souvik Roy <sroyy57@gmail.com>
"""

import argparse
import sys
import time

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers


def controllers_active(node, client, names):
    """Return True when every controller in names is in the 'active' state."""
    if not client.wait_for_service(timeout_sec=1.0):
        return False
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if future.result() is None:
        return False
    states = {c.name: c.state for c in future.result().controller}
    return all(states.get(name) == 'active' for name in names)


def action_available(node, name):
    """Return True when an action server with this name exists in the ROS graph."""
    return any(action == name for action, _ in get_action_names_and_types(node))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--controllers', nargs='+', default=[],
                        help='controllers that must be active')
    parser.add_argument('--controller-manager', default='/controller_manager',
                        help='controller manager namespace')
    parser.add_argument('--action', action='append', default=[],
                        help='action server that must exist (repeatable)')
    parser.add_argument('--timeout', type=float, default=120.0,
                        help='seconds to wait before giving up')
    args, _ = parser.parse_known_args()  # ignore --ros-args added by launch

    rclpy.init()
    node = Node('wait_for_ready')
    client = node.create_client(ListControllers, f'{args.controller_manager}/list_controllers')
    waiting_for = ', '.join(args.controllers + args.action) or 'nothing'
    node.get_logger().info(f'Waiting for: {waiting_for}')

    deadline = time.monotonic() + args.timeout
    ready = False
    while rclpy.ok() and time.monotonic() < deadline:
        ready = ((not args.controllers or controllers_active(node, client, args.controllers))
                 and all(action_available(node, a) for a in args.action))
        if ready:
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    if ready:
        node.get_logger().info(f'Ready: {waiting_for}')
    else:
        node.get_logger().error(f'Timed out after {args.timeout:.0f} s waiting for: {waiting_for}')
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ready else 1)


if __name__ == '__main__':
    main()
