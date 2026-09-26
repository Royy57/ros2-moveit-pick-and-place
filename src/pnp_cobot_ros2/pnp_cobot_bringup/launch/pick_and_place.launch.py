#!/usr/bin/env python3
"""
Full perception-based pick and place demo.

Starts Gazebo, the controllers, move_group with the MTC RViz view, then, once move_group can
execute MTC solutions, the perception server and the MTC pick and place node.

Usage:
    ros2 launch pnp_cobot_bringup pick_and_place.launch.py

To run the task again while the simulation keeps running:
    ros2 launch pnp_cobot_mtc_pick_place_demo pick_place_demo.launch.py

:author: Souvik Roy <sroyy57@gmail.com>
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bringup_utils import MTC_EXECUTE_ACTION, include_launch, wait_then  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('reset_object', default_value='true',
                              description='Move the target object to its start pose before planning'),
        DeclareLaunchArgument('save_debug_clouds', default_value='false',
                              description='Save intermediate perception point clouds to /tmp '
                                          '(for point_cloud.launch.py)'),
        DeclareLaunchArgument('perception_log_level', default_value='info',
                              description='Log level of the perception server (debug shows every step)'),
    ]

    # Arguments are read at startup, because the later stages run after this file's scope ends
    def launch_setup(context):
        def arg(name):
            return LaunchConfiguration(name).perform(context)

        moveit = include_launch('pnp_cobot_bringup', 'moveit.launch.py', {
            'world_file': 'pick_and_place_demo.world',
            'rviz_view': 'mtc',
        })

        # The MTC node waits for the perception service itself, so both can start together
        perception = include_launch(
            'pnp_cobot_mtc_pick_place_demo', 'get_planning_scene_server.launch.py', {
                'use_sim_time': 'true',
                'save_debug_clouds': arg('save_debug_clouds'),
                'log_level': arg('perception_log_level'),
            })

        pick_place_task = include_launch(
            'pnp_cobot_mtc_pick_place_demo', 'pick_place_demo.launch.py', {
                'use_sim_time': 'true',
                'reset_object': arg('reset_object'),
            })

        return [moveit, *wait_then('move_group', ['--action', MTC_EXECUTE_ACTION],
                                   [perception, pick_place_task], timeout=180)]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
