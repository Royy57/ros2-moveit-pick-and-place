#!/usr/bin/env python3
"""
Standalone MoveIt Task Constructor examples on the simulated robot.

Starts Gazebo, the controllers and move_group with the MTC RViz view, then runs one MTC
example once move_group can execute MTC solutions.

Usage:
    ros2 launch pnp_cobot_bringup mtc_demos.launch.py exe:=cartesian

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
        DeclareLaunchArgument(
            'exe', default_value='alternative_path_costs',
            choices=['alternative_path_costs', 'cartesian', 'fallbacks_move_to',
                     'ik_clearance_cost', 'modular'],
            description='MTC example to run'),
    ]

    # Arguments are read at startup, because the later stages run after this file's scope ends
    def launch_setup(context):
        moveit = include_launch('pnp_cobot_bringup', 'moveit.launch.py', {'rviz_view': 'mtc'})

        mtc_example = include_launch('pnp_cobot_mtc_demos', 'mtc_demos.launch.py', {
            'use_sim_time': 'true',
            'exe': LaunchConfiguration('exe').perform(context),
        })

        return [moveit, *wait_then('move_group', ['--action', MTC_EXECUTE_ACTION],
                                   [mtc_example], timeout=180)]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
