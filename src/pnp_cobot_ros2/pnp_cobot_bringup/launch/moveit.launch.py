#!/usr/bin/env python3
"""
Gazebo simulation plus MoveIt 2 (move_group) and RViz.

move_group starts once the controllers are active, so it can execute trajectories right away.

Usage:
    ros2 launch pnp_cobot_bringup moveit.launch.py                 # MotionPlanning panel
    ros2 launch pnp_cobot_bringup moveit.launch.py rviz_view:=mtc  # MoveIt Task Constructor panel

:author: Souvik Roy <sroyy57@gmail.com>
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bringup_utils import CONTROLLERS, RVIZ_VIEWS, include_launch, wait_then  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world_file', default_value='pick_and_place_demo.world',
                              description='World file in pnp_cobot_gazebo/worlds'),
        DeclareLaunchArgument('rviz_view', default_value='moveit', choices=list(RVIZ_VIEWS),
                              description='RViz layout: moveit (MotionPlanning panel) or '
                                          'mtc (Motion Planning Tasks panel)'),
    ]

    # Arguments are read here, at startup: the stages started later by wait_then run after
    # this file's scope has ended, when its launch configurations no longer exist
    def launch_setup(context):
        rviz_package, rviz_file = RVIZ_VIEWS[LaunchConfiguration('rviz_view').perform(context)]

        sim = include_launch('pnp_cobot_bringup', 'sim.launch.py', {
            'world_file': LaunchConfiguration('world_file').perform(context),
            'use_rviz': 'false',  # move_group brings its own RViz
        })

        move_group = include_launch('pnp_cobot_moveit_config', 'move_group.launch.py', {
            'use_sim_time': 'true',
            'use_rviz': 'true',
            'rviz_config_file': rviz_file,
            'rviz_config_package': rviz_package,
        })

        return [sim, *wait_then('controllers_for_moveit', ['--controllers', *CONTROLLERS],
                                [move_group])]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
