#!/usr/bin/env python3
"""
Replay a saved point cloud (.pcd) in RViz next to the running simulation.

Save the perception server's intermediate clouds first by running the demo with
    ros2 launch pnp_cobot_bringup pick_and_place.launch.py save_debug_clouds:=true
then this shows one of them from /tmp. Starts Gazebo and the controllers, then the viewer
once the robot is up.

Usage:
    ros2 launch pnp_cobot_bringup point_cloud.launch.py
    ros2 launch pnp_cobot_bringup point_cloud.launch.py file_name:=/tmp/5_support_plane_debug_cloud.pcd

:author: Souvik Roy <sroyy57@gmail.com>
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bringup_utils import CONTROLLERS, include_launch, wait_then  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('file_name', default_value='/tmp/5_objects_cloud_debug_cloud.pcd',
                              description='Point cloud file to show'),
    ]

    # Arguments are read at startup, because the later stages run after this file's scope ends
    def launch_setup(context):
        # The viewer brings its own RViz
        sim = include_launch('pnp_cobot_bringup', 'sim.launch.py', {'use_rviz': 'false'})

        viewer = include_launch('pnp_cobot_mtc_pick_place_demo', 'point_cloud_viewer.launch.py', {
            'file_name': LaunchConfiguration('file_name').perform(context),
            'use_sim_time': 'true',
        })

        return [sim, *wait_then('controllers_for_viewer', ['--controllers', *CONTROLLERS],
                                [viewer])]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
