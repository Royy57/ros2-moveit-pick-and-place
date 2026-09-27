#!/usr/bin/env python3
"""
Gazebo simulation with the robot, the RGB-D camera and the ros2_control controllers.

Once the controllers are active, the Gazebo camera is pointed at the workspace.

Usage:
    ros2 launch pnp_cobot_bringup sim.launch.py
    ros2 launch pnp_cobot_bringup sim.launch.py use_rviz:=false world_file:=empty.world
    ros2 launch pnp_cobot_bringup sim.launch.py camera_mount:=wrist

:author: Souvik Roy <sroyy57@gmail.com>
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bringup_utils import CONTROLLERS, include_launch, set_gazebo_view, wait_then  # noqa: E402


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world_file', default_value='pick_and_place_demo.world',
                              description='World file in pnp_cobot_gazebo/worlds'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Show the robot model in RViz'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='Simulate the RGB-D camera'),
        DeclareLaunchArgument('camera_mount', default_value='stand', choices=['stand', 'wrist'],
                              description='Camera on a stand next to the robot, or on the wrist'),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot x position, meters'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot y position, meters'),
        DeclareLaunchArgument('z', default_value='0.0', description='Robot z position, meters'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Robot yaw, radians'),
    ]

    # Arguments are read at startup, like in the other bringup launch files
    def launch_setup(context):
        def arg(name):
            return LaunchConfiguration(name).perform(context)

        gazebo = include_launch('pnp_cobot_gazebo', 'pnp_cobot.gazebo.launch.py', {
            'load_controllers': 'true',
            'world_file': arg('world_file'),
            'use_camera': arg('use_camera'),
            'camera_mount': arg('camera_mount'),
            'use_rviz': arg('use_rviz'),
            'use_robot_state_pub': 'true',
            'use_sim_time': 'true',
            'x': arg('x'),
            'y': arg('y'),
            'z': arg('z'),
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': arg('yaw'),
        })

        return [gazebo, *wait_then('controllers', ['--controllers', *CONTROLLERS],
                                   [set_gazebo_view()])]

    return LaunchDescription([*args, OpaqueFunction(function=launch_setup)])
