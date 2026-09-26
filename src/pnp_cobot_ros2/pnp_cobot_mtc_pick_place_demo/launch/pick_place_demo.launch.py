#!/usr/bin/env python3
"""
ROS 2 launch file for the MoveIt Task Constructor pick and place with perception node.

This launch file configures and starts a pick-and-place demo using the MoveIt Task Constructor (MTC)
framework with perception capabilities. It sets up the necessary configurations for
trajectory execution, motion planning, and robot control specifically for the PnP Cobot platform.

:author: Addison Sears-Collins
:modified by: Souvik Roy <sroyy57@gmail.com> (2026) — ported to ROS 2 Humble / Gazebo Fortress
:date: December 19, 2024
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Generate a launch description.

    Returns:
        LaunchDescription: A complete launch description for the MTC pick and place demo system
    """
    # Constants for paths to different files and folders
    package_name_moveit_config = 'pnp_cobot_moveit_config'
    package_name_mtc_pick_place_demo = 'pnp_cobot_mtc_pick_place_demo'

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the package share directory
    pkg_share_moveit_config_temp = FindPackageShare(package=package_name_moveit_config)
    pkg_share_mtc_pick_place_demo_temp = FindPackageShare(
        package=package_name_mtc_pick_place_demo)

    # Declare the launch arguments
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='pnp_cobot_280',
        description='Name of the robot to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Object reset arguments: put the target back at its start pose before each run, so the
    # demo can be launched again without restarting Gazebo
    declare_reset_object_cmd = DeclareLaunchArgument(
        name='reset_object',
        default_value='true',
        description='Move the target object back to its start pose in Gazebo before planning')

    declare_world_name_cmd = DeclareLaunchArgument(
        name='world_name',
        default_value='default',
        description='Name of the Gazebo world (the <world name> in the .world file)')

    declare_object_model_cmd = DeclareLaunchArgument(
        name='object_model',
        default_value='red_cylinder',
        description='Gazebo model name of the object to reset')

    declare_object_x_cmd = DeclareLaunchArgument(
        name='object_x', default_value='0.22', description='Object start x, meters')

    declare_object_y_cmd = DeclareLaunchArgument(
        name='object_y', default_value='0.12', description='Object start y, meters')

    declare_object_z_cmd = DeclareLaunchArgument(
        name='object_z', default_value='0.175', description='Object start z, meters')

    def configure_setup(context):
        """Configure MoveIt and create nodes with proper string conversions."""
        # Get the robot name as a string for use in MoveItConfigsBuilder
        robot_name_str = LaunchConfiguration('robot_name').perform(context)

        # Get package path
        pkg_share_moveit_config = pkg_share_moveit_config_temp.find(package_name_moveit_config)
        pkg_share_mtc_pick_place_demo = pkg_share_mtc_pick_place_demo_temp.find(
            package_name_mtc_pick_place_demo)

        # Construct file paths using robot name string
        config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)
        mtc_node_config_path = os.path.join(pkg_share_mtc_pick_place_demo, 'config')

        # Define all config file paths
        initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
        joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
        kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
        moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
        srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
        pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

        mtc_node_params_file_path = os.path.join(mtc_node_config_path, 'mtc_node_params.yaml')

        # Create MoveIt configuration
        moveit_config = (
            MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
            .trajectory_execution(file_path=moveit_controllers_file_path)
            .robot_description_semantic(file_path=srdf_model_path)
            .joint_limits(file_path=joint_limits_file_path)
            .robot_description_kinematics(file_path=kinematics_file_path)
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner"],  # STOMP is not available in MoveIt 2 Humble
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        # Create MTC demo node
        mtc_demo_node = Node(
            package="pnp_cobot_mtc_pick_place_demo",
            executable="mtc_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                # Read now: the node may start after this launch file's scope has ended
                {'use_sim_time': use_sim_time.perform(context) == 'true'},
                {'start_state': {'content': initial_positions_file_path}},
                mtc_node_params_file_path,
            ],
        )

        if LaunchConfiguration('reset_object').perform(context).lower() != 'true':
            return [mtc_demo_node]

        # Teleport the object back to its start pose (upright, at rest)
        pose_req = 'name: "{}", position: {{x: {}, y: {}, z: {}}}, orientation: {{w: 1.0}}'.format(
            *[LaunchConfiguration(arg).perform(context)
              for arg in ('object_model', 'object_x', 'object_y', 'object_z')])
        reset_object_cmd = ExecuteProcess(
            cmd=['ign', 'service',
                 '-s', f"/world/{LaunchConfiguration('world_name').perform(context)}/set_pose",
                 '--reqtype', 'ignition.msgs.Pose',
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '3000',
                 '--req', pose_req],
            output='screen')

        # Start the MTC node once the reset is done, after a short wait so the perception
        # server has received a point cloud showing the object back at its start pose
        start_mtc_after_reset_cmd = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=reset_object_cmd,
                on_exit=[TimerAction(period=2.0, actions=[mtc_demo_node])]))

        return [reset_object_cmd, start_mtc_after_reset_cmd]

    # Create the launch description
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_reset_object_cmd)
    ld.add_action(declare_world_name_cmd)
    ld.add_action(declare_object_model_cmd)
    ld.add_action(declare_object_x_cmd)
    ld.add_action(declare_object_y_cmd)
    ld.add_action(declare_object_z_cmd)

    # Add the setup and node creation
    ld.add_action(OpaqueFunction(function=configure_setup))

    return ld
