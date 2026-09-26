"""
Shared helpers for the pnp_cobot_bringup launch files.

:author: Souvik Roy <sroyy57@gmail.com>
"""

from launch.actions import (
    EmitEvent, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Controllers loaded by pnp_cobot_moveit_config/launch/load_ros2_controllers.launch.py
CONTROLLERS = ['joint_state_broadcaster', 'arm_controller', 'gripper_action_controller']

# RViz views for move_group: rviz_view name -> (package, config file in its rviz/ folder)
RVIZ_VIEWS = {
    'moveit': ('pnp_cobot_moveit_config', 'move_group.rviz'),  # MotionPlanning panel
    'mtc': ('pnp_cobot_mtc_demos', 'mtc_demos.rviz'),          # Motion Planning Tasks panel
}

# Action server that move_group offers once the MTC ExecuteTaskSolution capability is loaded
MTC_EXECUTE_ACTION = '/execute_task_solution'


def include_launch(package, launch_file, launch_arguments=None):
    """
    Include package/launch/launch_file with its own scope.

    In ROS 2 Humble, arguments passed to an included launch file are otherwise global: for
    example use_rviz:=false given to one include would also turn off RViz in every other one.

    Args:
        package: package that contains the launch file
        launch_file: file name in the package's launch/ folder
        launch_arguments: dict of argument name -> value (strings or substitutions)

    Returns:
        GroupAction: the scoped include, to add to a LaunchDescription or an action list
    """
    include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=(launch_arguments or {}).items())
    return GroupAction([include], scoped=True)


def wait_then(name, wait_args, actions, timeout=120):
    """
    Start `actions` once wait_for_ready.py reports ready; shut down if it times out.

    Args:
        name: short label for the wait, used in the waiter's node name and log messages
        wait_args: arguments for wait_for_ready.py, e.g. ['--controllers', 'arm_controller']
        actions: launch actions to start when ready
        timeout: seconds to wait before shutting everything down

    Returns:
        list: the waiter process and its exit handler, to add to a LaunchDescription
    """
    waiter = Node(
        package='pnp_cobot_bringup',
        executable='wait_for_ready.py',
        name=f'wait_for_{name}',
        arguments=[*wait_args, '--timeout', str(timeout)],
        output='screen')

    def on_exit(event, context):
        if event.returncode == 0:
            return actions
        return [LogInfo(msg=f'Gave up waiting for {name}; shutting down.'),
                EmitEvent(event=Shutdown(reason=f'timed out waiting for {name}'))]

    return [waiter, RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=on_exit))]


def set_gazebo_view():
    """Point the Gazebo GUI camera at the robot and the table."""
    return ExecuteProcess(
        cmd=['ign', 'service', '-s', '/gui/move_to/pose',
             '--reqtype', 'ignition.msgs.GUICamera',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'pose: {position: {x: 1.36, y: -0.58, z: 0.95} '
                      'orientation: {x: -0.26, y: 0.1, z: 0.89, w: 0.35}}'],
        output='log')
