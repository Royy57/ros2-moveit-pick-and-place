# pnp_cobot_ros2
ROS 2 Humble packages for the PnP Cobot perception-based pick and place simulation. For the
project overview, see the [main README](../../README.md).

## Packages
| Package | Contents |
| --- | --- |
| `pnp_cobot_description` | URDF/xacro, meshes, RGB-D camera, RViz config |
| `pnp_cobot_gazebo` | Gazebo worlds, object models, ROS–Gazebo bridge, simulation launch |
| `pnp_cobot_moveit_config` | SRDF, kinematics, joint limits, planner and controller configs, `move_group` launch |
| `pnp_cobot_mtc_pick_place_demo` | Perception server (`get_planning_scene_server`) and the MTC pick and place node |
| `pnp_cobot_mtc_demos` | Smaller standalone MTC examples |
| `pnp_cobot_moveit_demos` | MoveIt 2 motion planning examples |
| `pnp_cobot_interfaces` | Custom services and messages |
| `pnp_cobot_bringup` | Convenience launch scripts |
| `pnp_cobot_system_tests` | Arm and gripper test scripts |
| `pnp_cobot_ros2` | Metapackage |

## Setup
Requires Ubuntu 22.04, ROS 2 Humble and Gazebo Fortress. Install the dependencies:

```bash
sudo apt install \
  ros-humble-moveit ros-humble-moveit-configs-utils \
  ros-humble-moveit-planners-ompl ros-humble-pilz-industrial-motion-planner \
  ros-humble-moveit-task-constructor-core ros-humble-moveit-task-constructor-capabilities \
  ros-humble-moveit-task-constructor-msgs ros-humble-moveit-task-constructor-visualization \
  ros-humble-moveit-visual-tools ros-humble-rviz-visual-tools \
  ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control \
  ros-humble-ros-gz ros-humble-pcl-ros ros-humble-tf2-sensor-msgs ros-humble-sensor-msgs-py \
  ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-urdf-tutorial
```

Build:
```bash
cd ~/pick_n_place_cobot
colcon build --symlink-install
source install/setup.bash
```

## Run the simulation

**Full pick and place demo:**
```bash
bash src/pnp_cobot_ros2/pnp_cobot_mtc_pick_place_demo/scripts/robot.sh
```

**Simulation and controllers only**, with no MoveIt or perception:
```bash
bash src/pnp_cobot_ros2/pnp_cobot_bringup/scripts/pnp_cobot_280_gazebo.sh
```

On Ctrl+C, these scripts force-kill every process whose command line matches `ros2`, `gz`,
`rviz2`, `moveit` and similar. Close other ROS 2 or Gazebo work first.

## What to expect
`robot.sh` starts, in order: Gazebo, then `move_group` with RViz, then the perception server,
then the MTC node. The full run takes about a minute.

![Gazebo scene at start](../../docs/media/gazebo_scene.png)

1. **Gazebo (~20 s):** the robot spawns at the table, and the controllers `joint_state_broadcaster`,
   `arm_controller` and `gripper_action_controller` become active.
2. **Perception:** the log reports a support plane and 7 collision objects (the table, 4 boxes
   and 2 cylinders), and chooses `cylinder_1`, the red cylinder, as the target.
3. **Planning (~15 s):** the MTC node logs `Task planning succeeded`. The solution appears in RViz
   under **Motion Planning Tasks**.
4. **Execution (~10 s):** the arm grasps the red cylinder at (0.22, 0.12), places it upright at
   `place_pose` (-0.183, -0.14) within about 1 cm, and returns home. The node logs
   `Task execution completed`.

The following log messages are expected and harmless: `No 3D sensor plugin(s) defined for octomap updates`,
`Failed loading deceleration limits`, `/recognize_objects not available`, and
`Computed path is not valid` lines while MTC rejects grasp candidates.

**Running the demo again:** with Gazebo, `move_group` and the perception server still running,
start only the task again:
```bash
ros2 launch pnp_cobot_mtc_pick_place_demo pick_place_demo.launch.py
```
The launch first moves the red cylinder back to its start pose (0.22, 0.12), waits 2 s for a fresh
point cloud, then starts the MTC node. To disable this, pass `reset_object:=false`. For a
different object or start pose, use `object_model`, `object_x`, `object_y` and `object_z`.

## Configuration
In `pnp_cobot_mtc_pick_place_demo/config/`:
- `mtc_node_params.yaml`: `execute` (`false` plans only), `place_pose`, `controller_names`, and
  the grasp and motion parameters.
- `get_planning_scene_server.yaml`: the point cloud topic, crop box, and segmentation thresholds.

