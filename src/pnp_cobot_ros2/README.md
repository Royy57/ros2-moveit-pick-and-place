# pnp_cobot_ros2
![OS](https://img.shields.io/ubuntu/v/ubuntu-wallpapers/jammy)
![ROS_2](https://img.shields.io/ros/v/humble/rclcpp)

Perception-driven pick and place for a 6-axis desktop cobot, in simulation, with ROS 2 Humble,
MoveIt 2, the MoveIt Task Constructor (MTC) and Gazebo Fortress.

A simulated RGB-D camera looks at a table. The perception node segments the table and the objects
on it from the point cloud, fits them to primitive shapes and adds them to the MoveIt planning
scene. MTC then plans and executes a full pick and place of the target cylinder: open the
gripper, approach, grasp, lift, move, place, release, retreat and return home.

The arm model (URDF and meshes) is based on the Elephant Robotics myCobot 280.

![PnP Cobot in RViz](./pnp_cobot_description/urdf/pnp_cobot_280_rviz.png)

## Features
- Gazebo Fortress simulation with `ros2_control` (`gz_ros2_control`) arm and gripper controllers
- Simulated RGB-D camera publishing a colored point cloud
- Point-cloud perception with PCL:
  - Support plane (table) segmentation with RANSAC
  - Euclidean clustering of the objects on the table
  - Cylinder and box fitting (RANSAC, Hough transform)
  - Automatic planning scene generation from the detected objects
- Pick and place with the MoveIt Task Constructor, planned with OMPL, joint interpolation and Cartesian planners
- RViz visualization of the planning scene and MTC solutions
- Runs on CPU; no GPU required

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

## Requirements
- Ubuntu 22.04 (Jammy)
- ROS 2 Humble (apt binaries)
- Gazebo Fortress (`ign gazebo`)

Install the dependencies from apt:

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

## Build
```bash
cd ~/pick_n_place_cobot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

### Full pick and place demo
```bash
bash src/pnp_cobot_ros2/pnp_cobot_mtc_pick_place_demo/scripts/robot.sh
```

This starts, in order:
1. Gazebo with the pick and place world, the robot, the camera and the controllers
2. `move_group` with RViz
3. The perception server
4. The MTC pick and place node

The robot picks up the red cylinder and places it at `place_pose`. Planning takes about 15 s and
execution about 10 s.

### Simulation and controllers only
```bash
bash src/pnp_cobot_ros2/pnp_cobot_bringup/scripts/pnp_cobot_280_gazebo.sh
```

### Configuration
The pick and place node is configured in
`pnp_cobot_mtc_pick_place_demo/config/mtc_node_params.yaml`:

- `execute`: `true` runs the plan in Gazebo; `false` plans only and shows the solution in RViz.
- `place_pose`: where the object is placed, in `base_link`.
- `controller_names`: must match the controllers in
  `pnp_cobot_moveit_config/config/pnp_cobot_280/ros2_controllers.yaml`.

The perception pipeline (crop box, plane and cluster thresholds) is configured in
`pnp_cobot_mtc_pick_place_demo/config/get_planning_scene_server.yaml`.

## Notes
- **Running the demo again:** after a successful run the cylinder sits at `place_pose`, so a second
  run finds nothing valid to do and planning fails. Move the cylinder back first:
  ```bash
  ign service -s /world/default/set_pose \
    --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 3000 \
    --req 'name: "red_cylinder", position: {x: 0.22, y: 0.12, z: 0.175}, orientation: {w: 1.0}'
  ```
- **Stopping the scripts:** on Ctrl+C, the launch scripts force-kill every process whose command
  line matches `ros2`, `gz`, `rviz2`, `moveit` and similar. Close any other ROS 2 or Gazebo work
  before using them, or run the `ros2 launch` commands inside the scripts yourself.
- **Harmless log messages:** `No 3D sensor plugin(s) defined for octomap updates`,
  `Failed loading deceleration limits` (Pilz), `/recognize_objects not available` (RViz), and
  `Computed path is not valid` lines while MTC rejects grasp candidates during planning.
- The perception server writes debug `.pcd` files to `/tmp`.

## License
BSD-3-Clause. See the `LICENSE` file in each package.
