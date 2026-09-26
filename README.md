# ros2-moveit-pick-and-place
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20Jammy-E95420?logo=ubuntu&logoColor=white)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Gazebo Fortress](https://img.shields.io/badge/Gazebo-Fortress-F58113)
![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Task%20Constructor-5C2D91)
![License: MIT](https://img.shields.io/badge/License-MIT-green)

A 6-axis desktop cobot that **finds an object with a depth camera and picks and places it on
its own**, in simulation. No object positions are hard-coded: the robot builds its world model
from a point cloud, then plans and executes the whole task with the MoveIt Task Constructor.

<p align="center">
  <img src="docs/media/pick_place_gazebo.gif" alt="The cobot picks up the red cylinder and places it beside its base in Gazebo" width="480">
</p>

## How it works

```
 RGB-D camera ──► point cloud ──► perception ──► planning scene ──► MTC task ──► ros2_control ──► Gazebo
   (Gazebo)      (ros_gz bridge)   (PCL)        (MoveIt 2)         (plan)       (execute)
```

1. **See.** A simulated RGB-D camera streams a colored point cloud of the table.
2. **Understand.** A PCL pipeline crops the cloud, finds the table with RANSAC plane
   segmentation, clusters the objects on it, and fits each cluster to a box or a cylinder
   (RANSAC and a Hough transform). Every fitted shape becomes a MoveIt collision object.
3. **Plan.** The MoveIt Task Constructor searches over grasp poses and plans the full task:
   open the gripper, approach, grasp, lift, carry, place, release, retreat and return home. It
   uses OMPL, joint interpolation and Cartesian planners, while avoiding every detected object.
4. **Act.** The plan runs on the arm and gripper controllers in Gazebo through `ros2_control`.

![Robot camera view and the segmented point cloud with fitted shapes](docs/media/perception.png)

*Left: the robot's camera view. Right: the same scene after perception, seen from above. The
table is removed, the objects are fitted as boxes and cylinders, and the target cylinder and
the place pose are marked.*

## Results
Measured in simulation on this repository's default demo:

| | |
| --- | --- |
| Objects detected | Table and 6 objects (4 boxes, 2 cylinders) |
| Target cylinder, fitted vs. actual | 349 mm tall, 12.3 mm radius vs. 350 mm, 15 mm |
| Task planning time | ~15 s |
| Execution time | ~10 s |
| Placement error | < 1 cm from the requested place pose |

## Tech stack
- **ROS 2 Humble** on Ubuntu 22.04
- **Gazebo Fortress** with `gz_ros2_control` and the `ros_gz` bridge
- **MoveIt 2** and the **MoveIt Task Constructor** (OMPL, Cartesian and joint interpolation planners)
- **PCL** for point cloud segmentation and shape fitting
- **C++** for the perception and task nodes; **Python** launch files

## Quick start
```bash
sudo apt install git-lfs          # meshes and images are stored with Git LFS
git clone https://github.com/Royy57/ros2-moveit-pick-and-place.git ~/ros2-moveit-pick-and-place
cd ~/ros2-moveit-pick-and-place
colcon build --symlink-install && source install/setup.bash
bash src/pnp_cobot_ros2/pnp_cobot_mtc_pick_place_demo/scripts/robot.sh
```

Dependencies, the package overview, run options and what to expect are in
[src/pnp_cobot_ros2/README.md](src/pnp_cobot_ros2/README.md).

## Repository layout
```
ros2-moveit-pick-and-place/
├── docs/media/            README images and GIF
└── src/pnp_cobot_ros2/    ROS 2 packages (description, Gazebo, MoveIt config, perception + MTC)
```

## About
The arm model is based on the Elephant Robotics myCobot 280. The packages were ported from
ROS 2 Jazzy and Gazebo Harmonic to ROS 2 Humble and Gazebo Fortress.

## License
MIT, see [LICENSE](LICENSE).
