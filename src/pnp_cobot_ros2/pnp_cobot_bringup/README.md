# pnp_cobot_bringup

Entry-point launch files that start the PnP Cobot simulation, MoveIt and demos in the right order.

| Launch file | Starts |
| --- | --- |
| `sim.launch.py` | Gazebo, the RGB-D camera and the controllers (RViz optional) |
| `moveit.launch.py` | `sim.launch.py` + `move_group` and RViz (`rviz_view:=moveit` or `mtc`) |
| `pick_and_place.launch.py` | `moveit.launch.py` + the perception server and the MTC pick and place node |
| `mtc_demos.launch.py` | `moveit.launch.py` + one standalone MTC example (`exe:=...`) |
| `point_cloud.launch.py` | `sim.launch.py` + RViz replaying a `.pcd` cloud saved by `pick_and_place.launch.py save_debug_clouds:=true` |

Each stage starts only when the previous one is ready. `scripts/wait_for_ready.py` blocks until
the listed controllers are active or an action server exists, and the launch file continues when
it exits (it shuts down after a timeout). Helpers shared by the launch files are in
`launch/bringup_utils.py`.

Each file includes others through `include_launch()`, which scopes the include so its arguments
do not leak into the rest of the launch (in ROS 2 Humble they otherwise do). Arguments are read
once at startup, in `launch_setup()`, because stages started later by `wait_then()` run after the
file's scope has ended.
