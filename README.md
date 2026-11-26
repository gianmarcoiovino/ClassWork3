# Classwork 3 — Fra2mo: Exploration, Localization and ArUco

Robotics Lab 2026 — Prof. Mario Selvaggio

ROS 2 Humble · Gazebo **Fortress** (`ros-humble-ros-gz`) · SLAM Toolbox · Nav2 · explore_lite · aruco_ros

## Repository content

| Folder | Content |
|---|---|
| `docker/` | Dockerfile with all the dependencies of this classwork |
| `ros2_fra2mo/` | course package (URDF, world `leonardo_race_field`, launch files, Nav2/SLAM/AMCL config) |
| `ros2_fra2mo/maps/my_map.{pgm,yaml}` | map built with SLAM + autonomous exploration |
| `m-explore-ros2/` | `explore_lite` (frontier explorer); `map_merge` disabled with `COLCON_IGNORE` |

This folder is the `src` of the workspace: the run script mounts it as `~/ros2_ws/src` in the container.

## From a clean machine

```bash
git clone https://github.com/gianmarcoiovino/ClassWork3.git ~/Desktop/CW3
# copy docker/Dockerfile next to the course docker scripts, then:
./docker_build_image.sh project-img
./docker_run_container.sh project-img project-img_cont Desktop/CW3
# inside the container
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 pkg executables explore_lite      # -> explore_lite explore
```

Other terminals: `docker exec -it project-img_cont bash` and `source ~/ros2_ws/install/setup.bash`.

### Differences from the assignment PDF

The PDF refers to packages `fra2mo_description` / `fra2mo_navigation`; the course repository provides a single package, `ros2_fra2mo`:

| PDF | This repo |
|---|---|
| `fra2mo_gazebo.launch.py` | `gazebo_fra2mo.launch.py` |
| `keyboard_teleop` | `teleop_twist_keyboard` |
| `slam.launch.py` + `explore_lite explore.launch.py` | `fra2mo_explore.launch.py` (SLAM + Nav2 + explore + RViz) |
| `amcl.launch.py` | `fra2mo_navigation.launch.py` (AMCL + map_server + Nav2) |
| `nav2_params.yaml` | `config/amcl.yaml`, `config/navigation.yaml`, `config/slam.yaml`, `config/explore.yaml` |
| `/scan`, `/odom` | `/lidar`, `/model/fra2mo/odometry` |

Gazebo: the robot macros use Fortress tags (`<ignition_frame_id>`). On Harmonic the LiDAR frame becomes `fra2mo/base_footprint/laser_frame`, which is not in the TF tree, so SLAM drops every scan. With Fortress the package works unmodified.

## Task 1 — Starter

```bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # quit with Ctrl+C before using Nav2
```

**Find out.** Main topics: `/cmd_vel` (Twist, ROS → Gazebo, DiffDrive plugin), `/model/fra2mo/odometry` (wheel odometry, `fra2mo/odom` → `fra2mo/base_footprint`), `/lidar` (LaserScan, frame `laser_frame`), plus `/tf`, `/clock`. Nav2 config: `map_server` loads the map, `amcl` localizes, `planner_server` computes the global path on the global costmap, `controller_server` (Regulated Pure Pursuit) follows it on the local costmap, `bt_navigator` coordinates everything through a behavior tree.

## Task 2 — Exploration and mapping

```bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
ros2 launch ros2_fra2mo fra2mo_explore.launch.py
# when explore prints "No frontiers found" / "All frontiers traversed", with SLAM still running:
cd ~/ros2_ws/src/ros2_fra2mo/maps
ros2 run nav2_map_server map_saver_cli -f my_map
```

The map file is set in `launch/fra2mo_amcl.launch.py`, `launch/fra2mo_navigation.launch.py` and `config/navigation.yaml` (`yaml_filename`), then `colcon build --packages-select ros2_fra2mo`. Result: 409 × 212 cells at 0.05 m/px.

**Find out.** SLAM Toolbox matches each scan with the previous ones, corrects odometry drift, publishes `/map` and `map → odom`. explore_lite is not a planner: it finds frontiers (free/unknown boundaries) on `/map` and sends the best one as a `NavigateToPose` goal to Nav2, until no frontier is left. In narrow passages the RPP controller can stop ("collision ahead"): Nav2 recovery behaviors and explore_lite blacklisting of unreachable frontiers let the exploration continue.

## Task 3 — Localization with AMCL

```bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
rviz2 -d $(ros2 pkg prefix ros2_fra2mo)/share/ros2_fra2mo/rviz_conf/navigation.rviz --ros-args -p use_sim_time:=true
```

RViz: Fixed Frame `map` → *2D Pose Estimate* → *2D Goal Pose*. From a terminal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}"
```

**Find out.** AMCL needs what SLAM does not: a **prior map** (from `map_server`) and an **initial pose estimate**. Its particle filter compares the scans with the known map; until the initial pose is given it does not publish `map → odom`, so Nav2 cannot plan.

## Task 4 — ArUco markers in the map frame

_To be completed._

## Commits

One commit per task, with a message that says what works.
