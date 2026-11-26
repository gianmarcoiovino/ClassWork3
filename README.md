# Classwork 3 — Fra2mo: Exploration, Localization and ArUco

Robotics Lab 2026 — Prof. Mario Selvaggio

ROS 2 Humble · Gazebo **Fortress** (`ros-humble-ros-gz`) · SLAM Toolbox · Nav2 · explore_lite · aruco_ros

## Repository content

| Folder | Content |
|---|---|
| `project_docker/` | Dockerfile with all the dependencies of this classwork |
| `ros2_fra2mo/` | course package (URDF, world `leonardo_race_field`, launch files, Nav2/SLAM/AMCL config), with the Task 4 additions listed below |
| `ros2_fra2mo/maps/my_map.{pgm,yaml}` | map built with SLAM + autonomous exploration |
| `m-explore-ros2/` | `explore_lite` (frontier explorer); `map_merge` disabled with `COLCON_IGNORE` |

This folder is the `src` of the workspace: the run script mounts it as `~/ros2_ws/src` in the container.

## From a clean machine

```bash
git clone https://github.com/gianmarcoiovino/ClassWork3.git ~/Desktop/CW3
# copy project_docker/Dockerfile next to the course docker scripts, then:
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

### What was missing and what was added

In the course package the Task 4 setup was not usable in Gazebo Fortress, so it was completed (all changes inside `ros2_fra2mo`):

| Missing | Added | Why |
|---|---|---|
| camera on fra2mo (the D435 macro is commented and uses `libgazebo_ros_depth_camera.so`, Gazebo Classic) | `urdf/camera_gazebo_macro.xacro`: Fortress `camera` sensor, 640×480, 15 Hz, HFOV 80°, on `d435_camera_link` (3.5 cm in front of `d435_link`); optical frame `d435_optical_frame` | `d435_link` lies inside the robot shell: the first version only saw the inside of the body |
| camera topics in ROS | bridge of `/camera/image` and `/camera/camera_info` in `gazebo_fra2mo.launch.py` | |
| markers in the world (the only include was commented out) | 5 markers, ids 0–4, in `worlds/leonardo_race_field.sdf`, centre at z 0.28 | the walls are solid only for z ∈ [0.10, 0.45], windows above |
| marker texture in Fortress (Ogre material scripts are ignored) | `<pbr><albedo_map>` in `models/marker_<id>/marker_<id>.sdf`, explicit white support | |
| detector launch | `launch/fra2mo_aruco.launch.py` | |

Marker poses (world = map, since the robot starts at the origin of both):

| id | x | y | z (world) | wall |
|---|---|---|---|---|
| 0 | −7.50 | 4.99 | 0.28 | north |
| 1 | 1.00 | 4.99 | 0.28 | north |
| 2 | 9.99 | 2.50 | 0.28 | east |
| 3 | 2.50 | −4.99 | 0.28 | south |
| 4 | −9.99 | 2.50 | 0.28 | west |

### Detector parameters (Find out)

| Parameter | Value | How it was found |
|---|---|---|
| `marker_size` | **0.30 m** | the texture `ar_tag_<id>.png` (ARUCO_ORIGINAL) is black up to its edge and fills the 0.30 m tag visual; the 0.32 m white support gives the quiet zone. Ids checked offline with OpenCV (`ar_tag_k.png` → id k) |
| `camera_frame` | **`d435_optical_frame`** | optical frame in the TF tree (z forward, x right, y down) |
| `/image`, `/camera_info` | **`/camera/image`**, **`/camera/camera_info`** | `ros2 topic list` |
| `reference_frame` | `map` | each detection becomes a place on the map |

### Run

```bash
ros2 launch ros2_fra2mo gazebo_fra2mo.launch.py
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py          # AMCL + Nav2, give the initial pose
rviz2 -d $(ros2 pkg prefix ros2_fra2mo)/share/ros2_fra2mo/rviz_conf/navigation.rviz --ros-args -p use_sim_time:=true
ros2 launch ros2_fra2mo fra2mo_aruco.launch.py
ros2 topic echo /aruco_marker_publisher/markers_list           # ids in view
ros2 topic echo /aruco_marker_publisher/markers                # poses in map
```

RViz: *Add → By topic → `/aruco_marker_publisher/result` → Image*. Goals in front of the markers, e.g. id 1:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 3.5}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

Measurement at each goal (marker estimate, AMCL estimate, ground truth from Gazebo):

```bash
ros2 topic echo /aruco_marker_publisher/markers --once | grep -E "  id:|position" -A3
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m1 Translation
ign model -m fra2mo -p | grep -A1 XYZ
```

### Results

Markers found: **3, 0, 1** (in this order). Errors in the map plane (m):

| id | marker estimate (x, y) | true (x, y) | marker error | AMCL error (robot) | residual = marker − AMCL |
|---|---|---|---|---|---|
| 3 | (2.673, −4.888) | (2.50, −4.99) | (+0.173, +0.102) → **0.20** | (+0.166, +0.111) → 0.20 | (+0.007, −0.008) → **0.011** |
| 0 | (−7.205, 4.760) | (−7.50, 4.99) | (+0.295, −0.230) → **0.37** | (+0.363, −0.251) → 0.44 | (−0.068, +0.021) → **0.071** |
| 1 | (1.222, 4.827) | (1.00, 4.99) | (+0.222, −0.163) → **0.28** | (+0.233, −0.165) → 0.29 | (−0.011, +0.002) → **0.011** |

z of the estimates: 0.181–0.183 m = 0.28 (world) − 0.10 (floor height, the origin of `map` is at `base_footprint`).

**Find out.** `marker_publisher` chains the marker pose in the optical frame with the TF `map → d435_optical_frame`, so the pose in `map` **inherits the localization error of AMCL**: the marker error is almost the same as the robot error, and the camera alone is accurate to about 1 cm. The larger residual for id 0 (7 cm) is compatible with a small AMCL heading error (≈ 3° at ≈ 1.3 m), not measured here. The AMCL error grows with the distance travelled (0.20 → 0.44 m): the field is large, with long straight walls and few distinctive features. Consequence: to approach a target precisely, use the pose of the marker relative to the robot (camera frame), not its pose in the map.

## Commits

One commit per task, with a message that says what works.
