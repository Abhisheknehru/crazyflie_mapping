# Crazyflie Multi-ranger Mapping and Navigation

This package contains the old ROS 2 Crazyflie workflow used for live mapping, saved-map localization, scan matching, and A* navigation.

## 1. Build and Source

From the workspace root:

```bash
cd ~/eysip_hardware
colcon build --packages-select crazyflie
source install/setup.bash
```

If you open a new terminal, source again:

```bash
cd ~/eysip_hardware
source install/setup.bash
```

## 2. Create a Map With RViz Keyboard Mapping

Use this when the Crazyflie radio is connected and you want to fly/teleop while building a map.

The usual way to start mapping is the helper script:

```bash
cd ~/eysip_hardware/src/crazyflie/test_runs/flight_tests/rviz_mapping
./start.sh
```

Optional parameters can be passed through the script to the launch file:

```bash
cd ~/eysip_hardware/src/crazyflie/test_runs/flight_tests/rviz_mapping
./start.sh \
  uri:=radio://0/80/2M/E7E7E7E7E7 \
  size:=2.5 \
  resolution:=0.03 \
  map_save_prefix:=maps/rviz_keyboard
```

The script does these steps for you:

```text
source /opt/ros/humble/setup.bash
cd ~/eysip_hardware
export ROS_LOG_DIR=~/eysip_hardware/log/rviz_mapping
colcon build --packages-select crazyflie --symlink-install
source ~/eysip_hardware/install/setup.bash
ros2 launch .../rviz_keyboard_mapping.launch.py
```

Equivalent manual command after building/sourcing:

```bash
ros2 launch crazyflie rviz_keyboard_mapping.launch.py
```

This launch starts:

- `rviz_keyboard_mapping`: connects to Crazyflie, handles keyboard flight, publishes pose/range data and point cloud.
- `multiranger_mapper`: subscribes to `/crazyflie/pose` and `/crazyflie/range_*`, builds `/map`.
- `rviz2`: visualizes the live map and point cloud.

Topics produced during mapping:

```text
/crazyflie/pose
/crazyflie/range_front
/crazyflie/range_left
/crazyflie/range_back
/crazyflie/range_right
/crazyflie/point_cloud
/map
```

The launch uses these mapper trust thresholds:

```text
trust_near = 1.8 m
trust_far  = 2.0 m
```

The mapper saves the map on shutdown. Stop the launch cleanly with `Ctrl+C`. The saved map appears under `maps/`, with names like:

```text
maps/rviz_keyboard_YYYYMMDD_HHMMSS_xxxxxx.yaml
maps/rviz_keyboard_YYYYMMDD_HHMMSS_xxxxxx.pgm
```

## 3. Check Saved Maps

List saved maps:

```bash
ls maps/*.yaml
```

A map YAML path will look like:

```text
maps/rviz_keyboard_20260922_171438_959977.yaml
```

Use that YAML path for scan matching and navigation.

## 4. Start Pose Estimation With Scan Matching

Use this after you already have a saved map.

```bash
ros2 launch crazyflie scan_matching.launch.py \
  map_yaml:=maps/rviz_keyboard_20260922_171438_959977.yaml \
  clearance:=0.05
```

This launch starts:

- `publish_saved_map`: loads the saved YAML/PGM map and publishes `/map`.
- `scan_match_localizer`: uses `/crazyflie/pose` and `/crazyflie/range_*` to estimate corrected pose.
- `a_star_planner`: plans paths on the saved map.
- `rviz2`: opens the scan-matching RViz layout.

Important: this launch does not create a new map. It uses an existing saved map.

## 5. Initial Pose Options

By default, scan matching uses automatic initial pose parameters:

```text
initial_map_x = 0.0
initial_map_y = 0.0
initial_map_yaw_degrees = 0.0
```

You can override them:

```bash
ros2 launch crazyflie scan_matching.launch.py \
  map_yaml:=maps/your_map.yaml \
  initial_map_x:=0.2 \
  initial_map_y:=-0.1 \
  initial_map_yaw_degrees:=15.0 \
  clearance:=0.05
```

The scan matcher publishes:

```text
/crazyflie/scan_matched_pose
/crazyflie/scan_match_points
/crazyflie/scan_match_error
/crazyflie/scan_match_accepted
```

## 6. Planning and Navigation Flow

The planner listens for goals on:

```text
/crazyflie/navigation_goal
```

It publishes:

```text
/crazyflie/planned_path
/crazyflie/inflated_map
/crazyflie/navigation_velocity
```

Navigation is guarded. A goal is accepted only when scan matching is stable enough. After sending a goal, enable movement with:

```bash
ros2 topic pub --once /crazyflie/navigation_enable std_msgs/msg/Bool "{data: true}"
```

To stop navigation:

```bash
ros2 topic pub --once /crazyflie/navigation_enable std_msgs/msg/Bool "{data: false}"
```

## 7. Common Commands

Run mapper only:

```bash
ros2 run crazyflie multiranger_mapper
```

Run saved map publisher only:

```bash
ros2 run crazyflie publish_saved_map --ros-args -p map_yaml:=maps/your_map.yaml
```

Run scan matcher only:

```bash
ros2 run crazyflie scan_match_localizer --ros-args -p map_yaml:=maps/your_map.yaml
```

Run A* planner only:

```bash
ros2 run crazyflie a_star_planner --ros-args -p map_yaml:=maps/your_map.yaml -p clearance:=0.05
```

## 8. Important Notes

- `rviz_keyboard_mapping.launch.py` is for creating a live map with the Crazyflie.
- `scan_matching.launch.py` is for using an existing saved map.
- `scan_matching.launch.py` does not launch `multiranger_mapper`.
- The saved map must match the physical arena layout.
- Source the workspace in every new terminal before running commands.
