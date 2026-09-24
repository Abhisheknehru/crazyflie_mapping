# Sparse scan-matching localization

`scan_match_localizer` is separate from the particle-filter localizer. It uses
Flow-deck odometry to accumulate front, left, back, and right Multi-ranger hit
points, then aligns those points with the latest saved `maps/maze_*.yaml` map.

The main logic is `ScanMatchLocalizer.run_localization_cycle()`. Its sequence is:

1. Wait for odometry.
2. Remove old range points and form the accumulated scan.
3. Search around the current map-to-odometry alignment.
4. Accept only matches with a sufficiently small mean wall error.
5. Publish the corrected pose and aligned scan points.

Build and source the package:

```bash
cd /home/abhishek/eysip_hardware
colcon build --packages-select crazyflie
source install/setup.bash
```

Start the radio bridge in terminal 1:

```bash
ros2 run crazyflie state_estimate_bridge --ros-args \
  -p uri:=radio://0/80/2M/E7E7E7E7E7
```

Start scan matching from the workspace directory in terminal 2. Select the
saved map explicitly so the test cannot silently use a different map:

```bash
ros2 run crazyflie scan_match_localizer --ros-args \
  -p map_yaml:=maps/rviz_keyboard_20260922_171438_959977.yaml
```

Alternatively, start the saved-map publisher, localizer, and the prepared RViz
view together (this does not start mapping or connect to the Crazyradio):

```bash
ros2 launch crazyflie scan_matching.launch.py \
  map_yaml:=maps/rviz_keyboard_20260922_171438_959977.yaml
```

By default, the first received odometry pose is automatically anchored at map
`(0, 0, 0 degrees)`. Start the localizer before starting the flight controller.
This default is correct when the drone is placed at the original mapping start
position and heading. For a known takeoff pad elsewhere, configure it directly:

```bash
ros2 run crazyflie scan_match_localizer --ros-args \
  -p map_yaml:=maps/rviz_keyboard_20260922_171438_959977.yaml \
  -p initial_map_x:=0.8 -p initial_map_y:=-0.4 \
  -p initial_map_yaw_degrees:=90.0
```

Manual initialization remains available on
`/crazyflie/scan_match_initialpose`. Do not publish the scan matcher's initial
pose on `/initialpose`, because the radio bridge also consumes that topic. Then
rotate slowly and translate past a corner so the accumulated sparse scan sees
distinctive geometry. The node publishes:

- `/crazyflie/scan_matched_pose` (`geometry_msgs/PoseStamped`)
- `/crazyflie/scan_match_points` (`geometry_msgs/PoseArray`)
- `/crazyflie/scan_match_error` (`std_msgs/Float32`, metres; lower is better)
- `/crazyflie/scan_match_accepted` (`std_msgs/Bool`)

The default local search covers approximately +/-0.35 m and +/-15 degrees.
It is intended to correct odometry drift from a roughly known pose, not solve
global localization in a symmetric arena.
