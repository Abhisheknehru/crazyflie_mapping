# Reactive exploration prototype

The `reactive_explorer` node consumes the four existing
`/crazyflie/range_{front,left,back,right}` sensor_msgs/Range topics and publishes:

- `/exploration/proposed_velocity`: TwistStamped in base_link, metres/second.
- `/exploration/state`: String diagnostic state.

This is proposal-only software: it does not connect to the radio, take off,
maintain altitude, or execute flight commands. A flight adapter is still needed.
It must hold heading and altitude, arbitrate manual override, and stop motion
when proposals expire. Zero horizontal velocity alone is not a landing command.

## Run in a working ROS 2 Humble environment

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select crazyflie
source install/setup.bash
ros2 run crazyflie reactive_explorer
```

In another sourced terminal:

```bash
ros2 topic echo /exploration/proposed_velocity
ros2 topic echo /exploration/state
```

Start your existing range publisher separately. Each message must contain a
current ROS timestamp, valid min_range/max_range, and a finite positive range
in metres. Unknown or infinite readings stop proposals; the bridge must not
encode a sensor fault as clear space. All four faces must remain fresh.
The producer and node must share a time base.

Defaults: forward speed 0.10 m/s, stop clearance 0.30 m, wall distance 0.45 m,
opening threshold 0.75 m, sensor timeout 0.5 s, session duration 120 s from
first usable decision. These are provisional values, not flight-validated
clearances. Restart the node to reset its direction and duration.

## Behaviour and limits

The drone's heading stays fixed while the policy changes translation among
front, left, back and right. It tracks a right-side boundary, follows detected
openings, and chooses another direction when forward motion is blocked.
At or below stop clearance it stops instead of attempting recovery.

This heuristic has no visited-place memory, map-based planning, loop closure,
or guarantee of complete maze coverage. It can loop, miss branches, and is
limited by sparse range coverage around corners. It does not validate pose,
altitude, heading, battery or flight readiness. Do not wire the proposal topic
directly to motors. Verify geometry, stopping distances and the flight adapter
before controlled flight trials.

Existing mapping and estimation can run alongside this node, but several
modules referenced by setup.py and the older launch files are absent from src
and exist only in build/install outputs. This change does not restore them.

## Validation

```bash
PYTHONPATH=src/crazyflie python3 -m unittest test_runs.test_reactive_policy -v
```

Six policy tests cover corridor motion, invalid/close ranges, dead ends,
right openings, lateral clearance and parameter validation. Python compilation
passed. ROS runtime checks could not run in the development environment because
rclpy is unavailable even after sourcing /opt/ros/humble/setup.bash. No hardware
flight test was performed.

Range message contract:
https://docs.ros.org/en/humble/p/sensor_msgs/msg/Range.html

## Single-command radio startup

From the workspace root:

```bash
./src/crazyflie/test_runs/start_exploration.sh radio://0/80/2M/E7E7E7E7E7
```

Replace the URI with your Crazyflie's URI. The script sources ROS 2 Humble
(or ROS_DISTRO), checks Python dependencies, builds this package, and launches:

1. `exploration_radio_bridge`: one cflib radio connection.
2. `of_odometry`: existing pose processor, with geometry constraints and
   Mahalanobis filtering disabled for an initial baseline.
3. `reactive_explorer`: exploration velocity proposals.

Do not also start the old pose logger or another radio client for this URI,
or the UDP range publisher on the same topics.

Optional trajectory recording:

```bash
./src/crazyflie/test_runs/start_exploration.sh radio://0/80/2M/E7E7E7E7E7 record_path:=/tmp/exploration.csv
```

The launch file can also be used after building and sourcing the workspace:

```bash
ros2 launch crazyflie exploration.launch.py uri:=radio://0/80/2M/E7E7E7E7E7
```

Telemetry wiring:

| Topic | Type | Consumer |
|---|---|---|
| `/odom` | nav_msgs/Odometry | Existing of_odometry node |
| `/crazyflie/pose` | geometry_msgs/PoseStamped | Raw planar pose visualization/recording |
| `/crazyflie/range_front`, `range_left`, `range_back`, `range_right` (each under `/crazyflie/`) | sensor_msgs/Range | Pose processor and explorer |
| `/crazyflie/range_up`, `/crazyflie/range_down` | sensor_msgs/Range | Available for monitoring |
| `/of/pose`, `/of/path` | PoseStamped, nav_msgs/Path | Existing downstream mapping interface |
| `/exploration/proposed_velocity` | geometry_msgs/TwistStamped | Monitoring; no flight consumer yet |

The bridge reads onboard stateEstimate.x/y/z/yaw (already estimated using the
configured firmware estimator) and range.* logs at 10 Hz. At every startup it selects the Kalman estimator, pulses
kalman.resetEstimation, and waits for ten stable variance samples (up to
30 seconds). No pose or ranges are published until that check passes.
The first post-settling pose defines x=y=z=yaw=0; both translation and yaw
are transformed into this new start-heading frame. Orientation is
planar, and odometry twist/covariance are unpopulated; do not fuse those fields
as measured velocity or zero-uncertainty pose. No TF tree is broadcast.

Ranges are converted from millimetres to metres and clamped to 0.02–4 m.
Ordinary readings above 4 m become 4 m; readings below 0.02 m become 0.02 m.
Zero, non-finite input and firmware error values (32767 or larger) become
0.02 m, triggering a clearance stop. The bridge never publishes NaN ranges.
Limits are ROS parameters tof_min_range and tof_max_range (maximum 4 m).
The minimum is a software floor, not a claim of guaranteed sensor accuracy.
A saturated maximum is not a measured wall endpoint; future mapping must
handle saturation separately. Invalid/error readings can still pause exploration. Timestamps represent host receipt,
not synchronized firmware acquisition. Connection failure, missing required log
variables, or loss of either telemetry stream causes the launch to stop.

This startup does not launch a mapper (its source is missing), execute proposed
velocities, take off, or land. Those remain separate integration work. Ctrl-C
closes telemetry and stops the nodes; it is not a landing operation.

Firmware log reference:
https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/

## Fresh state on every startup

Start with the drone stationary on the ground: startup now resets its onboard
estimator, so do not restart this script during flight. ROS pose, path and
exploration state are also new for each launch. Nothing is restored from a
previous run. A settling timeout or reset configuration error stops startup.
Variance stability is a startup check, not a guarantee of localization accuracy.
The ROS start-relative position is zero even if the onboard height estimate
is nonzero. The reset does not erase maps saved by other programs.
