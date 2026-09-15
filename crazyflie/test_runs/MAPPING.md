# Initial hand-carried mapping

Run start_exploration.sh with your radio URI; it builds the mapper too. After
estimator settling, hold the drone level with the floor sensors unobstructed.
In a sourced second terminal run:

```bash
ros2 run crazyflie multiranger_mapper
```

In a sourced third terminal run `rviz2`. Set Fixed Frame to `map`, add a Map
display on `/map` (Reliable, Transient Local), and a Pose display on
`/crazyflie/pose`. Move slowly at consistent height and heading. First scan a
single flat wall and compare its mapped distance/shape with measurements.
Then try a corner and a short return-to-start route. Double walls on revisits
indicate localization/alignment errors; a visually plausible map is not proof
of accurate localization.

Default map is 10 by 10 m centered at the start, with 2 cm cells. Change with
`--ros-args -p size:=16.0 -p resolution:=0.05`. Sensor offset defaults to 3 cm
from the body origin in each sensor direction; measure your deck geometry and
set `-p sensor_offset:=...` accordingly. This approximates the sensors as rays;
the actual field of view, roll/pitch and individual extrinsics are not modeled.

Pose defaults to /crazyflie/pose because this is timestamped at radio receipt.
Ranges wait for pose data and use the nearest sample within 0.15 s. There is no
pose interpolation. All messages must share the ROS clock and map frame.

Minimum-clamped readings are skipped, so sensor error codes do not draw false
walls. Maximum-clamped ranges clear free cells without adding occupied endpoints.
Exact maximum-distance walls cannot be distinguished from saturation. A later
mapper should consume explicit raw range validity as well.

The mapper does not correct pose drift, recognize revisits, plan paths, save
files, or execute flight commands. Keep it stopped during startup/ground
handling to avoid drawing those movements. To clear the grid, stop and restart
the mapper. If you restart the radio bridge, restart the mapper too: the pose
origin has changed. Record /map, /crazyflie/pose and the four horizontal range
topics with ros2 bag record for replay; this is not a navigation map-file export.

Validation: 15 unit tests pass across grid, reset/range conversion, and reactive
policy. Mapper Python compilation passed. ROS middleware, RViz and hardware
mapping have not been tested in the development environment.

## Preconfigured RViz and mapper

After starting the radio script, replace the separate mapper and RViz commands
with:

```bash
source /opt/ros/humble/setup.bash
source /home/abhishek/eysip_hardware/install/setup.bash
ros2 launch crazyflie multiranger_mapping.launch.py
```

Rebuild once after this update (the radio startup script builds automatically).
This launches the mapper and RViz with fixed frame `map`, map `/map`, pose
`/crazyflie/pose`, trajectory `/of/path`, and a top-down view. It opens no radio
connection. Do not run another mapper alongside this launch.

To open only the configured RViz, with an existing mapper running:

```bash
rviz2 -d /home/abhishek/eysip_hardware/src/crazyflie/test_runs/multiranger.rviz
```

A plain `rviz2` command does not automatically select this project configuration;
use the launch command or `-d` form above.
