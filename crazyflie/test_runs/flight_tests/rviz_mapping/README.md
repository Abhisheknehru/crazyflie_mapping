# RViz keyboard mapping test

This experiment owns the Crazyradio connection, uses the same keyboard controls
as `point_cloud_test.py`, and publishes:

- `/crazyflie/point_cloud` (`sensor_msgs/PointCloud2`)
- `/crazyflie/pose` (`geometry_msgs/PoseStamped`)
- `/crazyflie/range_front`, `_left`, `_back`, `_right` (`sensor_msgs/Range`)
- `/map` (`nav_msgs/OccupancyGrid`), produced by `multiranger_mapper`

The range endpoints are rotated by measured roll, pitch, and yaw before they are
added to the `map` frame. The RViz pose arrow therefore turns with the measured
Crazyflie yaw.

For this 2.5 m arena test, horizontal wall hits are trusted up to approximately
1.8 m, with 2.0 m hysteresis. RViz starts zoomed to the arena instead of the
much larger default view.

Build and run:

```bash
cd /home/abhishek/eysip_hardware
./src/crazyflie/test_runs/flight_tests/rviz_mapping/start.sh
```

To select another radio URI, append `uri:=radio://...` to that command.
The default 2D map is 2.5 m by 2.5 m, centered on the initial map origin, with
3 cm cells. Set `size` to the measured arena side length when the arena is
different; for example, `size:=3.0 resolution:=0.02`. The occupancy map is
saved under `maps/` during a normal shutdown.

Click inside the keyboard-control window, then use arrows to translate, `A/D`
to yaw slowly, `Z/X` to yaw quickly, `W/S` for height, and `L` to land. Use only
one Crazyradio-owning program at a time.
