# Simple state-estimate mapping

The active mapping pipeline contains only:

1. `radio_bridge.py`: reads onboard `stateEstimate.x/y/z/yaw` and Multi-ranger
   values from one Crazyradio connection.
2. `multiranger_mapper.py`: combines `/crazyflie/pose` with the front, left,
   back and right ranges to publish `/map`.
3. RViz: displays `/map` and `/crazyflie/pose` in the `map` frame.

Run everything with:

```bash
cd /home/abhishek/eysip_hardware
./src/crazyflie/test_runs/start_mapping.sh radio://0/80/2M/E7E7E7E7E7
```

Replace the URI if required. Keep the drone stationary while the onboard
estimator resets and settles. Then carry it level and slowly through the maze.

The default map is 10 by 10 metres with 2 cm cells. Optional launch arguments:

```bash
./src/crazyflie/test_runs/start_mapping.sh \
  radio://0/80/2M/E7E7E7E7E7 size:=12.0 resolution:=0.02
```

The mapper has no loop closure, range-jump filter, wall confirmation, map
exploration controller, planner, or `/of/pose` dependency. Restart
the command to reset both the state-estimate origin and occupancy map.

Press Ctrl+C once in the launch terminal to stop. A nonempty map is saved as a
timestamped PGM/YAML pair under `/home/abhishek/eysip_hardware/maps` by default.
The terminal prints both complete paths. Override the location with, for example,
`map_save_prefix:=/tmp/my_maze`.
