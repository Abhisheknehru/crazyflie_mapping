"""Pure coordinate transforms between the ROS map frame and the Crazyflie's
raw onboard estimator frame, used to command flight against a map-frame goal.

radio_bridge.py builds the map frame from the raw stateEstimate in two
chained steps (see publish_pose): an always-applied origin transform, then
an optional one-time /initialpose alignment. Flight commands (go_to) need
the raw frame, so a goal given in map coordinates must go through the exact
inverse of that chain -- that's what to_estimator_frame does. to_map_frame
is kept alongside it (mirrors publish_pose's own math) purely so the two can
be round-trip tested against each other.
"""
import math


def to_map_frame(raw_x, raw_y, raw_yaw, origin, map_alignment):
    ox, oy, _, oyaw = origin
    dx, dy = raw_x - ox, raw_y - oy
    c, s = math.cos(oyaw), math.sin(oyaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    local_yaw = raw_yaw - oyaw
    if map_alignment is None:
        return local_x, local_y, local_yaw
    local_x0, local_y0, local_yaw0, map_x0, map_y0, map_yaw0 = map_alignment
    turn = map_yaw0 - local_yaw0
    c, s = math.cos(turn), math.sin(turn)
    dx, dy = local_x - local_x0, local_y - local_y0
    map_x = map_x0 + c * dx - s * dy
    map_y = map_y0 + s * dx + c * dy
    map_yaw = math.atan2(math.sin(local_yaw + turn), math.cos(local_yaw + turn))
    return map_x, map_y, map_yaw


def to_estimator_frame(map_x, map_y, map_yaw, origin, map_alignment):
    if map_alignment is not None:
        local_x0, local_y0, local_yaw0, map_x0, map_y0, map_yaw0 = map_alignment
        turn = map_yaw0 - local_yaw0
        c, s = math.cos(turn), math.sin(turn)
        dx, dy = map_x - map_x0, map_y - map_y0
        local_x = local_x0 + c * dx + s * dy
        local_y = local_y0 - s * dx + c * dy
        local_yaw = map_yaw - turn
    else:
        local_x, local_y, local_yaw = map_x, map_y, map_yaw
    ox, oy, _, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    raw_x = ox + c * local_x - s * local_y
    raw_y = oy + s * local_x + c * local_y
    raw_yaw = local_yaw + oyaw
    return raw_x, raw_y, raw_yaw


def path_is_clear(localization_map, x0, y0, x1, y1):
    distance = math.hypot(x1 - x0, y1 - y0)
    if distance == 0:
        index = localization_map.cell(x0, y0)
        return index is None or not localization_map.occupied[index]
    steps = max(1, math.ceil(distance / localization_map.resolution))
    for n in range(steps + 1):
        fraction = n / steps
        index = localization_map.cell(x0 + fraction * (x1 - x0),
                                       y0 + fraction * (y1 - y0))
        if index is not None and localization_map.occupied[index]:
            return False
    return True
