# VL53L1X CAN bridge

`vl53l1x_can_bridge_node` receives the fixed STM32 SocketCAN protocol on `can0` and publishes three `sensor_msgs/Range` topics:

| Sensor | Range | Compatibility topic | Frame |
| --- | --- | --- | --- |
| 0 | `/front/range` | `/vl53l1x_distance` (mm) | `front_range_link` |
| 1 | `/left/range` | `/vl53l1x_distance_left` (mm) | `left_range_link` |
| 2 | `/right/range` | `/vl53l1x_distance_right` (mm) | `right_range_link` |

Only samples with `VALID=1`, a non-sentinel distance, and configured range limits are published. Invalid samples are not converted into a far/no-obstacle reading. The existing `distance_safety_node` consumes the compatibility topics; if valid front data stops for `sensor_timeout_ms`, it commands the mux safety stop. STM32 `EMERGENCY` and diagnostic frames are logged and reported on `/diagnostics`.

Start with:

```bash
roslaunch robot_navigation vl53l1x_can.launch
```

Bring up SocketCAN first (`can0`, 500 kbit/s) and provide the real sensor TF positions as launch arguments. This is a close-range supplement to `/scan`; it is not a 360-degree lidar and is not injected into the costmap as free-space clearing data.
