# STM32 Range CAN bridge

`vl53l1x_can_bridge_node` receives the fixed STM32 SocketCAN protocol on `can0` and publishes three `sensor_msgs/Range` topics. The executable and compatibility topic names are retained for existing launches; the front device is now HC-SR04, not VL53L1X.

| Sensor | Device and range | Range topic | Compatibility topic | Frame |
| --- | --- | --- | --- | --- |
| 0 | HC-SR04 ultrasound, 0.02-4.0 m | `/front/range` | `/vl53l1x_distance` (mm) | `front_range_link` |
| 1 | VL53L1X ToF infrared, 0.05-3.0 m | `/left/range` | `/vl53l1x_distance_left` (mm) | `left_range_link` |
| 2 | VL53L1X ToF infrared, 0.05-3.0 m | `/right/range` | `/vl53l1x_distance_right` (mm) | `right_range_link` |

Only samples with `VALID=1`, no `OUT_OF_RANGE`, `TIMEOUT`, or `I2C_ERROR` flag, a non-sentinel distance, and configured range limits are published. `sigma_mm=0xFFFF` does not invalidate a measurement. Invalid samples are not converted into a far/no-obstacle reading. The existing `distance_safety_node` consumes the Range topics; if valid front data stops for `sensor_timeout_ms`, it commands the mux safety stop. STM32 `EMERGENCY` and diagnostic frames are logged and reported on `/diagnostics`.

Start with:

```bash
roslaunch robot_navigation vl53l1x_can.launch
```

Bring up SocketCAN first (`can0`, 500 kbit/s) and provide the real sensor TF positions as launch arguments. This is a close-range supplement to `/scan`; it is not a 360-degree lidar and is not injected into the costmap as free-space clearing data.
