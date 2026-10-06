# Hardware-button complete startup

`button_autostart.py` is a boot-time daemon. It waits for a debounced press on
BCM GPIO 17, starts `robot_navigation/mission_complete.launch` with the internal
GPIO button listener disabled, waits for `mission_controller_node`, and sends
the authenticated `/start_signal/physical` pulse (`0x5A17`). GPIO 27 is lit
while the system is waiting or the stack is running.

Build the workspace, then install and enable the supplied unit:

```bash
cd /home/fire/ros_ws
catkin_make --pkg robot_navigation
sudo cp src/robot_navigation/systemd/robot-button-autostart.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robot-button-autostart.service
journalctl -u robot-button-autostart.service -f
```

The unit runs as root because the legacy sysfs GPIO interface requires GPIO
access. Change `--button-pin`, `--led-pin`, or launch arguments in
`ExecStart` when the wiring or competition configuration differs. Do not also
start `start_button_node` for this mode; the daemon owns GPIO 17.

## Integration note for ROS_WS_Upload

This service is optional and is **not enabled by merging or catkin build**.
Do not enable it during the Y42 keyboard bench test: it starts the full mission
stack, not `y42_stm32_keyboard_test.launch`, and would create competing chassis
controllers if the keyboard test is already running.

The sample unit hardcodes `/home/fire/ros_ws`, ROS Noetic, and legacy sysfs GPIO.
Adapt the username/workspace, verify GPIO support and permissions on the actual
Raspberry Pi, and confirm the full-stack CAN protocol is compatible with the
flashed STM32 firmware before enabling it. It waits for a node name, not for
complete hardware readiness; it is not a hardware-safety acceptance test.
