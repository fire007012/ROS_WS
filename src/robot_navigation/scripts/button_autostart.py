#!/usr/bin/env python3
"""Boot-time hardware-button launcher for the complete robot stack."""

import argparse
import os
import signal
import subprocess
import sys
import time

DEFAULT_TOKEN = 0x5A17
DEFAULT_LAUNCH_ARGS = ("enable_physical_start:=false", "enable_can_start_signal:=false")


class SysfsGpio:
    def __init__(self, pin):
        self.pin = int(pin)
        self.path = "/sys/class/gpio/gpio{}".format(self.pin)
        self.exported_here = False

    def export(self, direction):
        if not os.path.isdir(self.path):
            try:
                with open("/sys/class/gpio/export", "w") as exported:
                    exported.write(str(self.pin))
                self.exported_here = True
            except OSError as exc:
                raise RuntimeError("cannot export GPIO {}: {}".format(self.pin, exc))
            deadline = time.monotonic() + 1.0
            while not os.path.isdir(self.path) and time.monotonic() < deadline:
                time.sleep(0.01)
        try:
            with open(os.path.join(self.path, "direction"), "w") as direction_file:
                direction_file.write(direction)
        except OSError as exc:
            raise RuntimeError("cannot configure GPIO {}: {}".format(self.pin, exc))

    def read(self):
        try:
            with open(os.path.join(self.path, "value"), "r") as value_file:
                return 1 if value_file.read(1) == "1" else 0
        except OSError:
            return -1

    def write(self, value):
        try:
            with open(os.path.join(self.path, "value"), "w") as value_file:
                value_file.write("1" if value else "0")
            return True
        except OSError:
            return False

    def close(self):
        if self.exported_here:
            try:
                with open("/sys/class/gpio/unexport", "w") as unexported:
                    unexported.write(str(self.pin))
            except OSError:
                pass


def confirmed_press(button, active_low, debounce_s, poll_s):
    active = lambda: button.read() == (0 if active_low else 1)
    while True:
        if active():
            started = time.monotonic()
            while active() and time.monotonic() - started < debounce_s:
                time.sleep(poll_s)
            if active() and time.monotonic() - started >= debounce_s:
                return
        time.sleep(poll_s)


def wait_for_node(node_name, timeout_s, env):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(["rosnode", "list"], env=env, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, universal_newlines=True, timeout=3)
            if node_name in result.stdout.splitlines():
                return True
        except (OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(0.5)
    return False


def publish_start(token, env):
    command = ["rostopic", "pub", "-1", "/start_signal/physical", "std_msgs/UInt32",
               "{data: %d}" % token]
    try:
        return subprocess.run(command, env=env, timeout=10).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def terminate_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10)
    except (OSError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser(description="Start the complete ROS stack from a GPIO button")
    parser.add_argument("--button-pin", type=int, default=17)
    parser.add_argument("--led-pin", type=int, default=27)
    parser.add_argument("--active-low", action="store_true", default=True)
    parser.add_argument("--active-high", action="store_false", dest="active_low")
    parser.add_argument("--debounce-ms", type=int, default=50)
    parser.add_argument("--poll-ms", type=int, default=20)
    parser.add_argument("--start-token", type=lambda value: int(value, 0), default=DEFAULT_TOKEN)
    parser.add_argument("--ready-timeout-s", type=float, default=120.0)
    parser.add_argument("--launch-package", default="robot_navigation")
    parser.add_argument("--launch-file", default="mission_complete.launch")
    parser.add_argument("launch_args", nargs="*", help="extra roslaunch arguments")
    args = parser.parse_args()

    env = os.environ.copy()
    button = SysfsGpio(args.button_pin)
    led = SysfsGpio(args.led_pin)
    launch_process = None
    try:
        button.export("in")
        led.export("out")
        led.write(1)
        print("[button_autostart] waiting for GPIO {}".format(args.button_pin), flush=True)
        while True:
            confirmed_press(button, args.active_low, max(0.001, args.debounce_ms / 1000.0),
                            max(0.001, args.poll_ms / 1000.0))
            led.write(0)
            command = ["roslaunch", args.launch_package, args.launch_file]
            command.extend(DEFAULT_LAUNCH_ARGS)
            command.extend(args.launch_args)
            print("[button_autostart] button pressed; starting complete stack", flush=True)
            launch_process = subprocess.Popen(command, env=env, start_new_session=True)
            if wait_for_node("/mission_controller_node", args.ready_timeout_s, env):
                if publish_start(args.start_token, env):
                    print("[button_autostart] stack ready; start pulse sent", flush=True)
                else:
                    print("[button_autostart] failed to send start pulse", file=sys.stderr, flush=True)
            else:
                print("[button_autostart] mission controller did not become ready", file=sys.stderr, flush=True)
                terminate_process(launch_process)
            # Keep the button daemon attached to the running stack so a second
            # press cannot create a duplicate ROS graph.
            if launch_process is not None:
                launch_process.wait()
            launch_process = None
            # Consume the release before allowing a new startup cycle.
            while button.read() == (0 if args.active_low else 1):
                time.sleep(max(0.001, args.poll_ms / 1000.0))
            led.write(1)
    except (RuntimeError, KeyboardInterrupt) as exc:
        if not isinstance(exc, KeyboardInterrupt):
            print("[button_autostart] {}".format(exc), file=sys.stderr)
            return 1
    finally:
        terminate_process(launch_process)
        led.close()
        button.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
