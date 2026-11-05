# TidyBot ROS 2 gamepad control

For D415 + ArUco teaching and docking, see [DOCKING.md](DOCKING.md).

This package uses the normal ROS 2 control flow:

```text
/dev/input/js0 -> joy_node -> /joy -> gamepad_teleop -> /cmd_vel -> base_node -> Phoenix 6/CAN
                                           |               |
                                           +-> /tidybot/enable
                                           +-> /tidybot/control_frame
                                                           +-> /odom
```

The examples below target ROS 2 Humble. Keep the robot's casters off the ground
for the first test and keep a physical emergency stop within reach.

## Install and build

Install the ROS-side dependencies in the ROS 2 environment/container:

```bash
sudo apt update
sudo apt install python3-pip ros-humble-joy ros-humble-rclpy ros-humble-nav-msgs
/usr/bin/python3 -m pip install -r requirements-ros2.txt
```

ROS 2 Humble on Ubuntu 22.04 uses Python 3.10. Confirm that the Python used by
`colcon` can import every runtime dependency:

```bash
/usr/bin/python3 --version
/usr/bin/python3 -c "import rclpy, numpy, phoenix6, ruckig"
```

Do not build or run this package from a Python 3.11+ Conda environment: Humble's
installed `rclpy` extension is compiled for Python 3.10. If you use the original
`tidybot2` Mamba environment, keep it at Python 3.10.14 and ensure the generated
scripts and dependencies use that same interpreter.

Build from the directory containing this repository:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tidybot_ros2_control
source install/setup.bash
```

The process also needs access to `/dev/input` and the configured SocketCAN
interface (`can0` by default). The supplied compose file already exposes `/dev`
and uses the host network.

## Verify before enabling motors

Check that Linux and ROS can see the gamepad:

```bash
ls -l /dev/input/js0
ros2 run joy joy_node --ros-args -p device_id:=0 -p autorepeat_rate:=50.0
ros2 topic echo /joy
```

In another terminal, verify the CAN interface:

```bash
ip -details link show can0
```

### Linux sees js0, but joy_node cannot see the controller

`joy_node` uses SDL and needs access to the controller's `/dev/input/eventN`.
Having a readable `/dev/input/js0` alone is insufficient. Check enumeration:

```bash
ros2 run joy joy_enumerate_devices
```

For this machine's Bluetooth Xbox Wireless Controller (`045e:0b13`), the event
device was accessible to `gdm` but not `hrc`, so enumeration was empty and `/joy`
was never published. `hrc` already belongs to `plugdev`. Install the narrow udev
rule below to give that group access to this joystick model, including after
reconnection:

```bash
bash /home/hrc/jerry/tidybot_ros2_control/scripts/setup_gamepad_permissions.sh
ros2 run joy joy_enumerate_devices
```

The script needs the local sudo password, refreshes only matching Xbox event
devices, and leaves the button/axis configuration unchanged. If `joy_node` is
already running, restart the gamepad launch after applying the rule. Other
controller models need their own measured IDs and mappings.

## Run

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tidybot_ros2_control tidybot_gamepad.launch.py
```

Controls use the measured raw mapping of the connected Xbox Series X controller
through `joy_node` (Y=4, LB=6, RB=7, Back/View=10):

- **Y**: arm ROS teleoperation (does not move the robot).
- **Back**: disarm and command zero velocity.
- **X**: increase the speed scale by 10%.
- **B**: decrease the speed scale by 10%.
- **A**: request visual docking when `tidybot_docking.launch.py` is running.
- **Menu**（Xbox 中央右側、三條線，`buttons[11]`）：按一次開始 rosbag，再按一次結束並儲存。
- **Xbox／Guide**（中央圓形 Xbox 標誌，`buttons[12]`）：停止車體並關閉整個 launch。
- Hold **LB + RB together**: enable motion in the robot-local frame.
- Left stick: forward/back and strafe.
- Right stick left/right: rotate counter-clockwise/clockwise.
- Releasing either LB or RB commands zero velocity and disables motion.

The standalone `tidybot_gamepad.launch.py` does not start the docking camera or
controller, so pressing A there only reports that `/docking/start` is unavailable.
Menu 仍可啟動 rosbag，但只有 `tidybot_docking.launch.py` 會提供帶 ArUco 框的影像 topic。

The initial speed remains conservative at 50%: 0.25 m/s in X/Y and about
0.79 rad/s in yaw. X/B adjust this between 20% and 100%; the configured ceiling
is 0.50 m/s in X/Y and 1.57 rad/s in yaw. Keep both `max_velocity` entries in
`config/gamepad.yaml` identical so the teleop and base limits agree.

## Useful checks

```bash
ros2 topic hz /joy
ros2 topic echo /cmd_vel
ros2 topic echo /tidybot/enable
ros2 topic echo /odom
```

If the button or axis numbers differ, run `ros2 topic echo /joy`, operate one
control at a time, and update the axis and `invert_*` values in
`config/gamepad.yaml`. The current settings expect forward as `axes[1] = +1.0`,
right rotation as `axes[2] = -1.0`, and left rotation as `axes[2] = +1.0`. If the
controller disconnects or `/cmd_vel` stops for 0.30 seconds, the software commands
zero velocity. This is a software safety layer, not a replacement for a physical
emergency stop.

The yaw deadzone is set separately to `0.20` because this controller showed about
`0.15` of horizontal crosstalk while its right stick was moved vertically.
