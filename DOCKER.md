# Ubuntu 24.04 / ROS 2 Jazzy Docker environment

This image runs the `tidybot_ros2_control` package on Ubuntu 24.04 (Noble) with
ROS 2 Jazzy. It contains Phoenix 6, Ruckig, SocketCAN tools, the ROS gamepad
driver, rosbag2, OpenCV/ArUco, and the RealSense ROS driver.

The Jazzy image uses the official ROS `cv_bridge` package and pins the Python
ArUco binding to `opencv-contrib-python-headless` 4.10. Ubuntu Noble's OpenCV
4.6 Python 3.12 ArUco binding crashed during the synthetic marker test; the
pinned binding is verified during every image build.

## What stays on the host

Docker shares the host kernel. Install and configure the CAN adapter's kernel
driver on the host, then bring up `can0` before starting the container. A
CANivore requires CTRE's `canivore-usb` host kernel module. The image does not
load modules or change host network configuration.

Verify the host interface:

```bash
ip -details link show can0
```

For a generic CAN adapter that requires a bitrate, configure it on the host
according to the adapter's documentation. Do not apply a generic bitrate command
to CANivore: its firmware manages the bus settings.

Plug in the gamepad and USB devices before Compose starts so `/dev/input` and
`/dev/bus/usb` exist. The existing host udev rule remains responsible for
gamepad permissions and hot-plug behavior.

## Build and inspect

From this repository:

```bash
mkdir -p rosbags
docker compose build
docker compose run --rm tidybot tidybot-hardware-check
```

The hardware check imports ROS/Phoenix/Ruckig, shows the CAN interface and USB
devices, and sends no motor command.

The image build runs all synthetic tests. To repeat them:

```bash
docker compose run --rm tidybot python3 -m pytest -q \
  src/tidybot_ros2_control/tests
```

Rebuild the image after editing source files:

```bash
docker compose build
```

## Start the robot control environment

Open a prepared shell:

```bash
docker compose run --rm tidybot bash
```

The entrypoint already sources ROS 2 Jazzy and the built workspace. Start the
gamepad/base stack only after the host CAN check passes:

```bash
ros2 launch tidybot_ros2_control tidybot_gamepad.launch.py
```

For docking preview without starting the base:

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py \
  dry_run:=true start_base:=false show_image:=false
```

To enable the base for docking, use the project's guarded launch arguments:

```bash
ros2 launch tidybot_ros2_control tidybot_docking.launch.py \
  dry_run:=false start_base:=true show_image:=false
```

Use `show_image:=true` only when the host has an X11 session, `DISPLAY` is set,
and that session permits the local container connection.

Stop a running Compose container with `docker compose stop`. Compose sends
SIGINT and waits 20 seconds so the base node can command neutral and rosbag2 can
finish its metadata.

## Isolation and persistent data

The service does not use `privileged`, host PID/IPC namespaces, an unconfined
seccomp profile, or a full `/dev` mount. Its root filesystem is read-only, and it
has only `NET_RAW` (SocketCAN) and `SYS_NICE` (the 250 Hz real-time thread).

Physical control necessarily grants the container access to CAN transmission,
USB hardware, gamepad input, and the mounted `rosbags/` directory. ROS target
calibration persists in the named `tidybot_ros_home` volume. Recorded bags stay
under the ignored host `rosbags/` directory and are not included in the image or
Git commits.

If the bus name differs, set it for both Compose and the controller:

```bash
TIDYBOT_CAN_INTERFACE=can1 docker compose run --rm tidybot \
  tidybot-hardware-check
```

## Ubuntu 24.04 compatibility boundary

The Docker image covers the ROS 2 base, gamepad, rosbag, D415/ArUco docking, and
SocketCAN/Phoenix motor-control path. The legacy simulation, learning, web app,
and Kinova arm scripts use the separate `requirements.txt` plus external Kinova
SDK components; they are not installed or claimed as validated by this control
image.
