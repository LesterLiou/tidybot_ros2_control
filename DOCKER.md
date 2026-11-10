# Ubuntu 24.04 / ROS 2 Jazzy Docker environment

This image runs the `tidybot_ros2_control` package on Ubuntu 24.04 (Noble) with
ROS 2 Jazzy. It contains Phoenix 6, Ruckig, SocketCAN tools, the ROS gamepad
driver, rosbag2, OpenCV/ArUco, and the RealSense ROS driver.
It also includes Tesseract and an isolated UVC-camera battery dashboard; see
[BATTERY_MONITOR.md](BATTERY_MONITOR.md).

The Jazzy image uses the official ROS `cv_bridge` package and pins the Python
ArUco binding to `opencv-contrib-python-headless` 4.10. Ubuntu Noble's OpenCV
4.6 Python 3.12 ArUco binding crashed during the synthetic marker test; the
pinned binding is verified during every image build.

## What stays on the host

Docker shares the host kernel. Install the CAN adapter's kernel driver on the
host. A CANivore requires CTRE's `canivore-usb` kernel module and its matching
`/usr/bin/canivore_setup` and `/usr/bin/canivore_shutdown` tools. The image does
not load kernel modules.

Verify that the host driver created the interface:

```bash
ip -details link show can0
```

Do not run plain `ip link set can0 up` for a CANivore: the kernel will reject it
until CTRE's fixed CAN FD timing has been applied. After building the image, use
the short-lived `can-init` service instead:

```bash
docker compose run --rm can-init
```

Phoenix calls the host's matching CTRE helper from this service. Only this
short-lived service receives `NET_ADMIN`; the robot-control service does not.
For a generic CAN adapter, configure its bitrate on the host according to that
adapter's documentation and do not use `can-init`.

Plug in the gamepad and D415 before Compose starts so `/dev/input` and
`/dev/bus/usb` exist. The host device tree is mounted read-only at `/host-dev`;
the entrypoint provides V4L2/media symlinks that survive D415 renumbering after
a hardware reset. Device cgroups still permit opening only the input, V4L2,
USB, and media-controller classes rather than granting unrestricted device
access. The existing host udev rule remains responsible for gamepad permissions
and hot-plug behavior.

## Build and inspect

From this repository:

```bash
mkdir -p rosbags
docker compose build
docker compose run --rm can-init
docker compose run --rm tidybot tidybot-hardware-check
```

The initializer creates a Phoenix `CANBus` status object, not a motor object or
enable request. The hardware check imports ROS/Phoenix/Ruckig and shows the CAN
interface and USB devices. Neither command sends a motor control request.

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

Physical control necessarily grants the robot container access to CAN
transmission, USB hardware, gamepad input, and the mounted `rosbags/` directory.
It cannot administer the host network. The separate `can-init` service receives
`NET_ADMIN` only while applying the CANivore's CTRE-defined timing, then exits.
ROS target calibration persists in the named `tidybot_ros_home` volume.
Recorded bags stay under the ignored host `rosbags/` directory and are not
included in the image or Git commits.

The optional `battery-monitor` service has no CAN or gamepad access and no
Linux capabilities. Its SQLite history persists in `tidybot_ros_home`; camera
frames are not recorded. Start it independently with
`docker compose up -d battery-monitor`.

Run `can-init` again after unplugging/reconnecting the CANivore or rebooting the
host.

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
