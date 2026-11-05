"""ROS 2 gamepad teleoperation for TidyBot."""

from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger


def apply_deadzone(value, deadzone):
    """Remove stick drift while retaining the full output range."""
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)


class GamepadTeleopNode(Node):
    """Convert a sensor_msgs/Joy stream to guarded TidyBot velocity commands."""

    def __init__(self, **node_kwargs):
        super().__init__('tidybot_gamepad_teleop', **node_kwargs)

        joy_topic = self.declare_parameter('joy_topic', '/joy').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        enable_topic = self.declare_parameter('enable_topic', '/tidybot/enable').value
        frame_topic = self.declare_parameter('frame_topic', '/tidybot/control_frame').value
        stop_topic = self.declare_parameter('stop_topic', '/tidybot/stop').value

        self._axis_x = int(self.declare_parameter('axis_linear_x', 1).value)
        self._axis_y = int(self.declare_parameter('axis_linear_y', 0).value)
        self._axis_yaw = int(self.declare_parameter('axis_angular_z', 2).value)
        self._invert_x = bool(self.declare_parameter('invert_linear_x', False).value)
        self._invert_y = bool(self.declare_parameter('invert_linear_y', False).value)
        self._invert_yaw = bool(self.declare_parameter('invert_angular_z', False).value)
        self._button_arm = int(self.declare_parameter('button_arm', 4).value)
        self._button_disarm = int(self.declare_parameter('button_disarm', 10).value)
        self._button_local = int(self.declare_parameter('button_local_deadman', 6).value)
        self._button_global = int(self.declare_parameter('button_global_deadman', 7).value)
        self._button_speed_up = int(self.declare_parameter('button_speed_up', 3).value)
        self._button_speed_down = int(self.declare_parameter('button_speed_down', 1).value)
        self._button_dock = int(self.declare_parameter('button_dock', 0).value)
        self._button_rosbag = int(self.declare_parameter('button_rosbag', 11).value)
        self._button_shutdown = int(self.declare_parameter('button_shutdown', 12).value)
        self._docking_start_service = str(
            self.declare_parameter('docking_start_service', '/docking/start').value
        )
        self._rosbag_enabled = bool(
            self.declare_parameter('rosbag_recording_enabled', True).value
        )
        self._rosbag_output_directory = Path(str(
            self.declare_parameter(
                'rosbag_output_directory', '~/jerry/tidybot_ros2_control/rosbags'
            ).value
        )).expanduser()
        self._rosbag_topics = tuple(str(topic) for topic in self.declare_parameter(
            'rosbag_topics', [
                '/docking/debug_image/compressed',
                '/docking/status',
                '/docking/marker_pose',
                '/docking/cmd_vel',
                '/tidybot/control_status',
                '/cmd_vel',
                '/odom',
                '/joy',
                '/tf',
                '/tf_static',
            ]
        ).value)
        self._rosbag_min_free_gb = float(
            self.declare_parameter('rosbag_min_free_space_gb', 2.0).value
        )
        self._require_both_deadman = bool(
            self.declare_parameter('require_both_deadman', True).value
        )
        self._control_frame = self.declare_parameter('control_frame', 'local').value
        self._deadzone = float(self.declare_parameter('deadzone', 0.05).value)
        self._angular_deadzone = float(
            self.declare_parameter('angular_deadzone', 0.20).value
        )
        self._max_velocity = tuple(
            float(value)
            for value in self.declare_parameter(
                'max_velocity', [0.50, 0.50, 1.57]
            ).value
        )
        self._publish_rate = float(self.declare_parameter('publish_rate', 50.0).value)
        self._joy_timeout = float(self.declare_parameter('joy_timeout', 0.25).value)
        self._speed_scale = float(self.declare_parameter('initial_speed_scale', 0.50).value)
        self._speed_scale_step = float(self.declare_parameter('speed_scale_step', 0.10).value)
        self._min_speed_scale = float(self.declare_parameter('min_speed_scale', 0.20).value)
        self._max_speed_scale = float(self.declare_parameter('max_speed_scale', 1.00).value)

        if len(self._max_velocity) != 3:
            raise ValueError("Parameter 'max_velocity' must contain exactly three values")
        if not 0.0 <= self._deadzone < 1.0:
            raise ValueError("Parameter 'deadzone' must be in [0, 1)")
        if not 0.0 <= self._angular_deadzone < 1.0:
            raise ValueError("Parameter 'angular_deadzone' must be in [0, 1)")
        if self._publish_rate <= 0.0 or self._joy_timeout <= 0.0:
            raise ValueError("'publish_rate' and 'joy_timeout' must be greater than zero")
        if self._speed_scale_step <= 0.0:
            raise ValueError("Parameter 'speed_scale_step' must be greater than zero")
        if not self._min_speed_scale <= self._speed_scale <= self._max_speed_scale:
            raise ValueError(
                "Expected min_speed_scale <= initial_speed_scale <= max_speed_scale"
            )
        if self._control_frame not in ('local', 'global'):
            raise ValueError("Parameter 'control_frame' must be 'local' or 'global'")
        if min(self._button_dock, self._button_rosbag, self._button_shutdown) < 0:
            raise ValueError('Docking, rosbag and shutdown buttons must be non-negative')
        if self._button_dock in {
            self._button_arm,
            self._button_disarm,
            self._button_local,
            self._button_global,
            self._button_speed_up,
            self._button_speed_down,
        }:
            raise ValueError("Parameter 'button_dock' must not reuse another control button")
        if self._button_rosbag in {
            self._button_arm,
            self._button_disarm,
            self._button_local,
            self._button_global,
            self._button_speed_up,
            self._button_speed_down,
            self._button_dock,
        }:
            raise ValueError("Parameter 'button_rosbag' must not reuse another control button")
        if self._button_shutdown in {
            self._button_arm,
            self._button_disarm,
            self._button_local,
            self._button_global,
            self._button_speed_up,
            self._button_speed_down,
            self._button_dock,
            self._button_rosbag,
        }:
            raise ValueError("Parameter 'button_shutdown' must not reuse another control button")
        if not self._docking_start_service:
            raise ValueError("Parameter 'docking_start_service' must not be empty")
        if (not self._rosbag_topics or any(not topic.startswith('/') for topic in self._rosbag_topics)
                or not math.isfinite(self._rosbag_min_free_gb)
                or self._rosbag_min_free_gb < 0):
            raise ValueError('Invalid rosbag topics or free-space limit')

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._enable_publisher = self.create_publisher(Bool, enable_topic, state_qos)
        self._frame_publisher = self.create_publisher(String, frame_topic, state_qos)
        self._stop_publisher = self.create_publisher(Empty, stop_topic, 1)
        self._docking_start_client = self.create_client(
            Trigger, self._docking_start_service
        )
        self.create_subscription(Joy, joy_topic, self._joy_callback, qos_profile_sensor_data)

        self._armed = False
        self._axes = []
        self._buttons = []
        self._previous_buttons = []
        self._last_joy_time = None
        self._was_active = False
        self._last_frame = None
        self._docking_start_future = None
        self._rosbag_process = None
        self._rosbag_output = None
        self._rosbag_stopping = False
        self._rosbag_stop_stage = 0
        self._rosbag_stop_requested_at = None
        self._launch_shutdown_requested = False
        self._launch_shutdown_signal_sent = False
        self.create_timer(1.0 / self._publish_rate, self._publish_command)
        self.create_timer(0.25, self._poll_rosbag)
        self.create_timer(0.10, self._poll_launch_shutdown)
        self._publish_enable(False)

        self.get_logger().info(
            'Press Y to arm; hold LB and RB together to move; release them and press A to dock; '
            'press Menu to start/stop rosbag recording; press Xbox/Guide to close the launch.'
        )
        self._log_speed('Initial')

    @staticmethod
    def _item(values, index):
        return values[index] if 0 <= index < len(values) else 0

    def _pressed(self, button):
        return bool(self._item(self._buttons, button))

    def _rising_edge(self, button):
        return self._pressed(button) and not bool(self._item(self._previous_buttons, button))

    def _joy_callback(self, msg):
        self._previous_buttons = self._buttons
        self._buttons = list(msg.buttons)
        self._axes = list(msg.axes)
        self._last_joy_time = time.monotonic()

        if self._rising_edge(self._button_shutdown):
            self._request_launch_shutdown()
            return
        if self._launch_shutdown_requested:
            return

        if self._rising_edge(self._button_disarm):
            self._armed = False
            self._stop_publisher.publish(Empty())
            self._deactivate('Control disarmed with Back.')
        elif self._rising_edge(self._button_arm):
            self._armed = True
            self.get_logger().info('Control armed. Hold LB and RB together to enable motion.')

        if self._rising_edge(self._button_speed_up):
            self._set_speed_scale(self._speed_scale + self._speed_scale_step, 'Increased')
        elif self._rising_edge(self._button_speed_down):
            self._set_speed_scale(self._speed_scale - self._speed_scale_step, 'Decreased')

        if self._rising_edge(self._button_dock):
            self._request_docking()
        if self._rising_edge(self._button_rosbag):
            self._toggle_rosbag()

    def _request_docking(self):
        if not self._armed:
            self.get_logger().warning('Docking not started: press Y to arm first.')
            return
        if self._pressed(self._button_local) or self._pressed(self._button_global):
            self.get_logger().warning(
                'Docking not started: release LB and RB before pressing A.'
            )
            return
        if (self._docking_start_future is not None
                and not self._docking_start_future.done()):
            self.get_logger().warning('Docking start request is already in progress.')
            return
        if not self._docking_start_client.service_is_ready():
            self.get_logger().warning(
                f'Docking not started: service {self._docking_start_service} is unavailable. '
                'Run tidybot_docking.launch.py and press A again.'
            )
            return
        self.get_logger().info('A pressed: requesting return to the docking station.')
        self._docking_start_future = self._docking_start_client.call_async(
            Trigger.Request()
        )
        self._docking_start_future.add_done_callback(self._docking_start_done)

    def _docking_start_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # rclpy reports service transport failures here.
            self.get_logger().error(f'Docking start service failed: {exc}')
            return
        if response.success:
            self.get_logger().info(f'Docking start accepted: {response.message}')
        else:
            self.get_logger().warning(f'Docking start rejected: {response.message}')

    def _toggle_rosbag(self):
        if not self._rosbag_enabled:
            self.get_logger().warning('Rosbag recording is disabled by configuration.')
            return
        if self._rosbag_process is not None:
            if self._rosbag_process.poll() is None:
                if self._rosbag_stopping:
                    self.get_logger().warning('Rosbag is already stopping; please wait.')
                else:
                    self._request_rosbag_stop('Menu pressed')
                return
            self._finish_rosbag(self._rosbag_process.returncode)
        self._start_rosbag()

    def _start_rosbag(self):
        ros2 = shutil.which('ros2')
        if ros2 is None:
            self.get_logger().error('Cannot start rosbag: ros2 executable was not found.')
            return
        try:
            self._rosbag_output_directory.mkdir(parents=True, exist_ok=True)
            free_gb = shutil.disk_usage(self._rosbag_output_directory).free / 1e9
        except OSError as exc:
            self.get_logger().error(f'Cannot prepare rosbag directory: {exc}')
            return
        if free_gb < self._rosbag_min_free_gb:
            self.get_logger().error(
                f'Cannot start rosbag: only {free_gb:.2f} GB free; '
                f'{self._rosbag_min_free_gb:.2f} GB required.'
            )
            return
        name = datetime.now().strftime('docking_%Y%m%d_%H%M%S_%f')
        output = self._rosbag_output_directory / name
        command = [
            ros2, 'bag', 'record',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--output', str(output),
            *self._rosbag_topics,
        ]
        try:
            self._rosbag_process = subprocess.Popen(command, start_new_session=True)
        except OSError as exc:
            self.get_logger().error(f'Could not start rosbag recording: {exc}')
            self._rosbag_process = None
            return
        self._rosbag_output = output
        self._rosbag_stopping = False
        self._rosbag_stop_stage = 0
        self._rosbag_stop_requested_at = None
        self.get_logger().info(
            f'ROSBAG RECORDING STARTED: {output} '
            f'(boxed ArUco image: /docking/debug_image/compressed)'
        )

    def _request_rosbag_stop(self, reason):
        process = self._rosbag_process
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        except OSError as exc:
            self.get_logger().error(f'Could not stop rosbag cleanly: {exc}')
            return
        self._rosbag_stopping = True
        self._rosbag_stop_stage = 1
        self._rosbag_stop_requested_at = time.monotonic()
        self.get_logger().info(f'ROSBAG STOP REQUESTED: {reason}; writing metadata...')

    def _poll_rosbag(self):
        process = self._rosbag_process
        if process is None:
            return
        returncode = process.poll()
        if returncode is not None:
            self._finish_rosbag(returncode)
            return
        if not self._rosbag_stopping:
            try:
                free_gb = shutil.disk_usage(self._rosbag_output_directory).free / 1e9
            except OSError:
                free_gb = math.inf
            if free_gb < self._rosbag_min_free_gb:
                self.get_logger().error(
                    f'Low disk space ({free_gb:.2f} GB); stopping rosbag safely.'
                )
                self._request_rosbag_stop('low disk space')
            return
        elapsed = time.monotonic() - self._rosbag_stop_requested_at
        if self._rosbag_stop_stage == 1 and elapsed > 5.0:
            self.get_logger().warning('Rosbag did not stop after SIGINT; sending SIGTERM.')
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            self._rosbag_stop_stage = 2
            self._rosbag_stop_requested_at = time.monotonic()
        elif self._rosbag_stop_stage == 2 and elapsed > 2.0:
            self.get_logger().error('Rosbag did not stop after SIGTERM; sending SIGKILL.')
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            self._rosbag_stop_stage = 3

    def _finish_rosbag(self, returncode):
        output = self._rosbag_output
        if returncode == 0:
            self.get_logger().info(f'ROSBAG SAVED: {output}')
        else:
            self.get_logger().warning(
                f'Rosbag process exited with code {returncode}; inspect {output} before use.'
            )
        self._rosbag_process = None
        self._rosbag_output = None
        self._rosbag_stopping = False
        self._rosbag_stop_stage = 0
        self._rosbag_stop_requested_at = None

    def _request_launch_shutdown(self):
        if self._launch_shutdown_requested:
            return
        self._launch_shutdown_requested = True
        self._armed = False
        self._stop_publisher.publish(Empty())
        self._deactivate('Launch shutdown requested from the Xbox/Guide button.')
        self._publish_zero()
        if (self._rosbag_process is not None
                and self._rosbag_process.poll() is None
                and not self._rosbag_stopping):
            self._request_rosbag_stop('launch shutdown requested')
        self.get_logger().warning(
            'GAMEPAD SHUTDOWN REQUESTED: stopping the robot and closing the entire launch.'
        )

    def _poll_launch_shutdown(self):
        if not self._launch_shutdown_requested or self._launch_shutdown_signal_sent:
            return
        self._launch_shutdown_signal_sent = True
        self.get_logger().warning('Exiting gamepad node now; launch shutdown will follow.')
        # main() handles SIGINT as a normal shutdown, publishes zero again,
        # finishes any active rosbag, and exits. The launch files use this
        # node's on_exit event to stop every other process in the launch.
        os.kill(os.getpid(), signal.SIGINT)

    def _stop_rosbag_blocking(self):
        process = self._rosbag_process
        if process is None:
            return
        if process.poll() is None:
            self._request_rosbag_stop('gamepad node shutting down')
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait()
        self._finish_rosbag(process.returncode)

    def _log_speed(self, prefix):
        effective = [limit * self._speed_scale for limit in self._max_velocity]
        self.get_logger().info(
            f'{prefix} speed to {100.0 * self._speed_scale:.0f}% '
            f'(x/y={effective[0]:.2f} m/s, yaw={effective[2]:.2f} rad/s).'
        )

    def _set_speed_scale(self, requested, prefix):
        new_scale = max(self._min_speed_scale, min(self._max_speed_scale, requested))
        self._speed_scale = round(new_scale, 10)
        self._log_speed(prefix)

    def _publish_enable(self, enabled):
        msg = Bool()
        msg.data = enabled
        self._enable_publisher.publish(msg)

    def _publish_frame(self, frame):
        if frame == self._last_frame:
            return
        msg = String()
        msg.data = frame
        self._frame_publisher.publish(msg)
        self._last_frame = frame

    def _publish_zero(self):
        self._cmd_publisher.publish(Twist())

    def _deactivate(self, message=None):
        if self._was_active:
            self._publish_zero()
            self._publish_enable(False)
            self._was_active = False
            if message:
                self.get_logger().info(message)
        else:
            self._publish_enable(False)

    def _publish_command(self):
        if self._launch_shutdown_requested:
            self._deactivate()
            return
        joy_fresh = (
            self._last_joy_time is not None
            and time.monotonic() - self._last_joy_time <= self._joy_timeout
        )
        local_deadman = self._pressed(self._button_local)
        global_deadman = self._pressed(self._button_global)
        if self._require_both_deadman:
            deadman_active = local_deadman and global_deadman
            frame = self._control_frame
        else:
            deadman_active = local_deadman or global_deadman
            frame = 'local' if local_deadman else 'global'
        active = self._armed and joy_fresh and deadman_active

        if not active:
            reason = 'Gamepad stream timed out; base disabled.' if self._armed and not joy_fresh else None
            self._deactivate(reason)
            return

        if not self._was_active:
            self.get_logger().info(f'Robot enabled in {frame} frame.')
        self._publish_frame(frame)
        self._publish_enable(True)
        self._was_active = True

        raw_axes = [
            float(self._item(self._axes, self._axis_x)),
            float(self._item(self._axes, self._axis_y)),
            float(self._item(self._axes, self._axis_yaw)),
        ]
        inverted = [self._invert_x, self._invert_y, self._invert_yaw]
        normalized = [
            -value if should_invert else value
            for value, should_invert in zip(raw_axes, inverted)
        ]
        normalized = [max(-1.0, min(1.0, value)) for value in normalized]
        deadzones = [self._deadzone, self._deadzone, self._angular_deadzone]
        normalized = [
            apply_deadzone(value, deadzone)
            for value, deadzone in zip(normalized, deadzones)
        ]
        target = [
            limit * self._speed_scale * value
            for limit, value in zip(self._max_velocity, normalized)
        ]

        msg = Twist()
        msg.linear.x = float(target[0])
        msg.linear.y = float(target[1])
        msg.angular.z = float(target[2])
        self._cmd_publisher.publish(msg)

    def stop(self):
        self._armed = False
        self._deactivate()
        self._publish_zero()
        self._stop_rosbag_blocking()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = GamepadTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
