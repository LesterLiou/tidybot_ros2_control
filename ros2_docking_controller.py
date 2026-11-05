"""Teach and approach a visual target, requesting exclusive control from the mux."""

import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from docking_core import (DockingControl, DockingSettings, average_transforms, load_target,
                          save_target, transform)


class DockingControllerNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('tidybot_docking_controller', **node_kwargs)
        p = lambda name, default: self.declare_parameter(name, default).value
        self.base_frame = p('base_frame', 'base_link')
        self.target_file = p('target_file', '~/.ros/tidybot/dock_target.yaml')
        teach_on_start = bool(p('teach_on_start', False))
        self.rate = float(p('control_rate', 30.0))
        self.teach_duration = float(p('teach_duration', 1.0))
        self.teach_min_samples = int(p('teach_min_samples', 15))
        self.teach_translation_spread = float(p('teach_translation_spread', 0.005))
        self.teach_rotation_spread = float(p('teach_rotation_spread', 0.035))
        if (not np.all(np.isfinite([self.rate, self.teach_duration, self.teach_translation_spread,
                                   self.teach_rotation_spread]))
                or min(self.rate, self.teach_duration, self.teach_translation_spread,
                       self.teach_rotation_spread) <= 0 or self.teach_min_samples < 3):
            raise ValueError('Invalid control rate or teaching settings')
        defaults = DockingSettings()
        self.core = DockingControl(DockingSettings(**{
            name: float(p(name, value)) for name, value in vars(defaults).items()}))
        settings = self.core.settings
        self.get_logger().info(
            f'Controller loaded from {Path(__file__).resolve()}; '
            f'progress_timeout={settings.progress_timeout:.1f} s, '
            f'phase_distances=[fine {settings.fine_align_distance:.3f} m, '
            f'insertion {settings.insertion_distance:.3f} m, '
            f'extra {settings.extra_insert_distance:.3f} m], '
            f'target_right_offset={settings.target_right_offset:.3f} m, '
            f'fine_alignment=[{settings.align_y:.3f} m, '
            f'{math.degrees(settings.align_yaw):.2f} deg], '
            f'max_velocity=[{settings.max_forward:.3f} m/s, '
            f'{settings.max_lateral:.3f} m/s, {settings.max_yaw_rate:.3f} rad/s], '
            f'minimum_motion=[forward {settings.min_forward:.3f} m/s, '
            f'lateral {settings.min_lateral:.3f} m/s]')
        self.detector_context = None
        self.calibrated = False
        self.goal = None
        self.goal_context = None
        self.last_stamp = -1
        self.granted = False
        self.pending = None
        self.pending_ack = False
        self.pending_cancelled = False
        self.teaching = None
        # Opt-in first setup: the user places the robot at the intended stop pose.
        # Never replace an existing file just because the launch was restarted.
        self.startup_teach_pending = teach_on_start and not Path(self.target_file).expanduser().exists()
        self.last_status = None
        self._last_feedback_time = -np.inf
        try:
            self.goal, self.goal_context = load_target(self.target_file, self.base_frame)
            self.get_logger().info(f'Loaded target from {self.target_file}')
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().info(f'No usable target yet; teach one before starting: {exc}')
        if teach_on_start and not self.startup_teach_pending:
            self.get_logger().warn('Startup teaching skipped: target file already exists; it will not be overwritten.')
        elif self.startup_teach_pending:
            self.get_logger().info('Startup teaching requested: keep the robot at the intended stop pose; waiting for a valid marker.')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/docking/cmd_vel', 1)
        self.active_pub = self.create_publisher(Bool, '/docking/active', 1)
        self.status_pub = self.create_publisher(String, '/docking/status', qos)
        self.control_client = self.create_client(SetBool, '/docking/set_control')
        self.create_subscription(String, '/docking/detector_config', self._detector_config, qos)
        self.create_subscription(PoseStamped, '/docking/marker_pose', self._pose, 1)
        self.create_subscription(Bool, '/docking/control_granted', self._grant, qos)
        self.create_service(Trigger, '/docking/teach', self._teach)
        self.create_service(Trigger, '/docking/start', self._start)
        self.create_service(Trigger, '/docking/cancel', self._cancel)
        self.create_timer(1.0/self.rate, self._tick)
        self._publish(np.zeros(3))

    def _detector_config(self, msg):
        try:
            data = json.loads(msg.data)
            calibrated = (data['calibrated'] is True
                          and data['settings']['base_frame'] == self.base_frame)
            if not isinstance(data['fingerprint'], str):
                raise ValueError('Missing fingerprint')
        except (ValueError, KeyError, TypeError):
            self._finish('FAILED', 'Invalid detector configuration')
            self.detector_context, self.calibrated = None, False
            return
        if self.detector_context is not None and self.detector_context != msg.data:
            self._finish('FAILED', 'Detector configuration changed; teach again')
            self.core.observation, self.core.observed_at = None, None
        self.detector_context, self.calibrated = msg.data, calibrated

    def _pose(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if (msg.header.frame_id != self.base_frame or stamp <= self.last_stamp
                or age < -0.05 or age > self.core.settings.pose_timeout):
            return
        self.last_stamp = stamp
        pos, q = msg.pose.position, msg.pose.orientation
        try:
            matrix = transform([pos.x, pos.y, pos.z], [q.x, q.y, q.z, q.w])
            if not self.core.observe(matrix, time.monotonic()-max(age, 0.0)):
                self._finish(self.core.state, self.core.reason)
                return
            if self.teaching is not None:
                self.teaching['samples'].append(matrix)
        except ValueError as exc:
            if self.core.active:
                self._finish('FAILED', f'Invalid marker pose: {exc}')

    def _grant(self, msg):
        previously_granted = self.granted
        self.granted = bool(msg.data)
        if not self.granted and (self.core.active or (previously_granted and self.pending is not None)):
            self._finish('CANCELLED', 'Control revoked by manual takeover, stop or watchdog')

    def _ready(self, allow_reacquisition=False):
        if not self.calibrated or self.detector_context is None:
            raise ValueError('Confirm camera_xyz/camera_rpy, set camera_mount_calibrated true in the config, and restart')
        freshness_limit = (self.core.settings.pose_timeout if allow_reacquisition
                           else self.core.settings.pose_command_timeout)
        if (self.core.observed_at is None
                or time.monotonic()-self.core.observed_at > freshness_limit):
            raise ValueError('No fresh marker pose in the base frame')

    def _teach(self, request, response):
        self.startup_teach_pending = False
        try:
            self._ready()
            if self.core.active or self.pending is not None or self.teaching is not None:
                raise ValueError('Cancel the current operation before teaching')
            self.teaching = {'started': time.monotonic(), 'context': self.detector_context, 'samples': []}
            self.core.stop('TEACHING', 'Keep the robot stationary; collecting fresh observations')
            response.success, response.message = True, 'Teaching started; watch /docking/status for completion'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _start(self, request, response):
        self.startup_teach_pending = False
        try:
            self._ready(allow_reacquisition=True)
            if self.core.active or self.pending is not None or self.teaching is not None:
                raise ValueError('An operation is already running')
            if self.goal is None or self.goal_context != self.detector_context:
                raise ValueError('No target for this detector/mount configuration; teach again')
            if not self.control_client.service_is_ready():
                raise ValueError('Control mux is unavailable')
            self.core.start(self.goal, time.monotonic())  # Validate before asking for control.
            self.core.stop('ACQUIRING', 'Waiting for exclusive control')
            self.pending_cancelled = False
            self.pending_ack = False
            self.pending_since = time.monotonic()
            self.pending = self.control_client.call_async(SetBool.Request(data=True))
            self.pending.add_done_callback(self._acquired)
            response.success, response.message = True, 'Start requested; watch /docking/status for result'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _acquired(self, future):
        try:
            result = future.result()
            if self.pending_cancelled:
                self._release()
                self.pending = None
            elif not result.success:
                self.pending = None
                self._finish('FAILED', result.message)
            else:
                self.pending_ack = True
        except Exception as exc:
            self.pending = None
            self._finish('FAILED', f'Could not acquire control: {exc}')

    def _release(self):
        if self.control_client.service_is_ready():
            self.control_client.call_async(SetBool.Request(data=False))

    def _finish(self, state, reason):
        self.core.stop(state, reason)
        self.teaching = None
        self.startup_teach_pending = False
        self.pending_cancelled = True
        if self.pending is not None and self.pending.done():
            self.pending = None
        self._publish(np.zeros(3))
        self._release()

    def _cancel(self, request, response):
        self._finish('CANCELLED', 'Cancelled explicitly')
        response.success, response.message = True, 'Docking cancelled; a new start command is required'
        return response

    def _update_readiness(self):
        # Readiness never starts docking or clears an operation's final result.
        if (self.core.active or self.pending is not None or self.teaching is not None
                or self.core.state not in {
                    'IDLE', 'WAITING_FOR_DETECTOR', 'NEEDS_CAMERA_SETUP',
                    'WAITING_FOR_MARKER', 'NEEDS_TEACH', 'POSE_MISMATCH',
                    'WAITING_FOR_CONTROL', 'READY'}):
            return
        if self.detector_context is None:
            state, reason = 'WAITING_FOR_DETECTOR', 'Waiting for the detector configuration'
        elif not self.calibrated:
            state, reason = 'NEEDS_CAMERA_SETUP', 'Confirm the fixed camera mounting in the configuration'
        elif (self.core.observed_at is None
              or time.monotonic()-self.core.observed_at
              > self.core.settings.pose_command_timeout):
            state, reason = 'WAITING_FOR_MARKER', 'Waiting for a fresh marker pose; check the image, marker, CameraInfo and TF'
        elif self.goal is None:
            if self.startup_teach_pending:
                response = self._teach(None, Trigger.Response())
                if not response.success:
                    self.core.stop('FAILED', response.message)
                return
            state, reason = 'NEEDS_TEACH', 'No saved target; place the robot at the stop pose and teach once'
        elif self.goal_context != self.detector_context:
            state, reason = 'NEEDS_TEACH', 'Saved target does not match the camera/marker configuration; teach again'
        else:
            try:
                self.core.check_planar_target(self.goal)
            except ValueError as exc:
                state, reason = 'POSE_MISMATCH', str(exc)
            else:
                if not self.control_client.service_is_ready():
                    state, reason = 'WAITING_FOR_CONTROL', 'Waiting for the control mux'
                else:
                    state, reason = 'READY', 'Camera mounting, marker and saved target ready; waiting for /docking/start'
        if (state, reason) != (self.core.state, self.core.reason):
            self.core.stop(state, reason)

    def _tick(self):
        now = time.monotonic()
        self._update_readiness()
        if self.pending is not None:
            if not self.pending_cancelled and now-self.pending_since > 1.0:
                self._finish('FAILED', 'Control acquisition timed out')
            elif not self.pending_cancelled and self.pending_ack and self.granted:
                self.pending = None
                try:
                    self._ready(allow_reacquisition=True)
                    if self.goal_context != self.detector_context:
                        raise ValueError('Detector configuration changed')
                    self.core.start(self.goal, now)
                except ValueError as exc:
                    self._finish('FAILED', str(exc))
        if self.teaching is not None and now-self.teaching['started'] >= self.teach_duration:
            try:
                self._ready()
                if len(self.teaching['samples']) < self.teach_min_samples:
                    raise ValueError('Too few fresh teaching samples')
                target = average_transforms(self.teaching['samples'], self.teach_translation_spread,
                                            self.teach_rotation_spread)
                save_target(self.target_file, target, self.teaching['context'], self.base_frame)
                self.goal, self.goal_context = target, self.teaching['context']
                self.core.stop('TAUGHT', f'Target saved to {self.target_file}')
            except (ValueError, OSError) as exc:
                self.core.stop('FAILED', f'Teaching failed: {exc}')
            self.teaching = None
        was_active = self.core.active
        command = self.core.tick(now)
        if was_active and not self.core.active:
            self._release()
        self._publish(command)

    def _publish(self, command):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = (float(v) for v in command)
        self.cmd_pub.publish(msg)
        self.active_pub.publish(Bool(data=self.core.active))
        now = time.monotonic()
        forward_blocked_by = []
        if self.core.active and self.core.error is not None:
            x, y, yaw = self.core.error
            settings = self.core.settings
            # Report why this phase actually set vx to zero. Far approach is
            # intentionally allowed to move and correct at the same time.
            if abs(msg.linear.x) <= 1e-9:
                if self.core.state == 'FAR_ALIGN':
                    if abs(y) >= settings.far_stop_y:
                        forward_blocked_by.append('lateral')
                    if abs(yaw) >= settings.far_stop_yaw:
                        forward_blocked_by.append('heading')
                elif self.core.state == 'FAR_HEADING':
                    forward_blocked_by.append('heading')
                elif self.core.state == 'FINE_ALIGN':
                    if abs(y) > settings.align_y:
                        forward_blocked_by.append('lateral')
                    if abs(yaw) > settings.align_yaw:
                        forward_blocked_by.append('heading')
                elif self.core.state == 'FINAL_ALIGN':
                    if abs(y) > settings.insertion_y:
                        forward_blocked_by.append('lateral')
                    if abs(yaw) > settings.insertion_yaw:
                        forward_blocked_by.append('heading')
                if x <= settings.tolerance_x:
                    forward_blocked_by.append('forward_tolerance')
                if self.core.state in {'SETTLING', 'EXTRA_SETTLING'}:
                    forward_blocked_by.append('settling')
            if now-self._last_feedback_time >= 1.0:
                pose_age = now-self.core.observed_at
                self.get_logger().info(
                    f'CONTROL: {self.core.state}, '
                    f'error_x={x:+.3f} m, error_y={y:+.3f} m, '
                    f'error_yaw={math.degrees(yaw):+.2f} deg; '
                    f'cmd_vx={msg.linear.x:+.4f} m/s, cmd_vy={msg.linear.y:+.4f} m/s, '
                    f'cmd_wz={msg.angular.z:+.4f} rad/s; '
                    f'forward_blocked_by={",".join(forward_blocked_by) or "none"}, '
                    f'control_granted={self.granted}, pose_age={pose_age:.3f} s')
                self._last_feedback_time = now
        else:
            self._last_feedback_time = -np.inf
        status = json.dumps({'state': self.core.state, 'reason': self.core.reason,
                             'target_loaded': self.goal is not None,
                             'target_compatible': (self.goal is not None
                                                   and self.goal_context == self.detector_context),
                             'camera_mount_calibrated': self.calibrated,
                             'marker_visible': (self.core.observed_at is not None
                                                and now-self.core.observed_at
                                                <= self.core.settings.pose_command_timeout),
                             'startup_teach_pending': self.startup_teach_pending,
                             'command': [msg.linear.x, msg.linear.y, msg.angular.z],
                             'control_granted': self.granted,
                             'forward_blocked_by': forward_blocked_by,
                             'error': self.core.error.tolist() if self.core.error is not None else None},
                            sort_keys=True)
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            self.last_status = status
        log_state = (self.core.state, self.core.reason)
        # Noisy measurements can change mismatch values every frame; keep the
        # full status topic current without flooding the launch terminal.
        if (log_state != getattr(self, '_last_log', None)
                and (self.core.state not in {'POSE_MISMATCH', 'READY'}
                     or now-getattr(self, '_last_log_time', -np.inf) >= 1.0)):
            self.get_logger().info(f'{self.core.state}: {self.core.reason}')
            self._last_log, self._last_log_time = log_state, now

    def stop(self):
        self._finish('CANCELLED', 'Controller shutting down')


def main(args=None):
    # Keep the context alive while the finally block publishes stop/release.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = DockingControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A terminal/launcher may deliver SIGINT twice; finish cleanup once.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
