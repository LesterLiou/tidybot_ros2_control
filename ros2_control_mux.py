"""Single owner of base commands, with expiring autonomy leases and manual override."""

import json
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool


class ControlMuxNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('tidybot_control_mux', **node_kwargs)
        p = lambda name, default: self.declare_parameter(name, default).value
        self.timeout = float(p('command_timeout', 0.25))
        self.joy_timeout = float(p('joy_timeout', 0.30))
        self.require_gamepad = bool(p('require_gamepad', True))
        self.rate = float(p('publish_rate', 50.0))
        self.arm_button = int(p('button_arm', 4))
        self.stop_button = int(p('button_disarm', 10))
        self.deadman_buttons = list(p('deadman_buttons', [6, 7]))
        self.manual_limits = np.asarray(p('manual_max_velocity', [1.0, 1.0, 1.57]), dtype=float)
        self.dock_limits = np.asarray(p('docking_max_velocity', [0.04, 0.04, 0.12]), dtype=float)
        if (not np.all(np.isfinite([self.timeout, self.joy_timeout, self.rate]))
                or min(self.timeout, self.joy_timeout, self.rate) <= 0
                or any(a.shape != (3,) or not np.all(np.isfinite(a)) or np.any(a <= 0)
                       for a in [self.manual_limits, self.dock_limits])
                or min([self.arm_button, self.stop_button]+self.deadman_buttons) < 0):
            raise ValueError('Invalid command limits, buttons or watchdog settings')
        self.manual_command = self.dock_command = None
        self.manual_time = self.dock_time = -np.inf
        self.manual_enabled = self.dock_active = False
        self.manual_enable_time = self.dock_active_time = -np.inf
        self.manual_frame = 'local'
        self.joy_time = None
        self.buttons = []
        self.stopped = False
        self.lease = False
        self.had_dock_active = False
        self.owner = 'NONE'
        self.reason = 'Waiting for commands'
        self.last_status = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.enable_pub = self.create_publisher(Bool, '/tidybot/enable', qos)
        self.frame_pub = self.create_publisher(String, '/tidybot/control_frame', qos)
        self.grant_pub = self.create_publisher(Bool, '/docking/control_granted', qos)
        self.status_pub = self.create_publisher(String, '/tidybot/control_status', qos)
        self.create_subscription(Twist, '/teleop/cmd_vel', self._manual_cmd, 1)
        self.create_subscription(Bool, '/teleop/enable', self._manual_enable, qos)
        self.create_subscription(String, '/teleop/control_frame', self._manual_frame, qos)
        self.create_subscription(Empty, '/teleop/stop', self._stop_event, 1)
        self.create_subscription(Twist, '/docking/cmd_vel', self._dock_cmd, 1)
        self.create_subscription(Bool, '/docking/active', self._dock_active, 1)
        self.create_subscription(Joy, '/joy', self._joy, qos_profile_sensor_data)
        self.create_service(SetBool, '/docking/set_control', self._set_control)
        self.create_timer(1.0/self.rate, self._tick)
        self._output(np.zeros(3), False)

    @staticmethod
    def _button(buttons, index):
        return 0 <= index < len(buttons) and bool(buttons[index])

    def _joy(self, msg):
        previous = self.buttons
        self.buttons = list(msg.buttons)
        self.joy_time = time.monotonic()
        if self._button(self.buttons, self.stop_button):
            self._stop_event(None)
        elif (self._button(self.buttons, self.arm_button)
              and not self._button(previous, self.arm_button)):
            self.stopped = False

    def _stop_event(self, msg):
        self.stopped = True
        self._revoke('Stopped with Back; press Y before enabling control again')
        self._output(np.zeros(3), False)

    def _manual_frame(self, msg):
        self.manual_frame = msg.data

    def _manual_enable(self, msg):
        self.manual_enabled = bool(msg.data)
        self.manual_enable_time = time.monotonic()
        if self.manual_enabled:
            self._revoke('Manual takeover; explicit docking restart required')
        else:
            self.manual_command = None

    def _dock_active(self, msg):
        self.dock_active = bool(msg.data)
        self.dock_active_time = time.monotonic()
        if self.lease and self.dock_active:
            self.had_dock_active = True
        elif self.lease and self.had_dock_active:
            self._revoke('Docking operation ended')

    def _command(self, msg):
        value = np.array([msg.linear.x, msg.linear.y, msg.angular.z])
        if not np.all(np.isfinite(value)):
            self.stopped = True
            self._revoke('Invalid velocity; press Y to re-arm')
            self.manual_command = self.dock_command = None
            self._output(np.zeros(3), False)
            return None
        return value

    def _manual_cmd(self, msg):
        self.manual_command = self._command(msg)
        self.manual_time = time.monotonic()

    def _dock_cmd(self, msg):
        value = self._command(msg)
        if self.lease:
            self.dock_command, self.dock_time = value, time.monotonic()

    def _revoke(self, reason):
        had_lease = self.lease
        self.lease = False
        self.dock_command = None
        self.had_dock_active = False
        self.owner, self.reason = 'NONE', reason
        self.grant_pub.publish(Bool(data=False))
        if had_lease:
            self.get_logger().warn(f'Docking control revoked: {reason}')

    def _joy_fresh(self, now):
        return self.joy_time is not None and now-self.joy_time <= self.joy_timeout

    def _set_control(self, request, response):
        now = time.monotonic()
        if not request.data:
            self._revoke('Docking released control')
            self._output(np.zeros(3), False)
            response.success, response.message = True, 'Released'
        elif self.stopped:
            response.success, response.message = False, 'Stop is latched; press Y first'
        elif self.require_gamepad and not self._joy_fresh(now):
            response.success, response.message = False, 'A fresh gamepad stream is required for this test configuration'
        elif ((self.manual_enabled and now-self.manual_enable_time <= self.timeout)
              or any(self._button(self.buttons, b) for b in self.deadman_buttons)):
            response.success, response.message = False, 'Release the gamepad deadman buttons before docking'
        elif self.lease:
            response.success, response.message = False, 'Docking already owns control'
        else:
            self.lease = True
            self.lease_since = now
            self.had_dock_active = False
            self.dock_command, self.dock_time = None, -np.inf
            self.dock_active, self.dock_active_time = False, -np.inf
            self.owner, self.reason = 'NONE', 'Waiting for fresh docking commands'
            self.grant_pub.publish(Bool(data=True))
            response.success, response.message = True, 'Exclusive control granted'
        return response

    def _tick(self):
        now = time.monotonic()
        command, enabled = np.zeros(3), False
        if self.require_gamepad and not self._joy_fresh(now):
            if self.joy_time is not None:
                self.stopped = True
            self._revoke('Gamepad stream unavailable or timed out')
        if self.stopped:
            self._revoke('Stopped; press Y to re-arm, then explicitly restart docking')
        elif self.manual_enabled and now-self.manual_enable_time <= self.timeout:
            self._revoke('Manual takeover')
            if (self.manual_command is not None and now-self.manual_time <= self.timeout
                    and self.manual_frame == 'local' and self._joy_fresh(now)):
                command = np.clip(self.manual_command, -self.manual_limits, self.manual_limits)
                enabled, self.owner, self.reason = True, 'MANUAL', 'Gamepad control'
        elif self.lease:
            fresh = (self.dock_command is not None and now-self.dock_time <= self.timeout
                     and self.dock_active and now-self.dock_active_time <= self.timeout)
            if fresh:
                command = np.clip(self.dock_command, -self.dock_limits, self.dock_limits)
                enabled, self.owner, self.reason = True, 'DOCKING', 'Visual docking control'
            elif self.had_dock_active or now-self.lease_since > 0.5:
                self._revoke('Docking heartbeat or velocity timed out')
        if not enabled:
            self.owner = 'NONE'
        self._output(command, enabled)

    def _output(self, command, enabled):
        self.frame_pub.publish(String(data='local'))
        self.enable_pub.publish(Bool(data=enabled))
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = (float(v) for v in command)
        self.cmd_pub.publish(msg)
        self.grant_pub.publish(Bool(data=self.lease))
        status = json.dumps({'owner': self.owner, 'reason': self.reason, 'stop_latched': self.stopped},
                            sort_keys=True)
        if status != self.last_status:
            self.status_pub.publish(String(data=status))
            self.last_status = status

    def stop(self):
        self._revoke('Control mux shutting down')
        self._output(np.zeros(3), False)


def main(args=None):
    # Python handles SIGINT so zero/disable can be sent before ROS shuts down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ControlMuxNode()
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
