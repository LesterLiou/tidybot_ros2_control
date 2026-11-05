"""ROS 2 driver node for the TidyBot powered-caster base."""

import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from base_controller import Vehicle


def _vector_parameter(node, name, default):
    """Declare and validate a three-element floating-point parameter."""
    value = list(node.declare_parameter(name, default).value)
    if len(value) != 3:
        raise ValueError(f"Parameter '{name}' must contain exactly three values")
    return tuple(float(item) for item in value)


class TidyBotBaseNode(Node):
    """Bridge ROS velocity commands to the TidyBot low-level controller."""

    def __init__(self):
        super().__init__('tidybot_base_node')

        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        enable_topic = self.declare_parameter('enable_topic', '/tidybot/enable').value
        frame_topic = self.declare_parameter('frame_topic', '/tidybot/control_frame').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self._odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self._base_frame = self.declare_parameter('base_frame', 'base_link').value
        self._require_enable = bool(self.declare_parameter('require_enable', True).value)
        self._command_timeout = float(self.declare_parameter('command_timeout', 0.30).value)
        self._publish_odom = bool(self.declare_parameter('publish_odom', True).value)
        odom_rate = float(self.declare_parameter('odom_publish_rate', 30.0).value)
        max_vel = _vector_parameter(self, 'max_velocity', [0.25, 0.25, 0.79])
        max_accel = _vector_parameter(self, 'max_acceleration', [0.25, 0.25, 0.79])

        if self._command_timeout <= 0.0:
            raise ValueError("Parameter 'command_timeout' must be greater than zero")
        if odom_rate <= 0.0:
            raise ValueError("Parameter 'odom_publish_rate' must be greater than zero")

        self._enabled = not self._require_enable
        self._command_frame = 'local'
        self._last_cmd_time = None
        self._watchdog_stopped = True
        self._closed = False
        self._close_lock = threading.Lock()

        self.get_logger().info('Initializing the TidyBot motor controller...')
        self.vehicle = Vehicle(max_vel=max_vel, max_accel=max_accel)
        self.vehicle.start_control()

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(Bool, enable_topic, self._enable_callback, state_qos)
        self.create_subscription(String, frame_topic, self._frame_callback, state_qos)
        self.create_timer(min(0.05, self._command_timeout / 2.0), self._watchdog_callback)

        self._odom_publisher = None
        if self._publish_odom:
            self._odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
            self.create_timer(1.0 / odom_rate, self._publish_odometry)

        if self._require_enable:
            self.get_logger().warn(
                'Base is locked. Press Y, then hold LB and RB together to move.'
            )
        else:
            self.get_logger().warn('Base enable interlock is disabled by parameter.')

    def _send_stop(self):
        if self.vehicle is not None:
            self.vehicle.set_target_velocity(np.zeros(3), frame='local')

    def _enable_callback(self, msg):
        enabled = bool(msg.data) or not self._require_enable
        if enabled == self._enabled:
            return

        self._enabled = enabled
        if enabled:
            self._last_cmd_time = time.monotonic()
            self._watchdog_stopped = False
            self.get_logger().info('Base enabled by gamepad dead-man switch.')
        else:
            self._send_stop()
            self._watchdog_stopped = True
            self.get_logger().info('Base disabled; commanding zero velocity.')

    def _frame_callback(self, msg):
        if msg.data not in ('local', 'global'):
            self.get_logger().warn(
                f"Ignoring unknown control frame '{msg.data}'; expected 'local' or 'global'.",
                throttle_duration_sec=2.0,
            )
            return
        self._command_frame = msg.data

    def _cmd_vel_callback(self, msg):
        velocity = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=float)
        if not np.all(np.isfinite(velocity)):
            self.get_logger().error('Rejected cmd_vel containing NaN or infinity.')
            self._send_stop()
            return

        self._last_cmd_time = time.monotonic()
        if not self._enabled:
            self._send_stop()
            return

        self._watchdog_stopped = False
        self.vehicle.set_target_velocity(velocity, frame=self._command_frame)

    def _watchdog_callback(self):
        if not self._enabled or self._last_cmd_time is None:
            return
        if time.monotonic() - self._last_cmd_time <= self._command_timeout:
            return
        if not self._watchdog_stopped:
            self._send_stop()
            self._watchdog_stopped = True
            self.get_logger().warn('cmd_vel timed out; commanding zero velocity.')

    def _publish_odometry(self):
        pose = np.asarray(self.vehicle.x, dtype=float).copy()
        velocity_global = np.asarray(self.vehicle.dx, dtype=float).copy()
        theta = pose[2]
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        velocity_local = np.array([
            cos_theta * velocity_global[0] + sin_theta * velocity_global[1],
            -sin_theta * velocity_global[0] + cos_theta * velocity_global[1],
            velocity_global[2],
        ])

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        msg.twist.twist.linear.x = float(velocity_local[0])
        msg.twist.twist.linear.y = float(velocity_local[1])
        msg.twist.twist.angular.z = float(velocity_local[2])
        self._odom_publisher.publish(msg)

    def close(self):
        """Stop the base once; safe to call from more than one shutdown path."""
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            if self.vehicle is not None:
                self._send_stop()
                self.vehicle.stop_control()
                self.vehicle = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TidyBotBaseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.get_logger().info('Shutting down the TidyBot base safely...')
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
