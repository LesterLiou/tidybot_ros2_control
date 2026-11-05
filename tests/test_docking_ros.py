"""ROS graph tests with synthetic data only: isolated topics/domain, no base imports."""

import json
import os
import time
import uuid

import cv2
import numpy as np
import pytest
import ros2_gamepad_teleop as gamepad_module

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from docking_core import matrix_quaternion
from ros2_aruco_detector import ArucoDetectorNode
from ros2_control_mux import ControlMuxNode
from ros2_docking_controller import DockingControllerNode
from ros2_gamepad_teleop import GamepadTeleopNode


class Harness:
    def __init__(self, context, target_file):
        self.context = context
        self.target_file = target_file
        self.executor = SingleThreadedExecutor(context=context)
        self.node = Node('docking_test_harness', context=context)
        self.controller = self.make_controller()
        self.mux = ControlMuxNode(context=context, parameter_overrides=[
            Parameter('manual_max_velocity', value=[0.1, 0.1, 0.5]),
        ])
        self.teleop = GamepadTeleopNode(context=context, parameter_overrides=[
            Parameter('cmd_vel_topic', value='/teleop/cmd_vel'),
            Parameter('enable_topic', value='/teleop/enable'),
            Parameter('frame_topic', value='/teleop/control_frame'),
            Parameter('stop_topic', value='/teleop/stop'),
            Parameter('max_velocity', value=[1.0, 1.0, 1.57]),
        ])
        self.nodes = [self.node, self.controller, self.mux, self.teleop]
        for node in self.nodes:
            self.executor.add_node(node)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_pub = self.node.create_publisher(String, '/docking/detector_config', qos)
        self.pose_pub = self.node.create_publisher(PoseStamped, '/docking/marker_pose', 1)
        self.joy_pub = self.node.create_publisher(Joy, '/joy', qos_profile_sensor_data)
        self.command = Twist()
        self.enabled = False
        self.node.create_subscription(Twist, '/cmd_vel', self._command, 1)
        self.node.create_subscription(Bool, '/tidybot/enable', self._enable, qos)
        self.buttons = [0]*13
        self.axes = [0.0]*8
        self.marker_x = 1.0
        self.marker_y = 0.0
        self.marker_z = 0.0
        self.publish_pose = self.publish_joy = True
        self.extra_publish = None
        self.config = json.dumps({'fingerprint': 'test-mount-1', 'calibrated': True,
                                  'settings': {'base_frame': 'base_link'}}, sort_keys=True)
        self.config_pub.publish(String(data=self.config))
        self.next_publish = 0

    def make_controller(self, **overrides):
        parameters = {'target_file': str(self.target_file), 'teach_duration': 0.25,
                      'teach_min_samples': 5, 'settle_time': 0.15,
                      'pose_command_timeout': 0.08, 'pose_timeout': 0.25}
        parameters.update(overrides)
        return DockingControllerNode(context=self.context, parameter_overrides=[
            Parameter(name, value=value) for name, value in parameters.items()])

    def restart_controller(self, **overrides):
        self.controller.stop()
        self.spin(0.1)  # Complete the release service round trip before destroying its client.
        self.executor.remove_node(self.controller)
        self.nodes.remove(self.controller)
        self.controller.destroy_node()
        self.controller = self.make_controller(**overrides)
        self.nodes.append(self.controller)
        self.executor.add_node(self.controller)

    def _command(self, msg):
        self.command = msg

    def _enable(self, msg):
        self.enabled = msg.data

    def pump(self):
        now = time.monotonic()
        if now >= self.next_publish:
            self.next_publish = now+0.02
            if self.publish_joy:
                joy = Joy()
                joy.header.stamp = self.node.get_clock().now().to_msg()
                joy.buttons, joy.axes = self.buttons, self.axes
                self.joy_pub.publish(joy)
            if self.publish_pose:
                pose = PoseStamped()
                pose.header.stamp = self.node.get_clock().now().to_msg()
                pose.header.frame_id = 'base_link'
                pose.pose.position.x = self.marker_x
                pose.pose.position.y = self.marker_y
                pose.pose.position.z = self.marker_z
                pose.pose.orientation.w = 1.0
                self.pose_pub.publish(pose)
            if self.extra_publish:
                self.extra_publish()
        self.executor.spin_once(timeout_sec=0.005)

    def spin(self, duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            self.pump()

    def call(self, name):
        client = self.node.create_client(Trigger, name)
        until = time.monotonic()+2
        while not client.service_is_ready() and time.monotonic() < until:
            self.pump()
        assert client.service_is_ready(), name
        future = client.call_async(Trigger.Request())
        until = time.monotonic()+2
        while not future.done() and time.monotonic() < until:
            self.pump()
        assert future.done(), name
        result = future.result()
        self.node.destroy_client(client)
        return result

    def teach_and_start(self, distance=0.2):
        self.spin(0.25)
        result = self.call('/docking/teach')
        assert result.success, result.message
        self.spin(0.45)
        assert self.controller.core.state == 'TAUGHT', self.controller.core.reason
        self.marker_x += distance
        self.spin(0.1)
        result = self.call('/docking/start')
        assert result.success, result.message
        self.spin(0.2)
        assert self.controller.core.active, self.controller.core.reason
        assert self.mux.lease

    def close(self):
        self.controller.stop()
        self.mux.stop()
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()


@pytest.fixture
def graph(tmp_path, monkeypatch):
    # Never participate in a robot's default ROS domain or use its command topics.
    monkeypatch.setenv('ROS_DOMAIN_ID', '179')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'logs'))
    context = Context()
    prefix = '/test_'+uuid.uuid4().hex
    topics = ['/cmd_vel', '/joy', '/tidybot/enable', '/tidybot/control_frame',
              '/tidybot/control_status', '/teleop/cmd_vel', '/teleop/enable',
              '/teleop/control_frame', '/teleop/stop', '/docking/marker_pose',
              '/docking/detector_config', '/docking/cmd_vel', '/docking/active',
              '/docking/status', '/docking/control_granted', '/docking/set_control',
              '/docking/teach', '/docking/start', '/docking/cancel', '/docking/debug_image',
              '/docking/debug_image/compressed',
              '/camera/camera/color/image_raw', '/camera/camera/color/camera_info', '/tf', '/tf_static']
    args = ['--ros-args']
    for topic in topics:
        args += ['-r', topic+':='+prefix+topic]
    rclpy.init(args=args, context=context)
    harness = None
    try:
        harness = Harness(context, tmp_path/'target.yaml')
        yield harness
    finally:
        if harness is not None:
            harness.close()
        context.shutdown()


def test_teach_start_manual_takeover_and_no_automatic_resume(graph):
    graph.teach_and_start()
    assert graph.command.linear.x > 0 and graph.enabled
    graph.buttons[4] = 1  # Y arms teleop without moving.
    graph.spin(0.08)
    graph.buttons[4] = 0
    graph.spin(0.04)
    graph.buttons[6] = graph.buttons[7] = 1
    graph.axes[1] = 0.8
    graph.spin(0.2)
    assert graph.mux.owner == 'MANUAL'
    assert graph.command.linear.x == pytest.approx(0.1)  # Mux also enforces the ceiling.
    assert graph.controller.core.state == 'CANCELLED'
    graph.buttons[6] = graph.buttons[7] = 0
    graph.spin(0.15)
    assert not graph.mux.lease and not graph.enabled
    assert graph.command.linear.x == 0


def test_gamepad_a_button_requests_docking(graph):
    graph.spin(0.25)
    result = graph.call('/docking/teach')
    assert result.success, result.message
    graph.spin(0.45)
    assert graph.controller.core.state == 'TAUGHT', graph.controller.core.reason
    graph.marker_x += 0.2
    graph.spin(0.1)

    graph.buttons[4] = 1  # Y arms the gamepad and clears any mux stop latch.
    graph.spin(0.08)
    graph.buttons[4] = 0
    graph.spin(0.04)
    graph.buttons[0] = 1  # A requests /docking/start once on its rising edge.
    graph.spin(0.12)
    graph.buttons[0] = 0
    graph.spin(0.2)

    assert graph.controller.core.active, graph.controller.core.reason
    assert graph.mux.lease
    assert graph.enabled and graph.command.linear.x > 0


def test_gamepad_menu_button_toggles_rosbag_without_blocking_control(
        graph, monkeypatch, tmp_path):
    class FakeProcess:
        pid = 4242
        returncode = None

        def poll(self):
            return self.returncode

        def wait(self, timeout=None):
            return self.returncode

    process = FakeProcess()
    launches = []
    signals = []

    def fake_popen(command, **kwargs):
        launches.append((command, kwargs))
        return process

    monkeypatch.setattr(gamepad_module.shutil, 'which', lambda executable: '/opt/ros/humble/bin/ros2')
    monkeypatch.setattr(gamepad_module.subprocess, 'Popen', fake_popen)
    monkeypatch.setattr(gamepad_module.os, 'killpg', lambda pid, sig: signals.append((pid, sig)))
    graph.teleop._rosbag_output_directory = tmp_path/'bags'
    graph.teleop._rosbag_min_free_gb = 0.0

    graph.buttons[11] = 1
    graph.spin(0.08)
    graph.buttons[11] = 0
    graph.spin(0.04)
    assert graph.teleop._rosbag_process is process
    assert len(launches) == 1
    command, options = launches[0]
    assert command[:3] == ['/opt/ros/humble/bin/ros2', 'bag', 'record']
    assert '/docking/debug_image/compressed' in command
    assert '--compression-format' in command and 'zstd' in command
    assert options['start_new_session'] is True

    graph.buttons[11] = 1
    graph.spin(0.08)
    graph.buttons[11] = 0
    graph.spin(0.04)
    assert signals == [(process.pid, gamepad_module.signal.SIGINT)]
    assert graph.teleop._rosbag_stopping
    process.returncode = 0
    graph.spin(0.3)
    assert graph.teleop._rosbag_process is None


def test_gamepad_guide_button_stops_robot_and_requests_launch_shutdown(
        graph, monkeypatch):
    shutdown_signals = []
    monkeypatch.setattr(
        gamepad_module.os, 'kill',
        lambda pid, sig: shutdown_signals.append((pid, sig)),
    )

    graph.buttons[4] = 1  # Arm with Y.
    graph.spin(0.08)
    graph.buttons[4] = 0
    graph.buttons[6] = graph.buttons[7] = 1
    graph.axes[1] = 0.5
    graph.spin(0.15)
    assert graph.enabled and graph.command.linear.x > 0

    graph.buttons[12] = 1  # Xbox/Guide requests the whole launch to exit.
    graph.spin(0.2)
    assert graph.teleop._launch_shutdown_requested
    assert graph.teleop._launch_shutdown_signal_sent
    assert shutdown_signals == [(os.getpid(), gamepad_module.signal.SIGINT)]
    assert graph.mux.stopped
    assert not graph.enabled and graph.command.linear.x == 0


def test_lost_marker_stops_and_stays_stopped_when_marker_returns(graph):
    graph.teach_and_start()
    graph.publish_pose = False
    graph.spin(0.35)
    assert graph.controller.core.state == 'FAILED'
    assert not graph.enabled and graph.command.linear.x == 0
    graph.publish_pose = True
    graph.spin(0.2)
    assert not graph.controller.core.active and not graph.mux.lease


def test_brief_marker_loss_holds_zero_and_resumes_with_same_lease(graph):
    graph.teach_and_start()
    graph.publish_pose = False
    graph.spin(0.12)
    assert graph.controller.core.state == 'WAITING_FOR_POSE'
    assert graph.controller.core.active and graph.mux.lease
    assert graph.command.linear.x == 0
    graph.publish_pose = True
    graph.spin(0.15)
    assert graph.controller.core.state == 'FAR_APPROACH'
    assert graph.controller.core.active and graph.mux.lease
    assert graph.command.linear.x > 0


def test_no_visual_progress_releases_control_and_requires_another_start(graph):
    graph.restart_controller(progress_timeout=0.45)
    graph.teach_and_start(distance=0.08)
    assert graph.enabled and graph.command.linear.x > 0
    graph.spin(0.45)  # Fresh observations continue, but the virtual robot is stuck.
    assert graph.controller.core.state == 'FAILED'
    assert 'No visual progress' in graph.controller.core.reason
    assert 'phase=FINE_APPROACH' in graph.controller.core.reason
    assert 'error_x=+0.080 m' in graph.controller.core.reason
    assert not graph.enabled and not graph.mux.lease
    assert graph.command.linear.x == 0
    graph.marker_x -= 0.02
    graph.spin(0.2)
    assert graph.controller.core.state == 'FAILED'
    assert not graph.controller.core.active and not graph.mux.lease


def test_status_reports_the_error_command_and_reason_forward_motion_is_blocked(graph):
    reports = []
    graph.node.create_subscription(
        String, '/docking/status', lambda msg: reports.append(json.loads(msg.data)), 10)
    graph.teach_and_start(distance=0.16)
    report = reports[-1]
    assert report['state'] == 'FAR_APPROACH'
    assert report['control_granted']
    assert report['error'] == pytest.approx([0.16, 0.0, 0.0])
    assert report['command'][0] > 0
    assert report['forward_blocked_by'] == []

    graph.marker_y = 0.03
    graph.spin(0.15)
    report = reports[-1]
    assert report['state'] == 'FAR_APPROACH'
    assert report['forward_blocked_by'] == []
    assert report['command'][0] > 0
    assert report['command'][1] > 0

    graph.marker_x = 1.08
    graph.spin(0.15)
    report = reports[-1]
    assert report['state'] == 'FINE_ALIGN'
    assert report['forward_blocked_by'] == ['lateral']
    assert report['error'] == pytest.approx([0.08, 0.03, 0.0])
    assert report['command'][0] == 0
    assert report['command'][1] > 0

    assert graph.call('/docking/cancel').success
    graph.spin(0.15)
    report = reports[-1]
    assert report['state'] == 'CANCELLED'
    assert report['command'] == [0.0, 0.0, 0.0]
    assert not report['control_granted']
    assert report['forward_blocked_by'] == []


def test_back_and_gamepad_loss_latch_stop(graph):
    graph.teach_and_start()
    graph.buttons[10] = 1
    graph.spin(0.1)
    assert graph.mux.stopped and not graph.mux.lease and not graph.enabled
    graph.buttons[10] = 0
    graph.spin(0.05)
    graph.buttons[4] = 1
    graph.spin(0.05)
    graph.buttons[4] = 0
    graph.spin(0.05)
    assert not graph.mux.stopped
    assert graph.call('/docking/start').success
    graph.spin(0.15)
    assert graph.mux.lease
    graph.publish_joy = False
    graph.spin(0.45)
    assert graph.mux.stopped and not graph.enabled and not graph.mux.lease
    graph.publish_joy = True
    graph.spin(0.1)
    assert graph.mux.stopped and graph.command.linear.x == 0


def test_pose_reached_and_explicit_cancel(graph):
    graph.teach_and_start(distance=0.04)
    graph.marker_x = 1.0
    graph.spin(0.4)
    assert graph.controller.core.state == 'REACHED'
    assert not graph.enabled and not graph.mux.lease
    graph.marker_x = 1.05
    graph.spin(0.1)
    assert graph.call('/docking/start').success
    assert graph.call('/docking/cancel').success
    graph.spin(0.2)
    assert graph.controller.core.state == 'CANCELLED'
    assert not graph.enabled and not graph.mux.lease


def test_controller_heartbeat_failure_revokes_lease_permanently(graph):
    graph.teach_and_start()
    graph.executor.remove_node(graph.controller)
    graph.spin(0.4)
    assert not graph.mux.lease and not graph.enabled
    graph.executor.add_node(graph.controller)
    graph.spin(0.2)
    assert not graph.mux.lease and graph.command.linear.x == 0


def test_mount_context_change_requires_new_teaching(graph):
    graph.teach_and_start()
    data = json.loads(graph.config)
    data['fingerprint'] = 'different-mount'
    graph.config_pub.publish(String(data=json.dumps(data, sort_keys=True)))
    graph.spin(0.15)
    result = graph.call('/docking/start')
    assert not result.success and 'teach again' in result.message
    assert not graph.enabled


def test_unconfirmed_mount_cannot_teach_or_start(graph):
    data = json.loads(graph.config)
    data['calibrated'] = False
    graph.config_pub.publish(String(data=json.dumps(data, sort_keys=True)))
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.2)
    assert graph.controller.core.state == 'NEEDS_CAMERA_SETUP'
    assert not graph.call('/docking/teach').success
    assert not graph.call('/docking/start').success
    assert not graph.target_file.exists()
    assert not graph.enabled and not graph.mux.lease


def test_boot_reports_missing_target_and_reuses_it_after_restart_without_motion(graph):
    graph.spin(0.3)
    assert graph.controller.core.state == 'NEEDS_TEACH'
    assert not graph.target_file.exists()
    assert graph.call('/docking/teach').success
    graph.spin(0.45)
    saved = graph.target_file.read_bytes()
    graph.marker_x += 0.2
    graph.restart_controller()
    graph.spin(0.35)
    assert graph.controller.core.state == 'READY', graph.controller.core.reason
    assert graph.target_file.read_bytes() == saved
    assert not graph.controller.core.active and not graph.enabled and not graph.mux.lease
    assert graph.command.linear.x == 0
    graph.publish_pose = False
    graph.spin(0.3)
    assert graph.controller.core.state == 'WAITING_FOR_MARKER'
    graph.publish_pose = True
    graph.spin(0.2)
    assert graph.controller.core.state == 'READY'
    assert not graph.controller.core.active and not graph.enabled


def test_ready_checks_height_and_recovers_without_starting_motion(graph):
    graph.spin(0.25)
    assert graph.call('/docking/teach').success
    graph.spin(0.45)
    graph.restart_controller()
    graph.spin(0.3)
    assert graph.controller.core.state == 'READY'
    graph.marker_z = 0.08
    graph.spin(0.2)
    assert graph.controller.core.state == 'POSE_MISMATCH'
    response = graph.call('/docking/start')
    assert not response.success and 'height=+0.080 m' in response.message
    assert not graph.controller.core.active and not graph.enabled and not graph.mux.lease
    graph.marker_z = 0.0
    graph.spin(0.2)
    assert graph.controller.core.state == 'READY'
    assert not graph.controller.core.active and not graph.enabled and not graph.mux.lease


def test_startup_teaching_waits_for_marker_saves_once_and_never_starts_docking(graph):
    graph.publish_pose = False
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.3)
    assert graph.controller.core.state == 'WAITING_FOR_MARKER'
    assert graph.controller.startup_teach_pending
    assert not graph.target_file.exists()
    graph.publish_pose = True
    graph.spin(0.65)
    assert graph.controller.core.state == 'TAUGHT', graph.controller.core.reason
    saved = graph.target_file.read_bytes()
    assert not graph.controller.startup_teach_pending
    assert not graph.controller.core.active and not graph.enabled and not graph.mux.lease
    graph.marker_x += 0.1
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.35)
    assert graph.controller.core.state == 'READY'
    assert graph.target_file.read_bytes() == saved
    assert not graph.enabled and graph.command.linear.x == 0


def test_startup_teaching_can_be_cancelled_before_the_marker_appears(graph):
    graph.publish_pose = False
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.25)
    assert graph.call('/docking/cancel').success
    graph.publish_pose = True
    graph.spin(0.65)
    assert graph.controller.core.state == 'CANCELLED'
    assert not graph.controller.startup_teach_pending
    assert not graph.target_file.exists() and not graph.enabled


def test_startup_teaching_rejects_unstable_samples_and_does_not_retry(graph):
    graph.restart_controller(teach_on_start=True)
    graph.extra_publish = lambda: setattr(graph, 'marker_x', 1.03 if graph.marker_x == 1.0 else 1.0)
    graph.spin(0.8)
    assert graph.controller.core.state == 'FAILED'
    assert 'unstable' in graph.controller.core.reason
    graph.extra_publish = None
    graph.spin(0.5)
    assert graph.controller.core.state == 'FAILED'
    assert not graph.target_file.exists() and not graph.enabled


def test_boot_rejects_changed_mount_without_overwriting_saved_target(graph):
    graph.spin(0.25)
    assert graph.call('/docking/teach').success
    graph.spin(0.45)
    saved = graph.target_file.read_bytes()
    data = json.loads(graph.config)
    data['fingerprint'] = 'different-mount'
    graph.config = json.dumps(data, sort_keys=True)
    graph.config_pub.publish(String(data=graph.config))
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.4)
    assert graph.controller.core.state == 'NEEDS_TEACH'
    assert not graph.call('/docking/start').success
    assert graph.target_file.read_bytes() == saved
    assert not graph.enabled and not graph.mux.lease


def test_startup_teaching_does_not_replace_an_invalid_existing_file(graph):
    graph.target_file.write_text('invalid target')
    graph.restart_controller(teach_on_start=True)
    graph.spin(0.3)
    assert graph.controller.core.state == 'NEEDS_TEACH'
    assert graph.target_file.read_text() == 'invalid target'
    assert not graph.controller.startup_teach_pending and not graph.enabled


def test_invalid_velocity_latches_stop_even_if_controller_keeps_publishing(graph):
    graph.teach_and_start()
    publisher = graph.node.create_publisher(Twist, '/docking/cmd_vel', 1)
    msg = Twist()
    msg.linear.x = float('nan')
    # Publish for several graph cycles so the controller's 30 Hz command
    # cannot overwrite the one invalid sample in a depth-one subscription.
    graph.extra_publish = lambda: publisher.publish(msg)
    graph.spin(0.2)
    graph.extra_publish = None
    assert graph.mux.stopped
    assert not graph.mux.lease and not graph.enabled
    assert graph.command.linear.x == 0


def test_replayed_pose_stamps_do_not_keep_control_alive(graph):
    graph.teach_and_start()
    graph.publish_pose = False
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = graph.node.get_clock().now().to_msg()
    pose.pose.position.x = graph.marker_x
    pose.pose.orientation.w = 1.0
    graph.extra_publish = lambda: graph.pose_pub.publish(pose)
    graph.spin(0.4)
    assert graph.controller.core.state == 'FAILED'
    assert not graph.enabled and not graph.mux.lease


def test_camera_image_info_and_tf_produce_base_pose_but_old_images_do_not(graph):
    from cv_bridge import CvBridge
    from tf2_ros import StaticTransformBroadcaster
    graph.publish_pose = False
    detector = ArucoDetectorNode(context=graph.context, parameter_overrides=[
        Parameter('camera_mount_calibrated', value=True),
    ])
    graph.nodes.append(detector)
    graph.executor.add_node(detector)
    broadcaster = StaticTransformBroadcaster(graph.node)
    tf = TransformStamped()
    tf.header.stamp = graph.node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'camera_link', 'camera_color_optical_frame'
    q = matrix_quaternion(np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]))
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
    broadcaster.sendTransform(tf)
    image_pub = graph.node.create_publisher(Image, '/camera/camera/color/image_raw', qos_profile_sensor_data)
    info_pub = graph.node.create_publisher(CameraInfo, '/camera/camera/color/camera_info', qos_profile_sensor_data)
    compressed_images = []
    graph.node.create_subscription(
        CompressedImage, '/docking/debug_image/compressed',
        compressed_images.append, qos_profile_sensor_data,
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    draw = getattr(cv2.aruco, 'generateImageMarker', getattr(cv2.aruco, 'drawMarker', None))
    image = np.full((720, 960), 255, dtype=np.uint8)
    image[280:440, 400:560] = draw(dictionary, 0, 160)
    bridge = CvBridge()
    old_images = False

    def publish_camera():
        msg = bridge.cv2_to_imgmsg(image, encoding='mono8')
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.header.stamp = graph.node.get_clock().now().to_msg()
        if old_images:
            msg.header.stamp.sec -= 2
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = 960, 720
        info.distortion_model = 'plumb_bob'
        info.k = [800., 0., 480., 0., 800., 360., 0., 0., 1.]
        info.d = [0.]*5
        info_pub.publish(info)
        image_pub.publish(msg)

    graph.extra_publish = publish_camera
    graph.spin(0.8)
    pose = graph.controller.core.observation
    assert pose is not None
    assert pose[0, 3] == pytest.approx(0.6, abs=0.006)
    assert pose[1, 3] == pytest.approx(0, abs=0.002)
    assert pose[2, 3] == pytest.approx(0.4, abs=0.002)
    assert compressed_images
    boxed = cv2.imdecode(
        np.frombuffer(compressed_images[-1].data, dtype=np.uint8), cv2.IMREAD_COLOR
    )
    assert boxed is not None and boxed.shape[:2] == image.shape
    green = ((boxed[:, :, 1] > 150) & (boxed[:, :, 0] < 140)
             & (boxed[:, :, 2] < 140))
    assert np.count_nonzero(green) > 20  # ArUco outline/reprojection text is present.
    old_images = True
    graph.spin(0.2)  # Drain fresh messages already in flight before the switch.
    accepted_at = graph.controller.core.observed_at
    accepted_stamp = detector.last_stamp
    graph.spin(0.3)
    assert graph.controller.core.observed_at == accepted_at
    assert detector.last_stamp == accepted_stamp
