"""Publish fresh, calibrated marker poses in base_link; never command motors."""

import hashlib
import json
import math
import signal

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from aruco_detection import ArucoPoseEstimator
from docking_core import matrix_quaternion, transform


class ArucoDetectorNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('tidybot_aruco_detector', **node_kwargs)
        p = lambda name, default: self.declare_parameter(name, default).value
        image_topic = p('image_topic', '/camera/camera/color/image_raw')
        info_topic = p('camera_info_topic', '/camera/camera/color/camera_info')
        self.base_frame = p('base_frame', 'base_link')
        camera_frame = p('camera_frame', 'camera_link')
        self.optical_frame = p('optical_frame', 'camera_color_optical_frame')
        xyz = list(p('camera_xyz', [0.0, 0.0, 0.40]))
        rpy = list(p('camera_rpy', [0.0, 0.0, 0.0]))
        self.calibrated = bool(p('camera_mount_calibrated', False))
        publish_tf = bool(p('publish_camera_tf', True))
        dictionary = p('dictionary', 'DICT_4X4_50')
        marker_id = int(p('marker_id', 0))
        marker_size = float(p('marker_size', 0.12))
        self.image_timeout = float(p('image_timeout', 0.25))
        self.max_distance = float(p('max_marker_distance', 3.0))
        self.debug = bool(p('publish_debug_image', True))
        self.debug_compressed_rate = float(p('debug_compressed_rate', 10.0))
        self.debug_jpeg_quality = int(p('debug_jpeg_quality', 85))
        if (len(xyz) != 3 or len(rpy) != 3 or not np.all(np.isfinite(xyz+rpy))
                or self.image_timeout <= 0 or not math.isfinite(self.image_timeout)
                or self.max_distance <= 0 or not math.isfinite(self.max_distance)
                or self.debug_compressed_rate <= 0
                or not math.isfinite(self.debug_compressed_rate)
                or not 1 <= self.debug_jpeg_quality <= 100
                or not self.base_frame or camera_frame == self.base_frame):
            raise ValueError('Invalid camera extrinsics, frames or observation limits')
        self.estimator = ArucoPoseEstimator(
            dictionary, marker_id, marker_size,
            float(p('max_reprojection_error', 2.0)), float(p('min_edge_pixels', 25.0)))
        self.bridge = CvBridge()
        self.info = None
        self.last_stamp = -1
        self.last_compressed_stamp = -1
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        if publish_tf:
            self.broadcaster = StaticTransformBroadcaster(self)
            roll, pitch, yaw = rpy
            cr, sr = math.cos(roll/2), math.sin(roll/2)
            cp, sp = math.cos(pitch/2), math.sin(pitch/2)
            cy, sy = math.cos(yaw/2), math.sin(yaw/2)
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = self.base_frame, camera_frame
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
            q = msg.transform.rotation
            q.x, q.y, q.z, q.w = sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy
            self.broadcaster.sendTransform(msg)
        context = {'dictionary': dictionary, 'marker_id': marker_id, 'marker_size': marker_size,
                   'base_frame': self.base_frame, 'camera_frame': camera_frame,
                   'optical_frame': self.optical_frame, 'camera_xyz': xyz, 'camera_rpy': rpy,
                   'camera_mount_calibrated': self.calibrated}
        self.detector_context = json.dumps({
            'fingerprint': hashlib.sha256(json.dumps(context, sort_keys=True).encode()).hexdigest(),
            'calibrated': self.calibrated, 'settings': context}, sort_keys=True)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.context_pub = self.create_publisher(String, '/docking/detector_config', state_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/docking/marker_pose', 1)
        self.debug_pub = self.create_publisher(Image, '/docking/debug_image', qos_profile_sensor_data)
        self.debug_compressed_pub = self.create_publisher(
            CompressedImage, '/docking/debug_image/compressed', qos_profile_sensor_data
        )
        self.create_subscription(CameraInfo, info_topic, self._info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic, self._image, qos_profile_sensor_data)
        self.create_timer(1.0, self._publish_context)
        self._publish_context()
        self.get_logger().info(f'Detecting {dictionary}/{marker_id}, black border size {marker_size:.3f} m')
        if not self.calibrated:
            self.get_logger().warn('Camera mounting is unconfirmed: detection works, teaching/start are locked.')

    def _publish_context(self):
        self.context_pub.publish(String(data=self.detector_context))

    def _info(self, msg):
        if (msg.distortion_model not in ('plumb_bob', 'rational_polynomial')
                or msg.k[0] <= 0 or msg.k[4] <= 0 or not np.all(np.isfinite(list(msg.k)+list(msg.d)))):
            self.info = None
            self.get_logger().warn('Unsupported or uncalibrated CameraInfo', throttle_duration_sec=5.0)
            return
        self.info = msg

    def _image(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-stamp)/1e9
        if stamp <= self.last_stamp or age < -0.05 or age > self.image_timeout:
            return
        self.last_stamp = stamp
        if (self.info is None or msg.header.frame_id != self.optical_frame
                or self.info.header.frame_id != msg.header.frame_id
                or (msg.width, msg.height) != (self.info.width, self.info.height)):
            self.get_logger().warn('Waiting for matching image/CameraInfo/optical frame', throttle_duration_sec=5.0)
            return
        try:
            tf = self.buffer.lookup_transform(self.base_frame, msg.header.frame_id,
                                              rclpy.time.Time.from_msg(msg.header.stamp))
            tr, qr = tf.transform.translation, tf.transform.rotation
            base_from_camera = transform([tr.x, tr.y, tr.z], [qr.x, qr.y, qr.z, qr.w])
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            k = np.asarray(self.info.k).reshape(3, 3)
            d = np.asarray(self.info.d)
            estimate = self.estimator.estimate(image, k, d)
            if estimate is not None:
                camera_from_marker, corners, error = estimate
                if np.linalg.norm(camera_from_marker[:3, 3]) <= self.max_distance:
                    matrix = base_from_camera @ camera_from_marker
                    pose = PoseStamped()
                    pose.header.stamp, pose.header.frame_id = msg.header.stamp, self.base_frame
                    pos, quat = pose.pose.position, pose.pose.orientation
                    pos.x, pos.y, pos.z = (float(v) for v in matrix[:3, 3])
                    quat.x, quat.y, quat.z, quat.w = (float(v) for v in matrix_quaternion(matrix[:3, :3]))
                    self.pose_pub.publish(pose)
                    if self.debug:
                        cv2.aruco.drawDetectedMarkers(image, [corners])
                        cv2.putText(image, f'reprojection {error:.2f}px', (15, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if self.debug:
                debug = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                debug.header = msg.header
                self.debug_pub.publish(debug)
                compressed_period_ns = int(1e9 / self.debug_compressed_rate)
                if (self.debug_compressed_pub.get_subscription_count() > 0
                        and stamp - self.last_compressed_stamp >= compressed_period_ns):
                    encoded, jpeg = cv2.imencode(
                        '.jpg', image,
                        [cv2.IMWRITE_JPEG_QUALITY, self.debug_jpeg_quality],
                    )
                    if not encoded:
                        raise RuntimeError('Could not JPEG-encode the docking debug image')
                    compressed = CompressedImage()
                    compressed.header = msg.header
                    compressed.format = 'jpeg'
                    compressed.data = jpeg.tobytes()
                    self.debug_compressed_pub.publish(compressed)
                    self.last_compressed_stamp = stamp
        except TransformException as exc:
            self.get_logger().warn(f'Waiting for camera TF: {exc}', throttle_duration_sec=5.0)
        except (ValueError, cv2.error, RuntimeError, CvBridgeError) as exc:
            self.get_logger().warn(f'Rejected image: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ArucoDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
