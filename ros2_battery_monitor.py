"""ROS 2 battery OCR monitor with a small dependency-free web dashboard."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from battery_monitor_core import (
    BatteryStore,
    MonitorConfig,
    crop_normalized,
    discover_camera_sources,
    recognize_percentage,
)


def _encode_jpeg(image, quality=85):
    if image is None:
        return None
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return encoded.tobytes() if ok else None


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, monitor, template: str):
        self.monitor = monitor
        self.template = template.encode("utf-8")
        super().__init__(address, DashboardHandler)


class DashboardHandler(BaseHTTPRequestHandler):
    server: DashboardServer

    def log_message(self, _format, *_args):
        return

    def _send(self, status, body=b"", content_type="application/json; charset=utf-8", headers=None):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(int(status))
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        if headers:
            for key, value in headers.items():
                self.send_header(key, value)
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, status, value):
        self._send(status, json.dumps(value, ensure_ascii=False).encode("utf-8"))

    def do_GET(self):  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self._send(HTTPStatus.OK, self.server.template, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/status":
            self._json(HTTPStatus.OK, self.server.monitor.dashboard_status())
            return
        if parsed.path == "/api/config":
            self._json(HTTPStatus.OK, self.server.monitor.get_config())
            return
        if parsed.path == "/api/history":
            query = parse_qs(parsed.query)
            try:
                hours = min(24 * 365, max(0.05, float(query.get("hours", [24])[0])))
            except ValueError:
                self._json(HTTPStatus.BAD_REQUEST, {"error": "hours must be a number"})
                return
            self._json(HTTPStatus.OK, self.server.monitor.history(hours))
            return
        if parsed.path == "/api/frame.jpg":
            image = self.server.monitor.frame_jpeg(cropped=False)
            if image is None:
                self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera frame unavailable"})
            else:
                self._send(HTTPStatus.OK, image, "image/jpeg")
            return
        if parsed.path == "/api/roi.jpg":
            image = self.server.monitor.frame_jpeg(cropped=True)
            if image is None:
                self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "camera frame unavailable"})
            else:
                self._send(HTTPStatus.OK, image, "image/jpeg")
            return
        if parsed.path == "/api/export.csv":
            query = parse_qs(parsed.query)
            try:
                hours = min(24 * 3650, max(0.05, float(query.get("hours", [24 * 30])[0])))
            except ValueError:
                self._json(HTTPStatus.BAD_REQUEST, {"error": "hours must be a number"})
                return
            body = self.server.monitor.export_csv(hours)
            self._send(
                HTTPStatus.OK,
                body,
                "text/csv; charset=utf-8",
                {"Content-Disposition": "attachment; filename=tidybot-battery.csv"},
            )
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self):  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path == "/api/scan":
            self.server.monitor.request_scan()
            self._json(HTTPStatus.ACCEPTED, {"ok": True})
            return
        if parsed.path != "/api/config":
            self._json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 2 or length > 65536:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            config = self.server.monitor.update_config(payload)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self._json(HTTPStatus.BAD_REQUEST, {"error": str(exc)})
            return
        self._json(HTTPStatus.OK, config)


class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__("tidybot_battery_monitor")
        self.declare_parameter("camera", "auto")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("sample_interval", 60.0)
        self.declare_parameter("web_bind", "0.0.0.0")
        self.declare_parameter("web_port", 8080)
        self.declare_parameter("data_dir", "")
        self.declare_parameter("low_battery_threshold", 20)
        self.declare_parameter("minimum_confidence", 35.0)

        data_dir_value = str(self.get_parameter("data_dir").value).strip()
        self.data_dir = Path(data_dir_value).expanduser() if data_dir_value else (
            Path.home() / ".ros" / "tidybot" / "battery_monitor"
        )
        self.data_dir.mkdir(parents=True, exist_ok=True)
        defaults = MonitorConfig(
            sample_interval=float(self.get_parameter("sample_interval").value),
            low_battery_threshold=int(self.get_parameter("low_battery_threshold").value),
            minimum_confidence=float(self.get_parameter("minimum_confidence").value),
        )
        self.config_path = self.data_dir / "config.json"
        self.config = MonitorConfig.load(self.config_path, defaults)
        self.store = BatteryStore(self.data_dir / "battery.sqlite3")

        self.camera_request = str(self.get_parameter("camera").value)
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self._lock = threading.RLock()
        self._frame = None
        self._frame_timestamp = 0.0
        self._camera_source = ""
        self._camera_label = ""
        self._camera_error = "尚未找到非 RealSense 的 USB 相機"
        self._camera_candidates = []
        self._last_attempt = None
        self._last_error = ""
        self._consecutive_failures = 0
        self._next_scan_time = time.time() + 2.0
        self._capture = None
        self._stop = threading.Event()
        self._scan_requested = threading.Event()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.battery_publisher = self.create_publisher(BatteryState, "/tidybot/battery", qos)
        self.percentage_publisher = self.create_publisher(
            Float32, "/tidybot/battery_percentage", qos
        )

        package_share = Path(get_package_share_directory("tidybot_ros2_control"))
        template_path = package_share / "templates" / "battery_monitor.html"
        template = template_path.read_text(encoding="utf-8")
        bind = str(self.get_parameter("web_bind").value)
        port = int(self.get_parameter("web_port").value)
        self._web_server = DashboardServer((bind, port), self, template)
        self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._sample_thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._web_thread = threading.Thread(target=self._web_server.serve_forever, daemon=True)
        self._camera_thread.start()
        self._sample_thread.start()
        self._web_thread.start()
        self.get_logger().info(f"Battery dashboard: http://localhost:{port}")
        self.get_logger().info(
            "Waiting for a non-RealSense UVC camera; OCR data is stored under " + str(self.data_dir)
        )

    def _open_camera(self, source: str):
        capture_source = int(source) if source.isdigit() else source
        capture = cv2.VideoCapture(capture_source, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            return None
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _camera_loop(self):
        while not self._stop.is_set():
            candidates = discover_camera_sources(self.camera_request)
            with self._lock:
                self._camera_candidates = [
                    {"source": source, "label": label} for source, label in candidates
                ]
            capture = None
            selected = None
            for source, label in candidates:
                capture = self._open_camera(source)
                if capture is not None:
                    selected = (source, label)
                    break
            if capture is None or selected is None:
                with self._lock:
                    self._frame = None
                    self._camera_source = ""
                    self._camera_label = ""
                    self._camera_error = "找不到可開啟的非 RealSense USB 相機"
                self._stop.wait(3.0)
                continue

            source, label = selected
            self._capture = capture
            failures = 0
            with self._lock:
                self._camera_source = source
                self._camera_label = label
                self._camera_error = ""
            self.get_logger().info(f"Battery camera connected: {label} ({source})")
            while not self._stop.is_set():
                ok, frame = capture.read()
                if ok and frame is not None and frame.size:
                    failures = 0
                    with self._lock:
                        self._frame = frame
                        self._frame_timestamp = time.time()
                else:
                    failures += 1
                    if failures >= 20:
                        break
                    self._stop.wait(0.05)
            capture.release()
            self._capture = None
            with self._lock:
                self._frame = None
                self._camera_error = "相機串流中斷，正在重新連線"
            if not self._stop.is_set():
                self.get_logger().warning("Battery camera disconnected; retrying")
                self._stop.wait(1.0)

    def _sample_loop(self):
        while not self._stop.is_set():
            with self._lock:
                wait_seconds = max(0.0, self._next_scan_time - time.time())
            requested = self._scan_requested.wait(min(wait_seconds, 1.0))
            if self._stop.is_set():
                break
            now = time.time()
            if not requested and now < self._next_scan_time:
                continue
            self._scan_requested.clear()
            self._scan_once(now)
            with self._lock:
                self._next_scan_time = time.time() + self.config.sample_interval

    def _scan_once(self, timestamp: float):
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
            source = self._camera_source
            calibrated = self.config.roi_calibrated
            config = MonitorConfig(**asdict(self.config))
        if frame is None:
            with self._lock:
                self._last_error = "沒有可用的相機畫面"
                self._last_attempt = {"timestamp": timestamp, "accepted": False, "reason": "no_camera"}
            return
        if not calibrated:
            with self._lock:
                self._last_error = "請先在網頁框選手機上的電量數字，再儲存偵測區域"
                self._last_attempt = {"timestamp": timestamp, "accepted": False, "reason": "roi_not_calibrated"}
            return

        result, _roi = recognize_percentage(frame, config)
        attempt = {
            "timestamp": timestamp,
            "value": result.value,
            "confidence": round(result.confidence, 1),
            "raw_text": result.text,
            "variants_agreeing": result.variants_agreeing,
            "accepted": result.accepted,
            "reason": result.reason,
        }
        if result.accepted and result.value is not None:
            self.store.add(timestamp, result, source)
            battery = BatteryState()
            battery.header.stamp = self.get_clock().now().to_msg()
            battery.percentage = result.value / 100.0
            battery.present = True
            self.battery_publisher.publish(battery)
            self.percentage_publisher.publish(Float32(data=float(result.value)))
            with self._lock:
                self._last_error = ""
                self._consecutive_failures = 0
            self.get_logger().info(
                f"Battery OCR: {result.value}% (confidence {result.confidence:.0f}, "
                f"{result.variants_agreeing} variants)"
            )
        else:
            with self._lock:
                self._last_error = "未能可靠辨識電量，請檢查手機亮度、對焦與偵測框"
                self._consecutive_failures += 1
            self.get_logger().warning(f"Battery OCR rejected: {result.reason}")
        with self._lock:
            self._last_attempt = attempt

    def request_scan(self):
        self._scan_requested.set()

    def get_config(self):
        with self._lock:
            return asdict(self.config)

    def update_config(self, payload):
        allowed = set(asdict(self.config))
        unknown = set(payload) - allowed
        if unknown:
            raise ValueError("unknown config fields: " + ", ".join(sorted(unknown)))
        with self._lock:
            for key, value in payload.items():
                setattr(self.config, key, value)
            self.config.validate()
            self.config.save(self.config_path)
            config = asdict(self.config)
            self._next_scan_time = min(self._next_scan_time, time.time() + 1.0)
        self._scan_requested.set()
        return config

    def dashboard_status(self):
        latest = self.store.latest()
        with self._lock:
            camera_connected = self._frame is not None and time.time() - self._frame_timestamp < 3.0
            if not camera_connected:
                state = "camera_disconnected"
            elif not self.config.roi_calibrated:
                state = "needs_calibration"
            elif self._last_error:
                state = "ocr_warning"
            elif latest is None:
                state = "waiting_for_sample"
            elif latest["percentage"] <= self.config.low_battery_threshold:
                state = "low_battery"
            else:
                state = "ok"
            return {
                "state": state,
                "camera_connected": camera_connected,
                "camera_source": self._camera_source,
                "camera_label": self._camera_label,
                "camera_error": self._camera_error,
                "camera_candidates": list(self._camera_candidates),
                "frame_timestamp": self._frame_timestamp or None,
                "latest": latest,
                "last_attempt": self._last_attempt,
                "last_error": self._last_error,
                "consecutive_failures": self._consecutive_failures,
                "next_scan_timestamp": self._next_scan_time,
                "config": asdict(self.config),
            }

    def frame_jpeg(self, cropped=False):
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
            config = MonitorConfig(**asdict(self.config))
        if frame is None:
            return None
        if cropped:
            frame = crop_normalized(frame, config)
        return _encode_jpeg(frame)

    def history(self, hours):
        return self.store.history(time.time() - float(hours) * 3600.0)

    def export_csv(self, hours):
        return self.store.csv_since(time.time() - float(hours) * 3600.0)

    def close(self):
        self._stop.set()
        self._scan_requested.set()
        capture = self._capture
        if capture is not None:
            capture.release()
        if hasattr(self, "_web_server"):
            self._web_server.shutdown()
            self._web_server.server_close()
        self._camera_thread.join(timeout=3.0)
        self._sample_thread.join(timeout=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
