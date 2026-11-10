"""Camera discovery, OCR, persistence, and configuration for battery monitoring."""

from __future__ import annotations

import csv
import glob
import io
import json
import os
import re
import sqlite3
import subprocess
import threading
import time
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Iterable, Optional

import cv2
import numpy as np


PERCENT_RE = re.compile(r"(?<!\d)(100|[1-9]?\d)(?:\s*%)?(?!\d)")


@dataclass
class MonitorConfig:
    sample_interval: float = 60.0
    roi_x: float = 0.0
    roi_y: float = 0.0
    roi_width: float = 1.0
    roi_height: float = 1.0
    roi_calibrated: bool = False
    low_battery_threshold: int = 20
    minimum_confidence: float = 35.0

    def validate(self) -> None:
        self.sample_interval = min(86400.0, max(5.0, float(self.sample_interval)))
        self.roi_x = min(0.99, max(0.0, float(self.roi_x)))
        self.roi_y = min(0.99, max(0.0, float(self.roi_y)))
        self.roi_width = min(1.0 - self.roi_x, max(0.01, float(self.roi_width)))
        self.roi_height = min(1.0 - self.roi_y, max(0.01, float(self.roi_height)))
        self.low_battery_threshold = min(100, max(0, int(self.low_battery_threshold)))
        self.minimum_confidence = min(100.0, max(0.0, float(self.minimum_confidence)))

    @classmethod
    def load(cls, path: Path, defaults: Optional["MonitorConfig"] = None) -> "MonitorConfig":
        config = cls(**asdict(defaults)) if defaults is not None else cls()
        try:
            saved = json.loads(path.read_text(encoding="utf-8"))
            for key in asdict(config):
                if key in saved:
                    setattr(config, key, saved[key])
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError):
            # Keep safe defaults if a partially-written/manual config is malformed.
            pass
        config.validate()
        return config

    def save(self, path: Path) -> None:
        self.validate()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, path)


@dataclass(frozen=True)
class OCRCandidate:
    value: int
    confidence: float
    text: str
    variant: str
    contains_percent: bool = False


@dataclass(frozen=True)
class OCRResult:
    value: Optional[int]
    confidence: float
    text: str
    variants_agreeing: int
    accepted: bool
    reason: str


def parse_percentage(text: str) -> Optional[int]:
    """Return a percentage only when the OCR text contains one valid 0..100 value."""
    values = [int(match.group(1)) for match in PERCENT_RE.finditer(text)]
    values = [value for value in values if 0 <= value <= 100]
    unique = set(values)
    return values[0] if len(unique) == 1 and values else None


def select_ocr_result(
    candidates: Iterable[OCRCandidate], minimum_confidence: float = 35.0
) -> OCRResult:
    candidates = list(candidates)
    if not candidates:
        return OCRResult(None, 0.0, "", 0, False, "no_percentage_detected")

    counts = Counter(candidate.value for candidate in candidates)

    def score(value: int) -> tuple[int, float, bool]:
        matching = [candidate for candidate in candidates if candidate.value == value]
        best_confidence = max(candidate.confidence for candidate in matching)
        # Independent preprocessing variants agreeing is stronger evidence than
        # one over-confident thresholded image. Confidence breaks agreement ties;
        # a percent sign is only a final tie-breaker because it can join a digit.
        return len(matching), best_confidence, any(candidate.contains_percent for candidate in matching)

    best_value = max(counts, key=score)
    matching = [candidate for candidate in candidates if candidate.value == best_value]
    best = max(matching, key=lambda candidate: candidate.confidence)
    agreement = len(matching)
    confidence = min(99.0, best.confidence + 10.0 * (agreement - 1))
    accepted = agreement >= 2 or best.confidence >= minimum_confidence
    reason = "ok" if accepted else "low_confidence"
    texts = []
    for candidate in matching:
        clean = candidate.text.strip()
        if clean and clean not in texts:
            texts.append(clean)
    return OCRResult(best_value, confidence, " | ".join(texts), agreement, accepted, reason)


def crop_normalized(frame: np.ndarray, config: MonitorConfig) -> np.ndarray:
    if frame is None or frame.size == 0:
        raise ValueError("empty frame")
    height, width = frame.shape[:2]
    x0 = min(width - 1, max(0, round(config.roi_x * width)))
    y0 = min(height - 1, max(0, round(config.roi_y * height)))
    x1 = min(width, max(x0 + 1, round((config.roi_x + config.roi_width) * width)))
    y1 = min(height, max(y0 + 1, round((config.roi_y + config.roi_height) * height)))
    return frame[y0:y1, x0:x1].copy()


def _digits_before_percent(binary: np.ndarray) -> Optional[tuple[int, int, int, int]]:
    """Locate digits before a trailing percent glyph made of overlapping strokes.

    Phone fonts normally draw the slash and two circles as two or three connected
    components whose horizontal ranges overlap. Removing that cluster prevents a
    common Tesseract failure where ``76%`` becomes ``16%``.
    """
    if binary.ndim != 2:
        return None
    # Normalize to white foreground on black background for component analysis.
    foreground = 255 - binary if np.mean(binary) > 127 else binary.copy()
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(foreground, 8)
    minimum_area = max(12, int(binary.size * 0.00005))
    components = [tuple(map(int, stats[index])) for index in range(1, count)
                  if int(stats[index, cv2.CC_STAT_AREA]) >= minimum_area]
    if len(components) < 3:
        return None
    left = min(item[cv2.CC_STAT_LEFT] for item in components)
    right = max(item[cv2.CC_STAT_LEFT] + item[cv2.CC_STAT_WIDTH] for item in components)
    top = min(item[cv2.CC_STAT_TOP] for item in components)
    bottom = max(item[cv2.CC_STAT_TOP] + item[cv2.CC_STAT_HEIGHT] for item in components)
    content_width = max(1, right - left)
    percent_start = None
    for index, first in enumerate(components):
        first_left, _, first_width, _, _ = first
        first_right = first_left + first_width
        for second in components[index + 1:]:
            second_left, _, second_width, _, _ = second
            second_right = second_left + second_width
            overlap = min(first_right, second_right) - max(first_left, second_left)
            overlap_ratio = overlap / max(1, min(first_width, second_width))
            pair_left = min(first_left, second_left)
            if overlap_ratio >= 0.25 and pair_left >= left + 0.45 * content_width:
                percent_start = pair_left if percent_start is None else min(percent_start, pair_left)
    if percent_start is None or percent_start <= left:
        return None
    margin_x = max(4, int(0.04 * content_width))
    percent_gap = max(2, int(0.01 * content_width))
    margin_y = max(4, int(0.08 * max(1, bottom - top)))
    return (
        max(0, left - margin_x),
        max(0, top - margin_y),
        max(left + 1, percent_start - percent_gap),
        min(binary.shape[0], bottom + margin_y),
    )


def preprocessing_variants(roi: np.ndarray) -> list[tuple[str, np.ndarray]]:
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY) if roi.ndim == 3 else roi
    longest = max(gray.shape[:2])
    scale = max(1.0, min(5.0, 1200.0 / max(1, longest)))
    enlarged = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    enlarged = cv2.GaussianBlur(enlarged, (3, 3), 0)
    equalized = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(enlarged)
    _, otsu = cv2.threshold(equalized, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    adaptive = cv2.adaptiveThreshold(
        equalized, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 7
    )
    variants = [("gray", equalized), ("otsu", otsu), ("inverse", 255 - otsu), ("adaptive", adaptive)]
    digit_crop = _digits_before_percent(otsu)
    if digit_crop is not None:
        x0, y0, x1, y1 = digit_crop
        def padded(image: np.ndarray) -> np.ndarray:
            border = max(12, int(0.15 * image.shape[0]))
            background = 255 if np.mean(image) > 127 else 0
            return cv2.copyMakeBorder(
                image, border, border, border, border,
                cv2.BORDER_CONSTANT, value=background,
            )
        variants.extend([
            ("digits_gray", padded(equalized[y0:y1, x0:x1])),
            ("digits_otsu", padded(otsu[y0:y1, x0:x1])),
            ("digits_inverse", padded(255 - otsu[y0:y1, x0:x1])),
        ])
    return variants


def _tesseract_tsv(image: np.ndarray, timeout: float = 8.0, psm: int = 8) -> tuple[str, float]:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("OpenCV could not encode the OCR crop")
    command = [
        "tesseract", "stdin", "stdout", "--psm", str(int(psm)), "-l", "eng",
        "tsv",
    ]
    result = subprocess.run(
        command,
        input=encoded.tobytes(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    if result.returncode != 0:
        error = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"Tesseract failed: {error or result.returncode}")
    rows = csv.DictReader(io.StringIO(result.stdout.decode("utf-8", errors="replace")), delimiter="\t")
    words: list[str] = []
    confidences: list[float] = []
    for row in rows:
        text = (row.get("text") or "").strip()
        if not text:
            continue
        words.append(text)
        try:
            confidence = float(row.get("conf") or -1)
        except ValueError:
            confidence = -1
        if confidence >= 0:
            confidences.append(confidence)
    return " ".join(words), (sum(confidences) / len(confidences) if confidences else 0.0)


def recognize_percentage(
    frame: np.ndarray,
    config: MonitorConfig,
    runner: Callable[[np.ndarray], tuple[str, float]] = _tesseract_tsv,
) -> tuple[OCRResult, np.ndarray]:
    roi = crop_normalized(frame, config)
    candidates: list[OCRCandidate] = []
    errors: list[str] = []
    for name, image in preprocessing_variants(roi):
        try:
            text, confidence = runner(image)
        except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
            errors.append(str(exc))
            continue
        value = parse_percentage(text)
        if value is not None:
            candidates.append(OCRCandidate(value, confidence, text, name, "%" in text))
    result = select_ocr_result(candidates, config.minimum_confidence)
    if not candidates and errors:
        result = OCRResult(None, 0.0, "", 0, False, errors[0])
    return result, roi


def _camera_name(video_path: str) -> str:
    basename = Path(video_path).name
    name_path = Path("/sys/class/video4linux") / basename / "name"
    try:
        return name_path.read_text(encoding="utf-8").strip()
    except OSError:
        return basename


def discover_camera_sources(requested: str = "auto") -> list[tuple[str, str]]:
    """Find UVC cameras while excluding the RealSense used for docking."""
    requested = str(requested).strip()
    if requested.lower() != "auto":
        if requested.isdigit():
            return [(requested, f"video{requested}")]
        return [(requested, _camera_name(requested))]

    discovered: list[tuple[str, str]] = []
    seen_devices: set[str] = set()
    by_id_paths = sorted(glob.glob("/dev/v4l/by-id/*video-index0"))
    for path in by_id_paths:
        label = Path(path).name
        if "realsense" in label.lower():
            continue
        try:
            real_path = os.path.realpath(path)
        except OSError:
            real_path = path
        if not os.path.exists(real_path) or real_path in seen_devices:
            continue
        discovered.append((path, label))
        seen_devices.add(real_path)

    for sys_path in sorted(glob.glob("/sys/class/video4linux/video*")):
        video_name = Path(sys_path).name
        path = f"/dev/{video_name}"
        label = _camera_name(path)
        if "realsense" in label.lower():
            continue
        real_path = os.path.realpath(path)
        if not os.path.exists(real_path) or real_path in seen_devices:
            continue
        discovered.append((path, label))
        seen_devices.add(real_path)
    return discovered


class BatteryStore:
    def __init__(self, path: Path):
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        with self._connect() as connection:
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    percentage INTEGER NOT NULL CHECK(percentage BETWEEN 0 AND 100),
                    confidence REAL NOT NULL,
                    raw_text TEXT NOT NULL,
                    camera_source TEXT NOT NULL
                )
                """
            )

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        return connection

    def add(self, timestamp: float, result: OCRResult, camera_source: str) -> int:
        if result.value is None or not result.accepted:
            raise ValueError("only accepted OCR results can be stored")
        with self._lock, self._connect() as connection:
            cursor = connection.execute(
                "INSERT INTO samples(timestamp, percentage, confidence, raw_text, camera_source) "
                "VALUES (?, ?, ?, ?, ?)",
                (timestamp, result.value, result.confidence, result.text, camera_source),
            )
            return int(cursor.lastrowid)

    def latest(self) -> Optional[dict]:
        with self._lock, self._connect() as connection:
            row = connection.execute("SELECT * FROM samples ORDER BY timestamp DESC LIMIT 1").fetchone()
        return dict(row) if row else None

    def history(self, since: float, limit: int = 10000) -> list[dict]:
        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT * FROM samples WHERE timestamp >= ? ORDER BY timestamp ASC LIMIT ?",
                (since, max(1, min(100000, int(limit)))),
            ).fetchall()
        return [dict(row) for row in rows]

    def csv_since(self, since: float) -> str:
        output = io.StringIO()
        writer = csv.writer(output)
        writer.writerow(["timestamp_unix", "timestamp_iso_utc", "percentage", "confidence", "raw_text", "camera_source"])
        for row in self.history(since, limit=100000):
            iso_time = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(row["timestamp"]))
            writer.writerow([
                f'{row["timestamp"]:.3f}', iso_time, row["percentage"],
                f'{row["confidence"]:.1f}', row["raw_text"], row["camera_source"],
            ])
        return output.getvalue()
