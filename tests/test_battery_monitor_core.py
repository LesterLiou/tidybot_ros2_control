import json
import shutil

import cv2
import numpy as np
import pytest

from battery_monitor_core import (
    BatteryStore,
    MonitorConfig,
    OCRCandidate,
    OCRResult,
    crop_normalized,
    parse_percentage,
    recognize_percentage,
    select_ocr_result,
)


@pytest.mark.parametrize("text, expected", [
    ("85%", 85), (" 7 ", 7), ("100 %", 100), ("0%", 0),
    ("battery 101%", None), ("12 34", None), ("empty", None),
])
def test_parse_percentage_requires_one_valid_value(text, expected):
    assert parse_percentage(text) == expected


def test_consensus_accepts_weak_matching_variants():
    result = select_ocr_result([
        OCRCandidate(83, 18, "83", "gray"),
        OCRCandidate(83, 22, "83%", "otsu", True),
        OCRCandidate(88, 70, "88", "noise"),
    ])
    assert result.value == 83
    assert result.accepted
    assert result.variants_agreeing == 2


def test_low_confidence_single_result_is_rejected():
    result = select_ocr_result([OCRCandidate(42, 10, "42", "gray")], 35)
    assert result.value == 42
    assert not result.accepted
    assert result.reason == "low_confidence"


def test_normalized_crop_and_config_roundtrip(tmp_path):
    path = tmp_path / "config.json"
    config = MonitorConfig(sample_interval=1, roi_x=.25, roi_y=.25,
                           roi_width=.5, roi_height=.5, roi_calibrated=True)
    config.save(path)
    saved = json.loads(path.read_text())
    assert saved["sample_interval"] == 5.0
    loaded = MonitorConfig.load(path)
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    assert crop_normalized(frame, loaded).shape == (50, 100, 3)


def test_recognition_pipeline_uses_variant_consensus():
    frame = np.full((120, 240, 3), 255, dtype=np.uint8)
    cv2.putText(frame, "76%", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 0), 5)
    result, roi = recognize_percentage(
        frame, MonitorConfig(roi_calibrated=True), lambda _: ("76%", 25)
    )
    assert result.value == 76
    assert result.accepted
    assert roi.shape == frame.shape


@pytest.mark.skipif(shutil.which("tesseract") is None, reason="Tesseract is installed in Docker")
@pytest.mark.parametrize("expected", [76, 85])
def test_real_tesseract_handles_large_percentage_glyph(expected):
    frame = np.full((180, 440, 3), 255, dtype=np.uint8)
    cv2.putText(frame, f"{expected}%", (20, 135), cv2.FONT_HERSHEY_SIMPLEX,
                3.3, (0, 0, 0), 7, cv2.LINE_AA)
    result, _ = recognize_percentage(frame, MonitorConfig(roi_calibrated=True))
    assert result.accepted
    assert result.value == expected


def test_store_only_accepts_valid_measurements(tmp_path):
    store = BatteryStore(tmp_path / "battery.sqlite3")
    accepted = OCRResult(64, 91.0, "64%", 3, True, "ok")
    store.add(1000.0, accepted, "/dev/video6")
    assert store.latest()["percentage"] == 64
    assert store.history(999.0)[0]["camera_source"] == "/dev/video6"
    assert "64" in store.csv_since(0)
    with pytest.raises(ValueError):
        store.add(1001.0, OCRResult(None, 0, "", 0, False, "no"), "camera")
