#!/usr/bin/env python3
"""Convert a CompressedImage topic in a ROS 2 bag to an MP4 video."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a sensor_msgs/CompressedImage rosbag topic to MP4."
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--topic",
        default="/docking/debug_image/compressed",
        help="CompressedImage topic to convert",
    )
    parser.add_argument("--output", type=Path, help="Output MP4 path")
    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
        help="Output frame rate; bag timestamps are preserved by repeating frames",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.is_dir():
        raise SystemExit(f"Bag directory does not exist: {bag_path}")
    if args.fps <= 0.0:
        raise SystemExit("--fps must be greater than zero")

    output_path = (
        args.output.expanduser().resolve()
        if args.output
        else bag_path / f"{bag_path.name}.mp4"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialCompressionReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    actual_type = topic_types.get(args.topic)
    expected_type = "sensor_msgs/msg/CompressedImage"
    if actual_type is None:
        available = "\n  ".join(sorted(topic_types))
        raise SystemExit(
            f"Topic not found: {args.topic}\nAvailable topics:\n  {available}"
        )
    if actual_type != expected_type:
        raise SystemExit(
            f"Topic {args.topic} has type {actual_type}, expected {expected_type}"
        )

    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))

    writer = None
    first_timestamp = None
    last_frame = None
    next_output_time = 0.0
    source_frames = 0
    output_frames = 0
    frame_size = None
    frame_period = 1.0 / args.fps

    try:
        while reader.has_next():
            topic, serialized_data, timestamp = reader.read_next()
            if topic != args.topic:
                continue

            message = deserialize_message(serialized_data, CompressedImage)
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if frame is None:
                print(f"Warning: skipped undecodable frame at {timestamp}")
                continue

            source_frames += 1
            if first_timestamp is None:
                first_timestamp = timestamp
                height, width = frame.shape[:2]
                frame_size = (width, height)
                writer = cv2.VideoWriter(
                    str(output_path),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    args.fps,
                    frame_size,
                )
                if not writer.isOpened():
                    raise RuntimeError(
                        "OpenCV could not open an MP4 writer with the mp4v codec"
                    )
                last_frame = frame
                continue

            if frame.shape[1::-1] != frame_size:
                frame = cv2.resize(frame, frame_size, interpolation=cv2.INTER_AREA)

            elapsed = (timestamp - first_timestamp) / 1_000_000_000.0
            while next_output_time < elapsed:
                writer.write(last_frame)
                output_frames += 1
                next_output_time += frame_period
            last_frame = frame

        if writer is None:
            raise RuntimeError(f"No decodable images found on {args.topic}")

        writer.write(last_frame)
        output_frames += 1
    except Exception:
        if output_path.exists():
            output_path.unlink()
        raise
    finally:
        if writer is not None:
            writer.release()

    duration = output_frames / args.fps
    print(f"Created: {output_path}")
    print(f"Source images: {source_frames}")
    print(f"Video frames: {output_frames}")
    print(f"Frame size: {frame_size[0]}x{frame_size[1]}")
    print(f"Frame rate: {args.fps:.3f} FPS")
    print(f"Video duration: {duration:.3f} seconds")


if __name__ == "__main__":
    main()
