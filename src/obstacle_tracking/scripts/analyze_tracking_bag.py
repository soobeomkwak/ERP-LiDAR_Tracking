#!/usr/bin/env python3

import argparse
import csv
import math
import os
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def normalize_yaw(yaw: float) -> float:
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    return yaw


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class DetectionSample:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    length: float
    width: float
    confidence: float


@dataclass
class TrackSample:
    stamp_ns: int
    track_id: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    yaw_rate: float
    length: float
    width: float
    confidence: float
    age: int
    hits: int
    missed: int


@dataclass
class ResidualSample:
    stamp_ns: int
    dt_sec: float
    corner: bool
    track_x: float
    track_y: float
    track_yaw: float
    track_speed: float
    track_yaw_rate: float
    det_x: float
    det_y: float
    det_yaw: float
    ex: float
    ey: float
    eyaw: float
    pos_err: float
    nearest_distance: float


def load_bag_messages(
    bag_path: str,
    topics: List[str],
) -> Dict[str, List[Tuple[int, object]]]:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    message_classes = {topic: get_message(type_map[topic]) for topic in topics if topic in type_map}

    out: Dict[str, List[Tuple[int, object]]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        if topic_name not in message_classes:
            continue
        msg = deserialize_message(data, message_classes[topic_name])
        out[topic_name].append((timestamp, msg))
    return out


def build_detection_index(messages: List[Tuple[int, object]]) -> Dict[int, List[DetectionSample]]:
    out: Dict[int, List[DetectionSample]] = defaultdict(list)
    for _, msg in messages:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for det in msg.detections:
            out[stamp_ns].append(
                DetectionSample(
                    stamp_ns=stamp_ns,
                    x=float(det.pose.position.x),
                    y=float(det.pose.position.y),
                    yaw=yaw_from_quaternion(det.pose.orientation),
                    length=float(det.length),
                    width=float(det.width),
                    confidence=float(det.confidence),
                )
            )
    return out


def build_track_samples(messages: List[Tuple[int, object]]) -> Dict[int, List[TrackSample]]:
    out: Dict[int, List[TrackSample]] = defaultdict(list)
    for _, msg in messages:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for track in msg.tracks:
            out[int(track.track_id)].append(
                TrackSample(
                    stamp_ns=stamp_ns,
                    track_id=int(track.track_id),
                    x=float(track.pose.position.x),
                    y=float(track.pose.position.y),
                    yaw=yaw_from_quaternion(track.pose.orientation),
                    vx=float(track.twist.linear.x),
                    vy=float(track.twist.linear.y),
                    yaw_rate=float(track.twist.angular.z),
                    length=float(track.length),
                    width=float(track.width),
                    confidence=float(track.confidence),
                    age=int(track.age),
                    hits=int(track.hits),
                    missed=int(track.missed),
                )
            )
    return out


def build_debug_residuals(messages: List[Tuple[int, object]]) -> Dict[int, List[ResidualSample]]:
    out: Dict[int, List[ResidualSample]] = defaultdict(list)
    for _, msg in messages:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for entry in msg.tracks:
            if not entry.matched:
                continue
            out[int(entry.track_id)].append(
                ResidualSample(
                    stamp_ns=stamp_ns,
                    dt_sec=0.0,
                    corner=abs(float(entry.predicted_yaw_rate)) >= math.radians(10.0),
                    track_x=float(entry.predicted_pose.position.x),
                    track_y=float(entry.predicted_pose.position.y),
                    track_yaw=yaw_from_quaternion(entry.predicted_pose.orientation),
                    track_speed=float(entry.predicted_speed),
                    track_yaw_rate=float(entry.predicted_yaw_rate),
                    det_x=float(entry.measurement_pose.position.x),
                    det_y=float(entry.measurement_pose.position.y),
                    det_yaw=yaw_from_quaternion(entry.measurement_pose.orientation),
                    ex=float(entry.residual_x),
                    ey=float(entry.residual_y),
                    eyaw=float(entry.residual_yaw),
                    pos_err=math.hypot(float(entry.residual_x), float(entry.residual_y)),
                    nearest_distance=float(entry.innovation_distance),
                )
            )
    for track_id, rows in out.items():
        rows.sort(key=lambda item: item.stamp_ns)
        prev_stamp_ns: Optional[int] = None
        for row in rows:
            row.dt_sec = 0.0 if prev_stamp_ns is None else max(0.0, (row.stamp_ns - prev_stamp_ns) * 1e-9)
            prev_stamp_ns = row.stamp_ns
    return out


def choose_track_ids(
    track_samples: Dict[int, List[TrackSample]],
    requested_track_id: Optional[int],
    top_n: int,
) -> List[int]:
    if requested_track_id is not None:
        if requested_track_id not in track_samples:
            raise ValueError(f"track_id {requested_track_id} not found in bag")
        return [requested_track_id]
    ranked = sorted(
        track_samples.items(),
        key=lambda item: (len(item[1]), max(sample.hits for sample in item[1])),
        reverse=True,
    )
    return [track_id for track_id, _ in ranked[:top_n]]


def find_nearest_detection(
    track: TrackSample,
    detections_by_stamp: Dict[int, List[DetectionSample]],
) -> Optional[DetectionSample]:
    detections = detections_by_stamp.get(track.stamp_ns, [])
    if not detections:
        return None
    return min(detections, key=lambda det: math.hypot(det.x - track.x, det.y - track.y))


def build_residuals(
    samples: List[TrackSample],
    detections_by_stamp: Dict[int, List[DetectionSample]],
    corner_yaw_rate_threshold: float,
) -> List[ResidualSample]:
    residuals: List[ResidualSample] = []
    prev_stamp_ns: Optional[int] = None
    for sample in samples:
        det = find_nearest_detection(sample, detections_by_stamp)
        if det is None:
            continue
        dt_sec = 0.0 if prev_stamp_ns is None else max(0.0, (sample.stamp_ns - prev_stamp_ns) * 1e-9)
        prev_stamp_ns = sample.stamp_ns
        ex = det.x - sample.x
        ey = det.y - sample.y
        eyaw = normalize_yaw(det.yaw - sample.yaw)
        pos_err = math.hypot(ex, ey)
        residuals.append(
            ResidualSample(
                stamp_ns=sample.stamp_ns,
                dt_sec=dt_sec,
                corner=abs(sample.yaw_rate) >= corner_yaw_rate_threshold,
                track_x=sample.x,
                track_y=sample.y,
                track_yaw=sample.yaw,
                track_speed=math.hypot(sample.vx, sample.vy),
                track_yaw_rate=sample.yaw_rate,
                det_x=det.x,
                det_y=det.y,
                det_yaw=det.yaw,
                ex=ex,
                ey=ey,
                eyaw=eyaw,
                pos_err=pos_err,
                nearest_distance=math.hypot(det.x - sample.x, det.y - sample.y),
            )
        )
    return residuals


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def rms(values: List[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def summarize_track(
    track_id: int,
    samples: List[TrackSample],
    residuals: List[ResidualSample],
) -> List[str]:
    corner = [sample for sample in residuals if sample.corner]
    straight = [sample for sample in residuals if not sample.corner]

    def block(name: str, rows: List[ResidualSample]) -> str:
        return (
            f"{name}: n={len(rows)} "
            f"rms_pos={rms([r.pos_err for r in rows]):.3f}m "
            f"rms_yaw={math.degrees(rms([r.eyaw for r in rows])):.2f}deg "
            f"mean_speed={mean([r.track_speed for r in rows]):.2f}m/s "
            f"mean_yaw_rate={math.degrees(mean([abs(r.track_yaw_rate) for r in rows])):.2f}deg/s"
        )

    missed_counter = Counter(sample.missed for sample in samples)
    lines = [
        f"track_id={track_id}: samples={len(samples)} residuals={len(residuals)} "
        f"age_max={max((s.age for s in samples), default=0)} "
        f"hits_max={max((s.hits for s in samples), default=0)} "
        f"missed_max={max((s.missed for s in samples), default=0)}",
        block("  straight", straight),
        block("  corner", corner),
        "  missed_hist=" + ", ".join(f"{miss}:{count}" for miss, count in sorted(missed_counter.items())[:8]),
    ]
    return lines


def tuning_hints(residuals: List[ResidualSample]) -> List[str]:
    if not residuals:
        return ["No residuals available. Check that /target/detections and /tracked/objects were both recorded."]

    corner = [r for r in residuals if r.corner]
    straight = [r for r in residuals if not r.corner]
    hints: List[str] = []

    corner_pos_rms = rms([r.pos_err for r in corner])
    straight_pos_rms = rms([r.pos_err for r in straight])
    corner_yaw_rms = math.degrees(rms([r.eyaw for r in corner]))
    straight_yaw_rms = math.degrees(rms([r.eyaw for r in straight]))

    if corner and straight and corner_pos_rms > max(0.4, straight_pos_rms * 1.5):
        hints.append(
            "Corner residual is much larger than straight residual. "
            "Try increasing process_noise_speed/process_noise_yaw_rate/process_noise_accel."
        )
    if corner_yaw_rms > 15.0:
        hints.append(
            "Corner yaw residual is high. Consider increasing measurement_noise_yaw "
            "or lowering yaw_measurement_alpha."
        )
    if straight_pos_rms > 0.5:
        hints.append(
            "Straight residual is already large. Check cluster stability first before tuning motion model."
        )
    if not hints:
        hints.append(
            "Residuals look reasonably bounded. Start with small parameter sweeps around the current noise values."
        )
    return hints


def write_track_csv(csv_path: str, residuals: List[ResidualSample]) -> None:
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "stamp_ns", "dt_sec", "corner", "track_x", "track_y", "track_yaw_deg",
            "track_speed_mps", "track_yaw_rate_degps", "det_x", "det_y", "det_yaw_deg",
            "ex_m", "ey_m", "eyaw_deg", "pos_err_m",
        ])
        for row in residuals:
            writer.writerow([
                row.stamp_ns,
                f"{row.dt_sec:.6f}",
                int(row.corner),
                f"{row.track_x:.6f}",
                f"{row.track_y:.6f}",
                f"{math.degrees(row.track_yaw):.6f}",
                f"{row.track_speed:.6f}",
                f"{math.degrees(row.track_yaw_rate):.6f}",
                f"{row.det_x:.6f}",
                f"{row.det_y:.6f}",
                f"{math.degrees(row.det_yaw):.6f}",
                f"{row.ex:.6f}",
                f"{row.ey:.6f}",
                f"{math.degrees(row.eyaw):.6f}",
                f"{row.pos_err:.6f}",
            ])


def ensure_output_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze obstacle_tracking bag residuals and lifecycle for field tuning."
    )
    parser.add_argument("bag_path", help="Path to rosbag2 directory")
    parser.add_argument("--track-id", type=int, default=None, help="Specific track_id to inspect")
    parser.add_argument("--top-n", type=int, default=3, help="If track-id is omitted, inspect top N longest-lived tracks")
    parser.add_argument("--detections-topic", default="/target/detections")
    parser.add_argument("--tracks-topic", default="/tracked/objects")
    parser.add_argument("--debug-topic", default="/tracked/debug")
    parser.add_argument("--corner-yaw-rate-degps", type=float, default=10.0)
    parser.add_argument("--output-dir", default="", help="Directory for CSV outputs. Default: <bag>/tracking_analysis")
    args = parser.parse_args()

    output_dir = args.output_dir or os.path.join(args.bag_path, "tracking_analysis")
    ensure_output_dir(output_dir)

    messages = load_bag_messages(args.bag_path, [args.detections_topic, args.tracks_topic, args.debug_topic])
    if not messages[args.detections_topic]:
        raise RuntimeError(f"No messages found on {args.detections_topic}")
    if not messages[args.tracks_topic]:
        raise RuntimeError(f"No messages found on {args.tracks_topic}")

    detections_by_stamp = build_detection_index(messages[args.detections_topic])
    track_samples = build_track_samples(messages[args.tracks_topic])
    debug_residuals = build_debug_residuals(messages[args.debug_topic]) if messages[args.debug_topic] else {}
    selected_track_ids = choose_track_ids(track_samples, args.track_id, args.top_n)

    print("Tracking bag analysis")
    print(f"  bag_path: {args.bag_path}")
    print(f"  detections_topic: {args.detections_topic}")
    print(f"  tracks_topic: {args.tracks_topic}")
    if args.track_id is None:
        print(f"  selection: top {args.top_n} longest-lived tracks ranked by sample count, then max hits")
    print(f"  selected_track_ids: {selected_track_ids}")
    print()

    all_residuals: List[ResidualSample] = []
    for track_id in selected_track_ids:
        samples = sorted(track_samples[track_id], key=lambda item: item.stamp_ns)
        residuals = debug_residuals.get(track_id)
        if residuals:
            for row in residuals:
                row.corner = abs(row.track_yaw_rate) >= math.radians(args.corner_yaw_rate_degps)
        else:
            residuals = build_residuals(
                samples,
                detections_by_stamp,
                math.radians(args.corner_yaw_rate_degps),
            )
        all_residuals.extend(residuals)
        for line in summarize_track(track_id, samples, residuals):
            print(line)
        csv_path = os.path.join(output_dir, f"track_{track_id}_residuals.csv")
        write_track_csv(csv_path, residuals)
        print(f"  csv: {csv_path}")
        print()

    print("Suggested tuning focus")
    for hint in tuning_hints(all_residuals):
        print(f"  - {hint}")

    print()
    print("Interpretation")
    if messages[args.debug_topic]:
        print("  - Used exact tracker debug topic for pre-update predicted state residuals.")
    else:
        print("  - This tool used nearest detection to each published track at the same stamp.")
        print("  - It is a residual proxy, not the exact pre-update EKF innovation.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
