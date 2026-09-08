#!/usr/bin/env python3

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class TrackStateSample:
    stamp_ns: int
    track_id: int
    base_x: float
    base_y: float
    base_z: float
    base_yaw: float
    base_vx: float
    base_vy: float
    base_yaw_rate: float
    map_x: float
    map_y: float
    map_z: float
    map_yaw: float
    map_vx: float
    map_vy: float
    map_yaw_rate: float
    length: float
    width: float
    confidence: float
    age: int
    hits: int
    missed: int


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


def build_track_samples(messages: List[Tuple[int, object]]) -> Dict[int, List[TrackStateSample]]:
    out: Dict[int, List[TrackStateSample]] = {}
    for _, msg in messages:
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        for track in msg.tracks:
            track_id = int(track.track_id)
            out.setdefault(track_id, []).append(
                TrackStateSample(
                    stamp_ns=stamp_ns,
                    track_id=track_id,
                    base_x=float(track.pose.position.x),
                    base_y=float(track.pose.position.y),
                    base_z=float(track.pose.position.z),
                    base_yaw=yaw_from_quaternion(track.pose.orientation),
                    base_vx=float(track.twist.linear.x),
                    base_vy=float(track.twist.linear.y),
                    base_yaw_rate=float(track.twist.angular.z),
                    map_x=float(track.map_pose.position.x),
                    map_y=float(track.map_pose.position.y),
                    map_z=float(track.map_pose.position.z),
                    map_yaw=yaw_from_quaternion(track.map_pose.orientation),
                    map_vx=float(track.map_twist.linear.x),
                    map_vy=float(track.map_twist.linear.y),
                    map_yaw_rate=float(track.map_twist.angular.z),
                    length=float(track.length),
                    width=float(track.width),
                    confidence=float(track.confidence),
                    age=int(track.age),
                    hits=int(track.hits),
                    missed=int(track.missed),
                )
            )
    for rows in out.values():
        rows.sort(key=lambda item: item.stamp_ns)
    return out


def mean(values: List[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def rms(values: List[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else 0.0


def choose_track_ids(
    track_samples: Dict[int, List[TrackStateSample]],
    requested_ids: List[int],
    top_n: int,
) -> List[int]:
    if requested_ids:
        missing = [track_id for track_id in requested_ids if track_id not in track_samples]
        if missing:
            raise ValueError(f"track_ids not found in bag: {missing}")
        return requested_ids
    ranked = sorted(
        track_samples.items(),
        key=lambda item: (len(item[1]), max(sample.hits for sample in item[1])),
        reverse=True,
    )
    return [track_id for track_id, _ in ranked[:top_n]]


def filter_samples_by_time_window(
    samples: List[TrackStateSample],
    time_start_sec: Optional[float],
    time_end_sec: Optional[float],
) -> List[TrackStateSample]:
    if not samples:
        return []
    t0_ns = samples[0].stamp_ns
    out: List[TrackStateSample] = []
    for sample in samples:
        t_sec = (sample.stamp_ns - t0_ns) * 1e-9
        if time_start_sec is not None and t_sec < time_start_sec:
            continue
        if time_end_sec is not None and t_sec > time_end_sec:
            continue
        out.append(sample)
    return out


def format_event_times(times_sec: List[float], limit: int = 5) -> str:
    if not times_sec:
        return "-"
    shown = ", ".join(f"{value:.2f}s" for value in times_sec[:limit])
    if len(times_sec) > limit:
        shown += f", ... ({len(times_sec)} total)"
    return shown


def write_track_csv(csv_path: str, samples: List[TrackStateSample]) -> None:
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "stamp_ns",
            "track_id",
            "base_x", "base_y", "base_z", "base_yaw_deg",
            "base_vx", "base_vy", "base_yaw_rate_degps",
            "map_x", "map_y", "map_z", "map_yaw_deg",
            "map_vx", "map_vy", "map_yaw_rate_degps",
            "length", "width", "confidence", "age", "hits", "missed",
        ])
        for row in samples:
            writer.writerow([
                row.stamp_ns,
                row.track_id,
                f"{row.base_x:.6f}",
                f"{row.base_y:.6f}",
                f"{row.base_z:.6f}",
                f"{math.degrees(row.base_yaw):.6f}",
                f"{row.base_vx:.6f}",
                f"{row.base_vy:.6f}",
                f"{math.degrees(row.base_yaw_rate):.6f}",
                f"{row.map_x:.6f}",
                f"{row.map_y:.6f}",
                f"{row.map_z:.6f}",
                f"{math.degrees(row.map_yaw):.6f}",
                f"{row.map_vx:.6f}",
                f"{row.map_vy:.6f}",
                f"{math.degrees(row.map_yaw_rate):.6f}",
                f"{row.length:.6f}",
                f"{row.width:.6f}",
                f"{row.confidence:.6f}",
                row.age,
                row.hits,
                row.missed,
            ])


def summarize_track(
    track_id: int,
    samples: List[TrackStateSample],
    teleport_distance_m: float,
    teleport_margin_m: float,
    shrink_ratio: float,
    expected_max_speed_mps: float,
) -> List[str]:
    duration_sec = 0.0
    if len(samples) >= 2:
        duration_sec = (samples[-1].stamp_ns - samples[0].stamp_ns) * 1e-9

    base_jumps: List[float] = []
    map_jumps: List[float] = []
    dt_values: List[float] = []
    teleport_times_sec: List[float] = []
    shrink_length_times_sec: List[float] = []
    shrink_width_times_sec: List[float] = []
    zero_map_pose_count = 0
    zero_map_twist_count = 0

    t0_ns = samples[0].stamp_ns if samples else 0
    for sample in samples:
        if abs(sample.map_x) < 1e-6 and abs(sample.map_y) < 1e-6 and abs(sample.map_z) < 1e-6:
            zero_map_pose_count += 1
        if abs(sample.map_vx) < 1e-6 and abs(sample.map_vy) < 1e-6 and abs(sample.map_yaw_rate) < 1e-6:
            zero_map_twist_count += 1

    for prev, curr in zip(samples, samples[1:]):
        dt_sec = max(0.0, (curr.stamp_ns - prev.stamp_ns) * 1e-9)
        if dt_sec <= 1e-6:
            continue
        dt_values.append(dt_sec)

        base_jump = math.hypot(curr.base_x - prev.base_x, curr.base_y - prev.base_y)
        map_jump = math.hypot(curr.map_x - prev.map_x, curr.map_y - prev.map_y)
        base_jumps.append(base_jump)
        map_jumps.append(map_jump)

        predicted_move = math.hypot(prev.map_vx, prev.map_vy) * dt_sec
        if map_jump > max(teleport_distance_m, predicted_move + teleport_margin_m):
            teleport_times_sec.append((curr.stamp_ns - t0_ns) * 1e-9)

        if curr.length < prev.length * shrink_ratio:
            shrink_length_times_sec.append((curr.stamp_ns - t0_ns) * 1e-9)
        if curr.width < prev.width * shrink_ratio:
            shrink_width_times_sec.append((curr.stamp_ns - t0_ns) * 1e-9)

    map_speed_values = [math.hypot(sample.map_vx, sample.map_vy) for sample in samples]
    base_speed_values = [math.hypot(sample.base_vx, sample.base_vy) for sample in samples]
    near_zero_speed_eps = 0.05
    base_zero_count = sum(1 for value in base_speed_values if value <= near_zero_speed_eps)
    map_zero_count = sum(1 for value in map_speed_values if value <= near_zero_speed_eps)
    base_over_limit_count = sum(1 for value in base_speed_values if value > expected_max_speed_mps)
    map_over_limit_count = sum(1 for value in map_speed_values if value > expected_max_speed_mps)
    ego_vel_suspect_count = sum(
        1
        for base_speed, map_speed in zip(base_speed_values, map_speed_values)
        if abs(base_speed - map_speed) <= 0.05
    )

    lines = [
        f"track_id={track_id}: samples={len(samples)} duration={duration_sec:.2f}s "
        f"age_max={max((s.age for s in samples), default=0)} "
        f"hits_max={max((s.hits for s in samples), default=0)} "
        f"missed_max={max((s.missed for s in samples), default=0)}",
        f"  base_pose: x[{min(s.base_x for s in samples):.2f},{max(s.base_x for s in samples):.2f}] "
        f"y[{min(s.base_y for s in samples):.2f},{max(s.base_y for s in samples):.2f}] "
        f"jump_rms={rms(base_jumps):.3f}m jump_max={max(base_jumps, default=0.0):.3f}m",
        f"  map_pose: x[{min(s.map_x for s in samples):.2f},{max(s.map_x for s in samples):.2f}] "
        f"y[{min(s.map_y for s in samples):.2f},{max(s.map_y for s in samples):.2f}] "
        f"jump_rms={rms(map_jumps):.3f}m jump_max={max(map_jumps, default=0.0):.3f}m",
        f"  twist: base_speed_mean={mean(base_speed_values):.2f}m/s base_speed_max={max(base_speed_values, default=0.0):.2f}m/s "
        f"map_speed_mean={mean(map_speed_values):.2f}m/s map_speed_max={max(map_speed_values, default=0.0):.2f}m/s",
        f"  speed_health: base_zero={base_zero_count}/{len(samples)} map_zero={map_zero_count}/{len(samples)} "
        f"base_over_{expected_max_speed_mps:.1f}={base_over_limit_count}/{len(samples)} "
        f"map_over_{expected_max_speed_mps:.1f}={map_over_limit_count}/{len(samples)} "
        f"map_eq_base={ego_vel_suspect_count}/{len(samples)}",
        f"  size: length_mean={mean([s.length for s in samples]):.2f}m "
        f"length_range=[{min(s.length for s in samples):.2f},{max(s.length for s in samples):.2f}] "
        f"width_mean={mean([s.width for s in samples]):.2f}m "
        f"width_range=[{min(s.width for s in samples):.2f},{max(s.width for s in samples):.2f}]",
        f"  map_output: zero_pose={zero_map_pose_count}/{len(samples)} zero_twist={zero_map_twist_count}/{len(samples)} "
        f"mean_dt={mean(dt_values):.3f}s",
        f"  events: teleports={len(teleport_times_sec)} at {format_event_times(teleport_times_sec)}",
        f"  events: shrink_length={len(shrink_length_times_sec)} at {format_event_times(shrink_length_times_sec)}",
        f"  events: shrink_width={len(shrink_width_times_sec)} at {format_event_times(shrink_width_times_sec)}",
    ]
    return lines


def build_hints(
    samples_by_track: Dict[int, List[TrackStateSample]],
    teleport_distance_m: float,
    teleport_margin_m: float,
    shrink_ratio: float,
    expected_max_speed_mps: float,
) -> List[str]:
    hints: List[str] = []
    any_zero_map_pose = False
    any_zero_map_twist = False
    any_teleport = False
    any_shrink = False
    any_high_missed = False
    any_zero_speed = False
    any_speed_over_limit = False
    any_map_speed_equals_base = False

    for samples in samples_by_track.values():
        map_speed_values = [math.hypot(sample.map_vx, sample.map_vy) for sample in samples]
        base_speed_values = [math.hypot(sample.base_vx, sample.base_vy) for sample in samples]
        zero_pose_count = sum(
            1
            for sample in samples
            if abs(sample.map_x) < 1e-6 and abs(sample.map_y) < 1e-6 and abs(sample.map_z) < 1e-6
        )
        zero_twist_count = sum(
            1
            for sample in samples
            if abs(sample.map_vx) < 1e-6 and abs(sample.map_vy) < 1e-6 and abs(sample.map_yaw_rate) < 1e-6
        )
        any_zero_map_pose = any_zero_map_pose or zero_pose_count == len(samples)
        any_zero_map_twist = any_zero_map_twist or zero_twist_count == len(samples)
        any_high_missed = any_high_missed or max((sample.missed for sample in samples), default=0) >= 2
        any_zero_speed = any_zero_speed or any(value <= 0.05 for value in map_speed_values)
        any_speed_over_limit = any_speed_over_limit or any(value > expected_max_speed_mps for value in map_speed_values)
        any_map_speed_equals_base = any_map_speed_equals_base or any(
            abs(base_speed - map_speed) <= 0.05
            for base_speed, map_speed in zip(base_speed_values, map_speed_values)
        )

        for prev, curr in zip(samples, samples[1:]):
            dt_sec = max(0.0, (curr.stamp_ns - prev.stamp_ns) * 1e-9)
            if dt_sec <= 1e-6:
                continue
            map_jump = math.hypot(curr.map_x - prev.map_x, curr.map_y - prev.map_y)
            predicted_move = math.hypot(prev.map_vx, prev.map_vy) * dt_sec
            if map_jump > max(teleport_distance_m, predicted_move + teleport_margin_m):
                any_teleport = True
            if curr.length < prev.length * shrink_ratio or curr.width < prev.width * shrink_ratio:
                any_shrink = True

    if any_zero_map_pose:
        hints.append("Some tracks had map_pose stuck at zero. Check map->base_link TF recording and topic timing.")
    if any_zero_map_twist:
        hints.append("Some tracks had map_twist stuck at zero. Check /vel recording. Without /vel, map_twist may fall back to weaker estimates.")
    if any_zero_speed:
        hints.append("Some tracks reported near-zero speed while still alive. Check track re-creation, missed handling, and velocity initialization.")
    if any_speed_over_limit:
        hints.append(f"Some tracks exceeded the expected speed limit of {expected_max_speed_mps:.1f} m/s. Check output speed derivation and max_speed_mps.")
    if any_map_speed_equals_base:
        hints.append("map_speed frequently matched base_speed exactly. Ego /vel compensation may be missing or unusable.")
    if any_teleport:
        hints.append("Teleport-like pose jumps were detected. Check cluster center stability and point-count collapse around those times.")
    if any_shrink:
        hints.append("Large length/width shrink events were detected. Check partial point visibility, ground over-removal, and lshape fit collapse.")
    if any_high_missed:
        hints.append("missed count rose during the selected tracks. Association likely broke after detection center/size changed too much.")
    if not hints:
        hints.append("Selected tracks look internally consistent on pose/map_pose/map_twist and size continuity.")
    return hints


def ensure_output_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze tracked object state quality from obstacle_tracking bag outputs."
    )
    parser.add_argument("bag_path", help="Path to rosbag2 directory")
    parser.add_argument("--track-id", type=int, action="append", default=[], help="Track ID to inspect. Repeat for multiple ids.")
    parser.add_argument("--top-n", type=int, default=3, help="If no track-id is provided, inspect top N longest-lived tracks")
    parser.add_argument("--tracks-topic", default="/tracked/objects")
    parser.add_argument("--output-dir", default="", help="Directory for CSV outputs. Default: <bag>/tracking_state_analysis")
    parser.add_argument("--teleport-distance-m", type=float, default=1.5)
    parser.add_argument("--teleport-margin-m", type=float, default=0.8)
    parser.add_argument("--shrink-ratio", type=float, default=0.75)
    parser.add_argument("--expected-max-speed-mps", type=float, default=7.0)
    parser.add_argument("--time-start-sec", type=float, default=None, help="Analyze only samples at or after this per-track elapsed time")
    parser.add_argument("--time-end-sec", type=float, default=None, help="Analyze only samples at or before this per-track elapsed time")
    args = parser.parse_args()

    output_dir = args.output_dir or os.path.join(args.bag_path, "tracking_state_analysis")
    ensure_output_dir(output_dir)

    messages = load_bag_messages(args.bag_path, [args.tracks_topic])
    if not messages[args.tracks_topic]:
        raise RuntimeError(f"No messages found on {args.tracks_topic}")

    track_samples = build_track_samples(messages[args.tracks_topic])
    selected_track_ids = choose_track_ids(track_samples, args.track_id, args.top_n)

    print("Tracked state analysis")
    print(f"  bag_path: {args.bag_path}")
    print(f"  tracks_topic: {args.tracks_topic}")
    if not args.track_id:
        print(f"  selection: top {args.top_n} longest-lived tracks ranked by sample count, then max hits")
    print(f"  selected_track_ids: {selected_track_ids}")
    print()

    selected_samples = {}
    for track_id in selected_track_ids:
        filtered = filter_samples_by_time_window(
            track_samples[track_id],
            args.time_start_sec,
            args.time_end_sec,
        )
        if not filtered:
            raise RuntimeError(f"No samples remain for track_id={track_id} after applying the time window")
        selected_samples[track_id] = filtered

    for track_id in selected_track_ids:
        samples = selected_samples[track_id]
        for line in summarize_track(
            track_id,
            samples,
            args.teleport_distance_m,
            args.teleport_margin_m,
            args.shrink_ratio,
            args.expected_max_speed_mps,
        ):
            print(line)
        csv_path = os.path.join(output_dir, f"track_{track_id}_state.csv")
        write_track_csv(csv_path, samples)
        print(f"  csv: {csv_path}")
        print()

    print("Suggested focus")
    for hint in build_hints(
        selected_samples,
        args.teleport_distance_m,
        args.teleport_margin_m,
        args.shrink_ratio,
        args.expected_max_speed_mps,
    ):
        print(f"  - {hint}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
