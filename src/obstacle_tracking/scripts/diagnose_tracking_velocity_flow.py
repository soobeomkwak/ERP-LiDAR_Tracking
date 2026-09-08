#!/usr/bin/env python3

import argparse
import math
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from obstacle_tracking.msg import Track2DArray
from rclpy.node import Node
from sensor_msgs.msg import Imu


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_vec2(x: float, y: float) -> str:
    return f"({x:+.3f}, {y:+.3f})"


class TrackingVelocityFlowDiag(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("tracking_velocity_flow_diag")
        self.args = args
        self.latest_vel: Optional[TwistWithCovarianceStamped] = None
        self.latest_ego: Optional[Odometry] = None
        self.latest_imu: Optional[Imu] = None
        self.vel_history = deque(maxlen=400)
        self.ego_history = deque(maxlen=400)
        self.imu_history = deque(maxlen=400)
        self.sample_count = 0

        self.create_subscription(
            TwistWithCovarianceStamped, args.vel_topic, self.on_vel, 20
        )
        self.create_subscription(Odometry, args.ego_topic, self.on_ego, 20)
        self.create_subscription(Track2DArray, args.track_topic, self.on_tracks, 20)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 20)

        self.get_logger().info(
            "Waiting for messages on "
            f"vel={args.vel_topic}, ego={args.ego_topic}, tracks={args.track_topic}, imu={args.imu_topic}"
        )

    def on_vel(self, msg: TwistWithCovarianceStamped) -> None:
        self.latest_vel = msg
        self.vel_history.append(msg)

    def on_ego(self, msg: Odometry) -> None:
        self.latest_ego = msg
        self.ego_history.append(msg)

    def on_imu(self, msg: Imu) -> None:
        self.latest_imu = msg
        self.imu_history.append(msg)

    def find_best_msg(self, history, track_t: float, prefer_past: bool):
        best_msg = None
        best_score = float("inf")
        for msg in history:
            msg_t = stamp_to_sec(msg.header.stamp)
            dt = track_t - msg_t
            if prefer_past and dt < 0.0:
                continue
            score = abs(dt)
            if score < best_score:
                best_score = score
                best_msg = msg
        return best_msg

    def on_tracks(self, msg: Track2DArray) -> None:
        self.sample_count += 1
        if self.sample_count < self.args.skip_samples:
            return

        track_t = stamp_to_sec(msg.header.stamp)
        print("")
        print(f"[sample {self.sample_count}] tracks t={track_t:.3f} frame={msg.header.frame_id} n={len(msg.tracks)}")

        vel_msg = self.find_best_msg(self.vel_history, track_t, prefer_past=True)
        vel_body = None
        if vel_msg is None:
            print("  /vel: missing")
        else:
            vel_t = stamp_to_sec(vel_msg.header.stamp)
            vel_age = track_t - vel_t
            vel_vx = float(vel_msg.twist.twist.linear.x)
            vel_vy = float(vel_msg.twist.twist.linear.y)
            vel_body = (vel_vx, vel_vy)
            print(
                "  /vel: "
                f"t={vel_t:.3f} dt(track-vel)={vel_age:+.3f}s frame={vel_msg.header.frame_id} "
                f"body_v={fmt_vec2(vel_vx, vel_vy)} speed={math.hypot(vel_vx, vel_vy):.3f}"
            )
            future_vel_msg = self.find_best_msg(self.vel_history, track_t, prefer_past=False)
            if future_vel_msg is not None:
                future_dt = track_t - stamp_to_sec(future_vel_msg.header.stamp)
                print(f"  /vel nearest(abs): dt(track-vel_nearest)={future_dt:+.3f}s")

        ego_map = None
        ego_yaw = None
        ego_msg = self.find_best_msg(self.ego_history, track_t, prefer_past=False)
        if ego_msg is None:
            print("  /localization/ego_state: missing")
        else:
            ego_t = stamp_to_sec(ego_msg.header.stamp)
            ego_age = track_t - ego_t
            ego_vx = float(ego_msg.twist.twist.linear.x)
            ego_vy = float(ego_msg.twist.twist.linear.y)
            ego_map = (ego_vx, ego_vy)
            ego_yaw = yaw_from_quaternion(ego_msg.pose.pose.orientation)
            print(
                "  /localization/ego_state: "
                f"t={ego_t:.3f} dt(track-ego)={ego_age:+.3f}s "
                f"map_v={fmt_vec2(ego_vx, ego_vy)} speed={math.hypot(ego_vx, ego_vy):.3f} "
                f"yaw={math.degrees(ego_yaw):+.1f}deg child={ego_msg.child_frame_id}"
            )

        imu_msg = self.find_best_msg(self.imu_history, track_t, prefer_past=False)
        if imu_msg is None:
            print("  /imu/data: missing")
        else:
            imu_t = stamp_to_sec(imu_msg.header.stamp)
            imu_yaw = yaw_from_quaternion(imu_msg.orientation)
            print(
                "  /imu/data: "
                f"t={imu_t:.3f} dt(track-imu)={track_t - imu_t:+.3f}s "
                f"yaw={math.degrees(imu_yaw):+.1f}deg wz={imu_msg.angular_velocity.z:+.3f}"
            )

        if vel_body is not None and ego_yaw is not None:
            cos_y = math.cos(ego_yaw)
            sin_y = math.sin(ego_yaw)
            vel_map_x = cos_y * vel_body[0] - sin_y * vel_body[1]
            vel_map_y = sin_y * vel_body[0] + cos_y * vel_body[1]
            print(
                "  inferred ego_map_from_/vel: "
                f"{fmt_vec2(vel_map_x, vel_map_y)} speed={math.hypot(vel_map_x, vel_map_y):.3f}"
            )
            if ego_map is not None:
                err_x = ego_map[0] - vel_map_x
                err_y = ego_map[1] - vel_map_y
                print(
                    "  ego_state_vs_/vel_map error: "
                    f"{fmt_vec2(err_x, err_y)} err_speed={math.hypot(err_x, err_y):.3f}"
                )

        filtered_tracks = msg.tracks
        if self.args.track_id:
            allowed = set(self.args.track_id)
            filtered_tracks = [track for track in msg.tracks if track.track_id in allowed]

        if not filtered_tracks:
            print("  tracks: no matching track ids")
        for track in filtered_tracks:
            map_vx = float(track.map_twist.linear.x)
            map_vy = float(track.map_twist.linear.y)
            rel_vx = float(track.twist.linear.x)
            rel_vy = float(track.twist.linear.y)
            print(
                f"  track {track.track_id}: "
                f"pose=({track.pose.position.x:+.2f},{track.pose.position.y:+.2f}) "
                f"map_v={fmt_vec2(map_vx, map_vy)} rel_v={fmt_vec2(rel_vx, rel_vy)}"
            )

            if ego_map is None or ego_yaw is None:
                continue

            rel_map_vx = map_vx - ego_map[0]
            rel_map_vy = map_vy - ego_map[1]
            cos_y = math.cos(ego_yaw)
            sin_y = math.sin(ego_yaw)
            expected_rel_vx = cos_y * rel_map_vx + sin_y * rel_map_vy
            expected_rel_vy = -sin_y * rel_map_vx + cos_y * rel_map_vy
            err_vx = rel_vx - expected_rel_vx
            err_vy = rel_vy - expected_rel_vy

            print(
                "    expected rel_v from ego_state/map_twist: "
                f"{fmt_vec2(expected_rel_vx, expected_rel_vy)} "
                f"err={fmt_vec2(err_vx, err_vy)} err_speed={math.hypot(err_vx, err_vy):.3f}"
            )

        if self.args.once:
            raise SystemExit(0)
        if self.args.max_samples > 0 and self.sample_count >= self.args.max_samples:
            raise SystemExit(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Diagnose how /vel, /localization/ego_state, and track velocities line up."
    )
    parser.add_argument("--vel-topic", default="/vel")
    parser.add_argument("--ego-topic", default="/localization/ego_state")
    parser.add_argument("--track-topic", default="/tracked/objects")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--track-id", nargs="*", type=int, default=[])
    parser.add_argument("--skip-samples", type=int, default=1)
    parser.add_argument("--max-samples", type=int, default=5)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = TrackingVelocityFlowDiag(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
