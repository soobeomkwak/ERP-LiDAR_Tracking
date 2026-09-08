#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from obstacle_tracking.msg import Track2DArray


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RelativeVelocityChecker(Node):
    def __init__(self, obstacles_topic: str, ego_topic: str, track_ids: set[int]) -> None:
        super().__init__("relative_velocity_checker")
        self.latest_ego = None
        self.ego_topic = ego_topic
        self.obstacles_topic = obstacles_topic
        self.track_ids = track_ids

        self.create_subscription(Odometry, ego_topic, self.on_ego, 10)
        self.create_subscription(Track2DArray, obstacles_topic, self.on_tracks, 10)

        self.get_logger().info(
            f"Waiting for ego on {ego_topic} and tracks on {obstacles_topic} ..."
        )

    def on_ego(self, msg: Odometry) -> None:
        self.latest_ego = msg

    def on_tracks(self, msg: Track2DArray) -> None:
        if self.latest_ego is None:
            self.get_logger().warn("No ego_state received yet.")
            return

        track_t = stamp_to_sec(msg.header.stamp)
        ego_t = stamp_to_sec(self.latest_ego.header.stamp)
        dt = track_t - ego_t
        ego_yaw = yaw_from_quaternion(self.latest_ego.pose.pose.orientation)
        ego_vx = self.latest_ego.twist.twist.linear.x
        ego_vy = self.latest_ego.twist.twist.linear.y

        print(
            f"tracks_stamp={track_t:.3f} ego_stamp={ego_t:.3f} dt={dt:+.3f}s "
            f"ego_map_v=({ego_vx:.3f},{ego_vy:.3f}) ego_yaw={math.degrees(ego_yaw):.1f}deg"
        )

        cos_yaw = math.cos(ego_yaw)
        sin_yaw = math.sin(ego_yaw)

        for track in msg.tracks:
            if self.track_ids and track.track_id not in self.track_ids:
                continue

            rel_map_vx = track.map_twist.linear.x - ego_vx
            rel_map_vy = track.map_twist.linear.y - ego_vy
            expected_rel_vx = cos_yaw * rel_map_vx + sin_yaw * rel_map_vy
            expected_rel_vy = -sin_yaw * rel_map_vx + cos_yaw * rel_map_vy

            actual_rel_vx = track.twist.linear.x
            actual_rel_vy = track.twist.linear.y

            err_vx = actual_rel_vx - expected_rel_vx
            err_vy = actual_rel_vy - expected_rel_vy
            err_speed = math.hypot(err_vx, err_vy)

            print(
                f"id={track.track_id} "
                f"map_v=({track.map_twist.linear.x:.3f},{track.map_twist.linear.y:.3f}) "
                f"rel_v_actual=({actual_rel_vx:.3f},{actual_rel_vy:.3f}) "
                f"rel_v_expected=({expected_rel_vx:.3f},{expected_rel_vy:.3f}) "
                f"err=({err_vx:.3f},{err_vy:.3f}) err_speed={err_speed:.3f}"
            )

        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    obstacles_topic = "/perception/obstacles"
    ego_topic = "/localization/ego_state"
    track_ids: set[int] = set()
    if len(sys.argv) > 1:
        obstacles_topic = sys.argv[1]
    if len(sys.argv) > 2:
        ego_topic = sys.argv[2]
    if len(sys.argv) > 3:
        track_ids = {int(arg) for arg in sys.argv[3:]}

    node = RelativeVelocityChecker(obstacles_topic, ego_topic, track_ids)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
