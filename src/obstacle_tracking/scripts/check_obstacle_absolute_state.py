#!/usr/bin/env python3

import math
import sys

import rclpy
from rclpy.node import Node

from obstacle_tracking.msg import Track2DArray


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class AbsoluteStateChecker(Node):
    def __init__(self, topic_name: str) -> None:
        super().__init__("absolute_state_checker")
        self.topic_name = topic_name
        self.subscription = self.create_subscription(
            Track2DArray, topic_name, self.on_tracks, 10
        )
        self.get_logger().info(f"Waiting for one message on {topic_name} ...")

    def on_tracks(self, msg: Track2DArray) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(f"topic={self.topic_name} stamp={stamp:.3f} frame={msg.header.frame_id} tracks={len(msg.tracks)}")

        for track in msg.tracks:
            map_pose = track.map_pose
            map_twist = track.map_twist
            rel_pose = track.pose
            rel_twist = track.twist

            map_speed = math.hypot(map_twist.linear.x, map_twist.linear.y)
            rel_speed = math.hypot(rel_twist.linear.x, rel_twist.linear.y)
            map_yaw_deg = math.degrees(yaw_from_quaternion(map_pose.orientation))
            rel_yaw_deg = math.degrees(yaw_from_quaternion(rel_pose.orientation))
            has_abs_pose = any(
                abs(v) > 1e-9
                for v in (map_pose.position.x, map_pose.position.y, map_pose.position.z)
            )
            has_abs_vel = any(
                abs(v) > 1e-9
                for v in (map_twist.linear.x, map_twist.linear.y, map_twist.angular.z)
            )

            print(
                f"id={track.track_id} confirmed_hits={track.hits} missed={track.missed} conf={track.confidence:.3f}"
            )
            print(
                f"  abs_pose: x={map_pose.position.x:.3f} y={map_pose.position.y:.3f} "
                f"z={map_pose.position.z:.3f} yaw={map_yaw_deg:.1f}deg valid={has_abs_pose}"
            )
            print(
                f"  abs_vel : vx={map_twist.linear.x:.3f} vy={map_twist.linear.y:.3f} "
                f"wz={map_twist.angular.z:.3f} speed={map_speed:.3f} valid={has_abs_vel}"
            )
            print(
                f"  rel_pose: x={rel_pose.position.x:.3f} y={rel_pose.position.y:.3f} "
                f"yaw={rel_yaw_deg:.1f}deg"
            )
            print(
                f"  rel_vel : vx={rel_twist.linear.x:.3f} vy={rel_twist.linear.y:.3f} "
                f"wz={rel_twist.angular.z:.3f} speed={rel_speed:.3f}"
            )

        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    topic_name = "/tracked/objects"
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]

    node = AbsoluteStateChecker(topic_name)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
