#!/usr/bin/env python3

import argparse
import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from obstacle_tracking.msg import Track2DArray
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rotate_to_map(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    return (
        math.cos(yaw) * vx - math.sin(yaw) * vy,
        math.sin(yaw) * vx + math.cos(yaw) * vy,
    )


def rotate_to_base(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    return (
        math.cos(yaw) * vx + math.sin(yaw) * vy,
        -math.sin(yaw) * vx + math.cos(yaw) * vy,
    )


class PerceptionValidator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("perception_output_validator")
        self.args = args
        self.track_ids = set(args.track_id)
        self.ego: Odometry | None = None
        self.imu_hist: deque[Imu] = deque(maxlen=200)
        self.vel_hist: deque[Odometry] = deque(maxlen=200)
        self.tf_hist: deque[TransformStamped] = deque(maxlen=400)
        self.sample_count = 0

        self.create_subscription(Track2DArray, args.obstacles_topic, self.on_tracks, 20)
        self.create_subscription(Odometry, args.ego_topic, self.on_ego, 20)
        self.create_subscription(Odometry, args.vel_topic, self.on_vel, 50)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 50)
        self.create_subscription(TFMessage, args.tf_topic, self.on_tf, 50)

        self.get_logger().info(
            "Waiting for obstacles=%s ego=%s vel=%s imu=%s tf=%s"
            % (
                args.obstacles_topic,
                args.ego_topic,
                args.vel_topic,
                args.imu_topic,
                args.tf_topic,
            )
        )

    def on_ego(self, msg: Odometry) -> None:
        self.ego = msg

    def on_vel(self, msg: Odometry) -> None:
        self.vel_hist.append(msg)

    def on_imu(self, msg: Imu) -> None:
        self.imu_hist.append(msg)

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.args.map_frame
                and transform.child_frame_id == self.args.base_frame
            ):
                self.tf_hist.append(transform)

    def pick_best(self, hist: deque, target_t: float):
        best = None
        best_dt = float("inf")
        for msg in hist:
            msg_t = stamp_to_sec(msg.header.stamp)
            dt = abs(target_t - msg_t)
            if dt < best_dt:
                best_dt = dt
                best = msg
        return best, best_dt

    def on_tracks(self, msg: Track2DArray) -> None:
        if self.ego is None:
            self.get_logger().warn("No ego_state received yet.")
            return

        target_t = stamp_to_sec(msg.header.stamp)
        vel_msg, vel_dt = self.pick_best(self.vel_hist, target_t)
        imu_msg, imu_dt = self.pick_best(self.imu_hist, target_t)
        tf_msg, tf_dt = self.pick_best(self.tf_hist, target_t)

        ego = self.ego
        ego_t = stamp_to_sec(ego.header.stamp)
        ego_yaw = yaw_from_quaternion(ego.pose.pose.orientation)
        ego_vx = ego.twist.twist.linear.x
        ego_vy = ego.twist.twist.linear.y

        print(
            f"\n[sample {self.sample_count + 1}] obstacles_t={target_t:.3f} "
            f"ego_t={ego_t:.3f} dt(obs-ego)={target_t - ego_t:+.3f}s n={len(msg.tracks)}"
        )
        print(
            f"  ego_state: map_pose=({ego.pose.pose.position.x:+.3f},{ego.pose.pose.position.y:+.3f}) "
            f"map_v=({ego_vx:+.3f},{ego_vy:+.3f}) speed={math.hypot(ego_vx, ego_vy):.3f} "
            f"yaw={math.degrees(ego_yaw):+.1f}deg wz={ego.twist.twist.angular.z:+.3f}"
        )

        if tf_msg is not None:
            tf_yaw = yaw_from_quaternion(tf_msg.transform.rotation)
            tf_px = tf_msg.transform.translation.x
            tf_py = tf_msg.transform.translation.y
            pose_err = math.hypot(ego.pose.pose.position.x - tf_px, ego.pose.pose.position.y - tf_py)
            yaw_err_deg = math.degrees(normalize_angle(ego_yaw - tf_yaw))
            print(
                f"  map->base TF: t={stamp_to_sec(tf_msg.header.stamp):.3f} dt(obs-tf)={target_t - stamp_to_sec(tf_msg.header.stamp):+.3f}s "
                f"pose_err={pose_err:.3f} yaw_err={yaw_err_deg:+.2f}deg"
            )
        else:
            print("  map->base TF: missing")

        if vel_msg is not None:
            vel_body_vx = vel_msg.twist.twist.linear.x
            vel_body_vy = vel_msg.twist.twist.linear.y
            inferred_map_vx, inferred_map_vy = rotate_to_map(vel_body_vx, vel_body_vy, ego_yaw)
            vel_err_vx = ego_vx - inferred_map_vx
            vel_err_vy = ego_vy - inferred_map_vy
            print(
                f"  /vel: t={stamp_to_sec(vel_msg.header.stamp):.3f} dt(obs-vel)={target_t - stamp_to_sec(vel_msg.header.stamp):+.3f}s "
                f"body_v=({vel_body_vx:+.3f},{vel_body_vy:+.3f}) inferred_map=({inferred_map_vx:+.3f},{inferred_map_vy:+.3f}) "
                f"ego_err=({vel_err_vx:+.3f},{vel_err_vy:+.3f}) err_speed={math.hypot(vel_err_vx, vel_err_vy):.3f}"
            )
        else:
            print("  /vel: missing")

        if imu_msg is not None:
            imu_yaw = yaw_from_quaternion(imu_msg.orientation)
            print(
                f"  /imu/data: t={stamp_to_sec(imu_msg.header.stamp):.3f} dt(obs-imu)={target_t - stamp_to_sec(imu_msg.header.stamp):+.3f}s "
                f"yaw={math.degrees(imu_yaw):+.1f}deg yaw_err_vs_ego={math.degrees(normalize_angle(ego_yaw - imu_yaw)):+.2f}deg"
            )
        else:
            print("  /imu/data: missing")

        for track in msg.tracks:
            if self.track_ids and track.track_id not in self.track_ids:
                continue

            expected_pose_x = (
                math.cos(ego_yaw) * (track.map_pose.position.x - ego.pose.pose.position.x)
                + math.sin(ego_yaw) * (track.map_pose.position.y - ego.pose.pose.position.y)
            )
            expected_pose_y = (
                -math.sin(ego_yaw) * (track.map_pose.position.x - ego.pose.pose.position.x)
                + math.cos(ego_yaw) * (track.map_pose.position.y - ego.pose.pose.position.y)
            )
            pose_err_x = track.pose.position.x - expected_pose_x
            pose_err_y = track.pose.position.y - expected_pose_y
            pose_err = math.hypot(pose_err_x, pose_err_y)

            expected_rel_vx, expected_rel_vy = rotate_to_base(
                track.map_twist.linear.x - ego_vx,
                track.map_twist.linear.y - ego_vy,
                ego_yaw,
            )
            vel_err_x = track.twist.linear.x - expected_rel_vx
            vel_err_y = track.twist.linear.y - expected_rel_vy
            vel_err = math.hypot(vel_err_x, vel_err_y)

            pose_yaw = yaw_from_quaternion(track.pose.orientation)
            map_pose_yaw = yaw_from_quaternion(track.map_pose.orientation)
            expected_pose_yaw = normalize_angle(map_pose_yaw - ego_yaw)
            yaw_err_deg = math.degrees(normalize_angle(pose_yaw - expected_pose_yaw))

            print(
                f"  id={track.track_id} age={track.age} hits={track.hits} conf={track.confidence:.2f} "
                f"size=({track.length:.2f},{track.width:.2f})"
            )
            print(
                f"    pose=({track.pose.position.x:+.3f},{track.pose.position.y:+.3f}) "
                f"expected_pose=({expected_pose_x:+.3f},{expected_pose_y:+.3f}) "
                f"pose_err=({pose_err_x:+.3f},{pose_err_y:+.3f}) err_norm={pose_err:.3f}"
            )
            print(
                f"    rel_v=({track.twist.linear.x:+.3f},{track.twist.linear.y:+.3f}) "
                f"expected_rel_v=({expected_rel_vx:+.3f},{expected_rel_vy:+.3f}) "
                f"vel_err=({vel_err_x:+.3f},{vel_err_y:+.3f}) err_norm={vel_err:.3f}"
            )
            print(
                f"    yaw pose={math.degrees(pose_yaw):+.1f}deg map_pose={math.degrees(map_pose_yaw):+.1f}deg "
                f"expected_pose_yaw={math.degrees(expected_pose_yaw):+.1f}deg yaw_err={yaw_err_deg:+.2f}deg"
            )

        self.sample_count += 1
        if self.sample_count >= self.args.max_samples:
            raise SystemExit(0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--obstacles-topic", default="/perception/obstacles")
    parser.add_argument("--ego-topic", default="/localization/ego_state")
    parser.add_argument("--vel-topic", default="/vel")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--track-id", type=int, nargs="*", default=[])
    parser.add_argument("--max-samples", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = PerceptionValidator(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
