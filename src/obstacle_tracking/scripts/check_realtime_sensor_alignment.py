#!/usr/bin/env python3

import argparse
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage


GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
RESET = "\033[0m"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RealtimeSensorAlignmentChecker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("realtime_sensor_alignment_checker")
        self.args = args
        self.sample_count = 0
        self.fix_hist: deque[NavSatFix] = deque(maxlen=400)
        self.vel_hist: deque[TwistWithCovarianceStamped] = deque(maxlen=400)
        self.imu_hist: deque[Imu] = deque(maxlen=400)
        self.tf_hist: deque[TransformStamped] = deque(maxlen=800)

        self.create_subscription(PointCloud2, args.lidar_topic, self.on_lidar, 20)
        self.create_subscription(NavSatFix, args.fix_topic, self.on_fix, 50)
        self.create_subscription(TwistWithCovarianceStamped, args.vel_topic, self.on_vel, 50)
        self.create_subscription(Imu, args.imu_topic, self.on_imu, 50)
        self.create_subscription(TFMessage, args.tf_topic, self.on_tf, 50)

        self.get_logger().info(
            "Waiting for lidar=%s fix=%s vel=%s imu=%s tf=%s"
            % (args.lidar_topic, args.fix_topic, args.vel_topic, args.imu_topic, args.tf_topic)
        )

    def on_fix(self, msg: NavSatFix) -> None:
        self.fix_hist.append(msg)

    def on_vel(self, msg: TwistWithCovarianceStamped) -> None:
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
        best_abs_dt = float("inf")
        for msg in hist:
            dt = target_t - stamp_to_sec(msg.header.stamp)
            abs_dt = abs(dt)
            if abs_dt < best_abs_dt:
                best_abs_dt = abs_dt
                best = msg
        return best

    def tag(self, abs_dt: float, missing: bool = False) -> str:
        if missing:
            return f"{RED}BAD{RESET}"
        if abs_dt <= self.args.pass_dt:
            return f"{GREEN}PASS{RESET}"
        if abs_dt <= self.args.warn_dt:
            return f"{YELLOW}WARN{RESET}"
        return f"{RED}BAD{RESET}"

    def format_item(self, name: str, target_t: float, msg, extra: str = "") -> str:
        if msg is None:
            return f"  {name}: {self.tag(0.0, missing=True)} missing"
        msg_t = stamp_to_sec(msg.header.stamp)
        dt = target_t - msg_t
        return (
            f"  {name}: {self.tag(abs(dt))} t={msg_t:.3f} "
            f"dt(lidar-{name})={dt:+.3f}s{extra}"
        )

    def on_lidar(self, msg: PointCloud2) -> None:
        lidar_t = stamp_to_sec(msg.header.stamp)
        fix_msg = self.pick_best(self.fix_hist, lidar_t)
        vel_msg = self.pick_best(self.vel_hist, lidar_t)
        imu_msg = self.pick_best(self.imu_hist, lidar_t)
        tf_msg = self.pick_best(self.tf_hist, lidar_t)

        print(
            f"\n[sample {self.sample_count + 1}] lidar_t={lidar_t:.3f} "
            f"frame={msg.header.frame_id} points={msg.width * msg.height}"
        )
        print(self.format_item("fix", lidar_t, fix_msg))
        print(
            self.format_item(
                "vel",
                lidar_t,
                vel_msg,
                "" if vel_msg is None else f" frame={vel_msg.header.frame_id}",
            )
        )
        print(
            self.format_item(
                "imu",
                lidar_t,
                imu_msg,
                "" if imu_msg is None else f" frame={imu_msg.header.frame_id}",
            )
        )
        print(self.format_item("tf", lidar_t, tf_msg))

        all_msgs = [fix_msg, vel_msg, imu_msg, tf_msg]
        all_valid = all(m is not None for m in all_msgs)
        if all_valid:
            fix_t = stamp_to_sec(fix_msg.header.stamp)
            vel_t = stamp_to_sec(vel_msg.header.stamp)
            imu_t = stamp_to_sec(imu_msg.header.stamp)
            tf_t = stamp_to_sec(tf_msg.header.stamp)
            spread = max(fix_t, vel_t, imu_t, tf_t, lidar_t) - min(fix_t, vel_t, imu_t, tf_t, lidar_t)
            spread_tag = self.tag(spread)
            print(f"  overall_spread: {spread_tag} spread={spread:.3f}s")
        else:
            print(f"  overall_spread: {RED}BAD{RESET} insufficient data")

        self.sample_count += 1
        if self.sample_count >= self.args.max_samples:
            raise SystemExit(0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-topic", default="/velodyne_points")
    parser.add_argument("--fix-topic", default="/fix")
    parser.add_argument("--vel-topic", default="/vel")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--pass-dt", type=float, default=0.03)
    parser.add_argument("--warn-dt", type=float, default=0.08)
    parser.add_argument("--max-samples", type=int, default=20)
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = RealtimeSensorAlignmentChecker(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
