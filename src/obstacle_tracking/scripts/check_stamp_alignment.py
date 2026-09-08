#!/usr/bin/env python3

import argparse
from collections import deque

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from lshape_fitting.msg import Detection2DArray
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StampAlignmentChecker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stamp_alignment_checker")
        self.args = args
        self.vel_history = deque(maxlen=500)
        self.tf_history = deque(maxlen=500)
        self.sample_count = 0

        self.create_subscription(
            TwistWithCovarianceStamped, args.vel_topic, self.on_vel, 50
        )
        self.create_subscription(
            Detection2DArray, args.detection_topic, self.on_detection, 20
        )
        self.create_subscription(TFMessage, args.tf_topic, self.on_tf, 100)

        self.get_logger().info(
            "Waiting for detection/vel/tf on "
            f"{args.detection_topic}, {args.vel_topic}, {args.tf_topic}"
        )

    def on_vel(self, msg: TwistWithCovarianceStamped) -> None:
        self.vel_history.append(msg)

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.args.map_frame
                and transform.child_frame_id == self.args.base_frame
            ):
                self.tf_history.append(transform)

    def find_best_past(self, history, target_t: float):
        best_msg = None
        best_dt = float("inf")
        for msg in history:
            msg_t = stamp_to_sec(msg.header.stamp)
            dt = target_t - msg_t
            if dt < 0.0:
                continue
            if dt < best_dt:
                best_dt = dt
                best_msg = msg
        return best_msg, best_dt

    def on_detection(self, msg: Detection2DArray) -> None:
        self.sample_count += 1
        if self.sample_count < self.args.skip_samples:
            return

        det_t = stamp_to_sec(msg.header.stamp)
        vel_msg, vel_dt = self.find_best_past(self.vel_history, det_t)
        tf_msg, tf_dt = self.find_best_past(self.tf_history, det_t)

        print("")
        print(
            f"[sample {self.sample_count}] detection_t={det_t:.3f} "
            f"frame={msg.header.frame_id} n={len(msg.detections)}"
        )

        if vel_msg is None:
            print("  /vel: missing")
        else:
            print(
                f"  /vel: t={stamp_to_sec(vel_msg.header.stamp):.3f} "
                f"dt(det-vel)={vel_dt:+.3f}s frame={vel_msg.header.frame_id}"
            )

        if tf_msg is None:
            print("  map->base TF: missing")
        else:
            print(
                f"  map->base TF: t={stamp_to_sec(tf_msg.header.stamp):.3f} "
                f"dt(det-tf)={tf_dt:+.3f}s"
            )

        if vel_msg is not None and tf_msg is not None:
            tf_vel_dt = abs(
                stamp_to_sec(tf_msg.header.stamp) - stamp_to_sec(vel_msg.header.stamp)
            )
            print(f"  abs(tf-vel)={tf_vel_dt:.3f}s")
            aligned = (
                vel_dt <= self.args.max_dt
                and tf_dt <= self.args.max_dt
                and tf_vel_dt <= self.args.max_dt
            )
            print(f"  aligned(max_dt={self.args.max_dt:.3f}s)={int(aligned)}")

        if self.args.max_samples > 0 and self.sample_count >= self.args.max_samples:
            raise SystemExit(0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check whether detection, /vel, and map->base_link TF stamps are aligned."
    )
    parser.add_argument("--detection-topic", default="/target/detections")
    parser.add_argument("--vel-topic", default="/vel")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--skip-samples", type=int, default=1)
    parser.add_argument("--max-samples", type=int, default=10)
    parser.add_argument("--max-dt", type=float, default=0.05)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = StampAlignmentChecker(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
