#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from obstacle_tracking.msg import Track2DArray
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from tf2_msgs.msg import TFMessage


GREEN = "\033[32m"
RED = "\033[31m"
RESET = "\033[0m"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class PerceptionPipelineMonitor(Node):
    def __init__(self) -> None:
        super().__init__("perception_pipeline_monitor")

        self.declare_parameter("lidar_topic", "/velodyne_points")
        self.declare_parameter("fix_topic", "/fix")
        self.declare_parameter("vel_topic", "/vel")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("perception_obstacles_topic", "/perception/obstacles")
        self.declare_parameter("perception_ego_state_topic", "/localization/ego_state")
        self.declare_parameter("pass_dt", 0.03)
        self.declare_parameter("warn_dt", 0.08)
        self.declare_parameter("fresh_timeout_sec", 0.5)
        self.declare_parameter("log_timeout_sec", 3.5)

        self.pass_dt = float(self.get_parameter("pass_dt").value)
        self.warn_dt = float(self.get_parameter("warn_dt").value)
        self.fresh_timeout_sec = float(self.get_parameter("fresh_timeout_sec").value)
        self.log_timeout_sec = float(self.get_parameter("log_timeout_sec").value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.fix_hist: deque[NavSatFix] = deque(maxlen=400)
        self.vel_hist: deque[TwistWithCovarianceStamped] = deque(maxlen=400)
        self.imu_hist: deque[Imu] = deque(maxlen=400)
        self.tf_hist: deque[TransformStamped] = deque(maxlen=800)

        self.latest_lidar: PointCloud2 | None = None
        self.latest_perc_obstacles: Track2DArray | None = None
        self.latest_perc_ego: Odometry | None = None

        self.last_filtering_diag_wall = None
        self.last_tracking_diag_wall = None
        self.last_filtering_diag_msg = ""
        self.last_tracking_diag_msg = ""

        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("lidar_topic").value),
            self.on_lidar,
            20,
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("fix_topic").value),
            self.on_fix,
            50,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            str(self.get_parameter("vel_topic").value),
            self.on_vel,
            50,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self.on_imu,
            50,
        )
        self.create_subscription(
            TFMessage,
            str(self.get_parameter("tf_topic").value),
            self.on_tf,
            50,
        )
        self.create_subscription(
            Track2DArray,
            str(self.get_parameter("perception_obstacles_topic").value),
            self.on_perception_obstacles,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("perception_ego_state_topic").value),
            self.on_perception_ego,
            20,
        )
        self.create_subscription(Log, "/rosout", self.on_rosout, 200)

        self.create_timer(1.0, self.on_timer)

        self.get_logger().info(
            "Integrated perception monitor started. 1Hz PASS/BAD report enabled."
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
                transform.header.frame_id == self.map_frame
                and transform.child_frame_id == self.base_frame
            ):
                self.tf_hist.append(transform)

    def on_lidar(self, msg: PointCloud2) -> None:
        self.latest_lidar = msg

    def on_perception_obstacles(self, msg: Track2DArray) -> None:
        self.latest_perc_obstacles = msg

    def on_perception_ego(self, msg: Odometry) -> None:
        self.latest_perc_ego = msg

    def on_rosout(self, msg: Log) -> None:
        # obstacle_filtering periodic diagnostic contains "waypoint/fix diagnostic"
        if msg.name == "obstacle_filtering" and "waypoint/fix diagnostic" in msg.msg:
            self.last_filtering_diag_wall = self.get_clock().now()
            self.last_filtering_diag_msg = msg.msg

        # obstacle_tracking periodic diagnostics contain these substrings
        if msg.name == "obstacle_tracking_node" and (
            "Match diag" in msg.msg or "Lifecycle" in msg.msg
        ):
            self.last_tracking_diag_wall = self.get_clock().now()
            self.last_tracking_diag_msg = msg.msg

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

    def assess_dt(self, dt_abs: float | None) -> tuple[bool, str]:
        if dt_abs is None:
            return False, "missing"
        if dt_abs <= self.pass_dt:
            return True, f"PASS {dt_abs:.3f}s"
        if dt_abs <= self.warn_dt:
            return True, f"WARN {dt_abs:.3f}s"
        return False, f"BAD {dt_abs:.3f}s"

    def assess_fresh(self, stamp_sec: float | None, now_sec: float) -> tuple[bool, str]:
        if stamp_sec is None:
            return False, "missing"
        age = now_sec - stamp_sec
        if not math.isfinite(age):
            return False, "nan"
        if age <= self.fresh_timeout_sec:
            return True, f"PASS age={age:.3f}s"
        return False, f"BAD age={age:.3f}s"

    def log_state(self, ok: bool, text: str) -> str:
        tag = f"{GREEN}PASS{RESET}" if ok else f"{RED}BAD{RESET}"
        return f"{tag} {text}"

    def on_timer(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        lidar_t = (
            stamp_to_sec(self.latest_lidar.header.stamp)
            if self.latest_lidar is not None
            else None
        )

        fix_msg = self.pick_best(self.fix_hist, lidar_t) if lidar_t is not None else None
        vel_msg = self.pick_best(self.vel_hist, lidar_t) if lidar_t is not None else None
        imu_msg = self.pick_best(self.imu_hist, lidar_t) if lidar_t is not None else None
        tf_msg = self.pick_best(self.tf_hist, lidar_t) if lidar_t is not None else None

        sensor_checks = []
        sensor_ok = True
        for name, msg in [("fix", fix_msg), ("vel", vel_msg), ("imu", imu_msg), ("tf", tf_msg)]:
            if lidar_t is None or msg is None:
                ok, detail = False, "missing"
            else:
                ok, detail = self.assess_dt(abs(lidar_t - stamp_to_sec(msg.header.stamp)))
            sensor_ok = sensor_ok and ok
            sensor_checks.append(f"{name}:{detail}")

        obst_t = (
            stamp_to_sec(self.latest_perc_obstacles.header.stamp)
            if self.latest_perc_obstacles is not None
            else None
        )
        ego_t = (
            stamp_to_sec(self.latest_perc_ego.header.stamp)
            if self.latest_perc_ego is not None
            else None
        )

        if lidar_t is not None and obst_t is not None:
            obst_sync_ok, obst_sync_detail = self.assess_dt(abs(lidar_t - obst_t))
        else:
            obst_sync_ok, obst_sync_detail = False, "missing"

        if lidar_t is not None and ego_t is not None:
            ego_sync_ok, ego_sync_detail = self.assess_dt(abs(lidar_t - ego_t))
        else:
            ego_sync_ok, ego_sync_detail = False, "missing"

        obst_fresh_ok, obst_fresh_detail = self.assess_fresh(obst_t, now_sec)
        ego_fresh_ok, ego_fresh_detail = self.assess_fresh(ego_t, now_sec)
        perception_ok = (
            obst_sync_ok and ego_sync_ok and obst_fresh_ok and ego_fresh_ok
        )

        filtering_log_ok = False
        tracking_log_ok = False
        if self.last_filtering_diag_wall is not None:
            filtering_log_ok = (
                (now - self.last_filtering_diag_wall).nanoseconds / 1e9
                <= self.log_timeout_sec
            )
        if self.last_tracking_diag_wall is not None:
            tracking_log_ok = (
                (now - self.last_tracking_diag_wall).nanoseconds / 1e9
                <= self.log_timeout_sec
            )
        log_ok = filtering_log_ok and tracking_log_ok

        overall_ok = sensor_ok and perception_ok and log_ok

        summary = (
            f"PIPELINE {'PASS' if overall_ok else 'BAD'} | "
            f"sensors={'PASS' if sensor_ok else 'BAD'} [{', '.join(sensor_checks)}] | "
            f"perception={'PASS' if perception_ok else 'BAD'} "
            f"[obs_sync:{obst_sync_detail}, ego_sync:{ego_sync_detail}, "
            f"obs_fresh:{obst_fresh_detail}, ego_fresh:{ego_fresh_detail}] | "
            f"diag_logs={'PASS' if log_ok else 'BAD'} "
            f"[filtering:{'ok' if filtering_log_ok else 'timeout'}, "
            f"tracking:{'ok' if tracking_log_ok else 'timeout'}]"
        )

        if overall_ok:
            self.get_logger().info(summary)
        else:
            self.get_logger().error(summary)


def main() -> None:
    rclpy.init()
    node = PerceptionPipelineMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
