#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "lshape_fitting/msg/detection2_d.hpp"
#include "lshape_fitting/msg/detection2_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

namespace
{
// Pipeline:
//   /obstacle_in cluster cloud
//     -> group points by cluster_id
//     -> fit one oriented 2D box per cluster
//     -> publish /target/detections plus RViz markers.
//
// The tracker receives /target/detections, so this file is the first place to
// inspect when boxes split, yaw flips, or target dimensions look wrong.
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 0.5 * kPi;
constexpr double kDegToRad = kPi / 180.0;

struct FieldMeta
{
  int offset{-1};
  uint8_t datatype{0};
};

struct PointXYZ
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ClusterData
{
  std::vector<PointXYZ> points;
  double z_min{std::numeric_limits<double>::infinity()};
  double z_max{-std::numeric_limits<double>::infinity()};
};

struct FitResult
{
  bool valid{false};
  double theta{0.0};
  double c1_min{0.0};
  double c1_max{0.0};
  double c2_min{0.0};
  double c2_max{0.0};
  double score{-std::numeric_limits<double>::infinity()};
  double closeness{0.0};
  double variance{0.0};
  double area{0.0};
};

struct BoxResult
{
  int32_t cluster_id{0};
  size_t point_count{0};
  double cx{0.0};
  double cy{0.0};
  double cz{0.0};
  double yaw{0.0};
  double length{0.0};
  double width{0.0};
  double height{0.0};
  double heading_confidence{0.0};
  bool used_pca_heading{false};
  std::array<PointXYZ, 4> corners{};
};

struct PreviousBoxState
{
  double cx{0.0};
  double cy{0.0};
  double yaw{0.0};
  double length{0.0};
  double width{0.0};
  size_t point_count{0};
  double heading_confidence{0.0};
  bool used_pca_heading{false};
};

struct PcaOrientation
{
  bool valid{false};
  double yaw{0.0};
  double anisotropy_ratio{1.0};
};

// PointCloud2 field helpers. Keep these small and local because the input cloud
// can come from different clustering implementations with slightly different
// field datatypes.
FieldMeta find_field(const std::vector<sensor_msgs::msg::PointField> & fields, const std::string & name)
{
  FieldMeta meta;
  for (const auto & f : fields) {
    if (f.name == name) {
      meta.offset = static_cast<int>(f.offset);
      meta.datatype = f.datatype;
      return meta;
    }
  }
  return meta;
}

bool read_float_field(const uint8_t * point_ptr, const FieldMeta & meta, float & out)
{
  if (meta.offset < 0) {
    return false;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    std::memcpy(&out, point_ptr + meta.offset, sizeof(float));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    double tmp = 0.0;
    std::memcpy(&tmp, point_ptr + meta.offset, sizeof(double));
    out = static_cast<float>(tmp);
    return true;
  }
  return false;
}

bool read_int32_field(const uint8_t * point_ptr, const FieldMeta & meta, int32_t & out)
{
  if (meta.offset < 0) {
    return false;
  }
  switch (meta.datatype) {
    case sensor_msgs::msg::PointField::INT8:
    {
      int8_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(tmp);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT8:
    {
      uint8_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(tmp);
      return true;
    }
    case sensor_msgs::msg::PointField::INT16:
    {
      int16_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(tmp);
      return true;
    }
    case sensor_msgs::msg::PointField::UINT16:
    {
      uint16_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(tmp);
      return true;
    }
    case sensor_msgs::msg::PointField::INT32:
    {
      int32_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = tmp;
      return true;
    }
    case sensor_msgs::msg::PointField::UINT32:
    {
      uint32_t tmp = 0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(tmp);
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT32:
    {
      float tmp = 0.0F;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(std::llround(static_cast<double>(tmp)));
      return true;
    }
    case sensor_msgs::msg::PointField::FLOAT64:
    {
      double tmp = 0.0;
      std::memcpy(&tmp, point_ptr + meta.offset, sizeof(tmp));
      out = static_cast<int32_t>(std::llround(tmp));
      return true;
    }
    default:
      return false;
  }
}

double normalize_yaw(double yaw)
{
  while (yaw > kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

double force_positive_x_heading(double yaw)
{
  yaw = normalize_yaw(yaw);
  if (std::cos(yaw) < 0.0) {
    yaw = normalize_yaw(yaw + kPi);
  }
  return yaw;
}

double choose_yaw_closest_to_reference(double yaw, double reference_yaw)
{
  const double candidate_a = normalize_yaw(yaw);
  const double candidate_b = normalize_yaw(yaw + kPi);
  const double error_a = std::abs(normalize_yaw(candidate_a - reference_yaw));
  const double error_b = std::abs(normalize_yaw(candidate_b - reference_yaw));
  return error_a <= error_b ? candidate_a : candidate_b;
}

double sample_variance(const std::vector<double> & values)
{
  if (values.size() < 2U) {
    return 0.0;
  }
  double mean = 0.0;
  for (const auto v : values) {
    mean += v;
  }
  mean /= static_cast<double>(values.size());

  double accum = 0.0;
  for (const auto v : values) {
    const double dv = v - mean;
    accum += dv * dv;
  }
  return accum / static_cast<double>(values.size());
}

double clamp01(double v)
{
  return std::max(0.0, std::min(1.0, v));
}

std::string format_fixed(double value, int precision = 2)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

PcaOrientation compute_pca_orientation(const std::vector<PointXYZ> & points)
{
  PcaOrientation result;
  if (points.size() < 2U) {
    return result;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto & point : points) {
    mean_x += point.x;
    mean_y += point.y;
  }
  mean_x /= static_cast<double>(points.size());
  mean_y /= static_cast<double>(points.size());

  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
  for (const auto & point : points) {
    const double dx = point.x - mean_x;
    const double dy = point.y - mean_y;
    cov_xx += dx * dx;
    cov_xy += dx * dy;
    cov_yy += dy * dy;
  }
  const double inv_n = 1.0 / static_cast<double>(points.size());
  cov_xx *= inv_n;
  cov_xy *= inv_n;
  cov_yy *= inv_n;

  const double trace = cov_xx + cov_yy;
  const double det_term = std::sqrt(std::max(0.0, 0.25 * (cov_xx - cov_yy) * (cov_xx - cov_yy) + cov_xy * cov_xy));
  const double lambda_max = 0.5 * trace + det_term;
  const double lambda_min = std::max(1e-9, 0.5 * trace - det_term);

  double vx = cov_xy;
  double vy = lambda_max - cov_xx;
  if (std::abs(vx) < 1e-9 && std::abs(vy) < 1e-9) {
    vx = 1.0;
    vy = 0.0;
  }
  const double norm = std::hypot(vx, vy);
  vx /= norm;
  vy /= norm;

  result.valid = std::isfinite(vx) && std::isfinite(vy);
  result.yaw = force_positive_x_heading(std::atan2(vy, vx));
  result.anisotropy_ratio = lambda_max / lambda_min;
  return result;
}
}  // namespace

class LShapeFittingNode : public rclcpp::Node
{
public:
  explicit LShapeFittingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lshape_fitting_node", options)
  {
    declare_parameter<std::string>("input_topic", "/pointcloud/clustered");
    declare_parameter<std::string>("marker_topic", "/target/markers");
    declare_parameter<std::string>("pose_vis_topic", "/target/poses_vis");
    declare_parameter<std::string>("detection_topic", "/target/detections");
    declare_parameter<std::string>("cluster_field", "cluster_id");
    declare_parameter<int>("queue_size", 1);

    declare_parameter<int>("min_cluster_points", 10);
    declare_parameter<int>("far_min_cluster_points", 6);
    declare_parameter<double>("far_min_cluster_points_range_m", 15.0);
    declare_parameter<int>("max_cluster_points", 5000);
    declare_parameter<double>("delta_theta_deg", 1.0);
    declare_parameter<double>("d0", 0.1);
    declare_parameter<std::string>("score_mode", "closeness");
    declare_parameter<double>("closeness_weight", 1.0);
    declare_parameter<double>("variance_weight", 1.0);

    declare_parameter<double>("min_length", 0.2);
    declare_parameter<double>("max_length", 5.0);
    declare_parameter<double>("min_width", 0.15);
    declare_parameter<double>("max_width", 2.4);
    declare_parameter<double>("min_height", 0.05);
    declare_parameter<double>("max_height", 3.0);
    declare_parameter<double>("max_aspect_ratio", 8.0);
    declare_parameter<bool>("reject_low_flat_clusters", true);
    declare_parameter<double>("low_flat_max_height_m", 0.22);
    declare_parameter<double>("low_flat_max_center_z_m", -0.35);
    declare_parameter<double>("low_flat_min_length_m", 0.6);

    declare_parameter<double>("marker_alpha", 0.45);
    declare_parameter<double>("marker_min_height", 0.2);
    declare_parameter<bool>("use_fixed_output_height", true);
    declare_parameter<double>("fixed_output_height_m", 1.3);
    declare_parameter<double>("fixed_output_base_z_m", 0.0);
    declare_parameter<double>("outline_width", 0.05);
    declare_parameter<bool>("publish_outline", true);
    declare_parameter<bool>("publish_text_markers", true);
    declare_parameter<bool>("publish_ego_marker", true);
    declare_parameter<double>("ego_marker_x_m", 0.20);
    declare_parameter<double>("ego_marker_y_m", 0.0);
    declare_parameter<double>("ego_marker_yaw_deg", 0.0);
    declare_parameter<double>("ego_marker_length_m", 2.60);
    declare_parameter<double>("ego_marker_width_m", 1.50);
    declare_parameter<double>("ego_marker_height_m", 1.10);
    declare_parameter<double>("ego_marker_base_z_m", -0.67);

    declare_parameter<bool>("use_pca_heading", true);
    declare_parameter<double>("pca_min_anisotropy_ratio", 1.15);
    declare_parameter<bool>("force_heading_positive_x", true);
    declare_parameter<bool>("enable_temporal_yaw_stabilization", true);
    declare_parameter<double>("temporal_yaw_match_distance_m", 1.5);
    declare_parameter<double>("temporal_yaw_size_weight", 0.5);
    declare_parameter<double>("temporal_yaw_alpha", 0.35);
    declare_parameter<double>("temporal_position_alpha", 0.45);
    declare_parameter<double>("temporal_size_alpha", 0.35);
    declare_parameter<double>("temporal_motion_heading_alpha", 0.50);
    declare_parameter<double>("temporal_motion_heading_min_distance_m", 0.20);
    declare_parameter<double>("yaw_freeze_low_confidence_threshold", 0.45);
    declare_parameter<double>("yaw_freeze_point_count_drop_ratio", 0.55);
    declare_parameter<double>("yaw_freeze_size_jump_m", 0.90);
    declare_parameter<double>("yaw_freeze_large_error_rad", 0.70);
    declare_parameter<std::string>("output_frame_id", "");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("align_output_stamp_to_tf", true);
    declare_parameter<double>("output_stamp_tf_tolerance_sec", 0.05);
    declare_parameter<double>("input_offset_x_m", 0.0);
    declare_parameter<double>("input_offset_y_m", 0.0);
    declare_parameter<double>("input_offset_z_m", 0.0);

    declare_parameter<bool>("fit_vehicle_spec_only", true);
    declare_parameter<bool>("use_vehicle_size_prior", true);
    declare_parameter<bool>("use_vehicle_height_gate", true);
    declare_parameter<bool>("require_wheel_track_consistency", true);
    declare_parameter<double>("vehicle_size_weight", 25.0);
    declare_parameter<double>("vehicle_length_m", 2.060);
    declare_parameter<double>("vehicle_width_m", 1.160);
    declare_parameter<double>("vehicle_height_m", 0.822);
    declare_parameter<double>("vehicle_ground_clearance_m", 0.160);
    declare_parameter<double>("vehicle_track_front_m", 0.970);
    declare_parameter<double>("vehicle_track_rear_m", 0.938);
    declare_parameter<double>("vehicle_wheelbase_m", 1.212);
    declare_parameter<double>("vehicle_length_tolerance_m", 0.800);
    declare_parameter<double>("vehicle_width_tolerance_m", 0.500);
    declare_parameter<double>("vehicle_height_tolerance_m", 0.800);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();
    const auto pose_vis_topic = get_parameter("pose_vis_topic").as_string();
    const auto detection_topic = get_parameter("detection_topic").as_string();
    cluster_field_ = get_parameter("cluster_field").as_string();
    const int queue_size = get_parameter("queue_size").as_int();

    min_cluster_points_ = get_parameter("min_cluster_points").as_int();
    far_min_cluster_points_ = get_parameter("far_min_cluster_points").as_int();
    far_min_cluster_points_range_m_ = get_parameter("far_min_cluster_points_range_m").as_double();
    max_cluster_points_ = get_parameter("max_cluster_points").as_int();
    delta_theta_deg_ = get_parameter("delta_theta_deg").as_double();
    d0_ = get_parameter("d0").as_double();
    score_mode_ = get_parameter("score_mode").as_string();
    closeness_weight_ = get_parameter("closeness_weight").as_double();
    variance_weight_ = get_parameter("variance_weight").as_double();

    min_length_ = get_parameter("min_length").as_double();
    max_length_ = get_parameter("max_length").as_double();
    min_width_ = get_parameter("min_width").as_double();
    max_width_ = get_parameter("max_width").as_double();
    min_height_ = get_parameter("min_height").as_double();
    max_height_ = get_parameter("max_height").as_double();
    max_aspect_ratio_ = get_parameter("max_aspect_ratio").as_double();
    reject_low_flat_clusters_ = get_parameter("reject_low_flat_clusters").as_bool();
    low_flat_max_height_m_ = get_parameter("low_flat_max_height_m").as_double();
    low_flat_max_center_z_m_ = get_parameter("low_flat_max_center_z_m").as_double();
    low_flat_min_length_m_ = get_parameter("low_flat_min_length_m").as_double();

    marker_alpha_ = get_parameter("marker_alpha").as_double();
    marker_min_height_ = get_parameter("marker_min_height").as_double();
    use_fixed_output_height_ = get_parameter("use_fixed_output_height").as_bool();
    fixed_output_height_m_ = get_parameter("fixed_output_height_m").as_double();
    fixed_output_base_z_m_ = get_parameter("fixed_output_base_z_m").as_double();
    outline_width_ = get_parameter("outline_width").as_double();
    publish_outline_ = get_parameter("publish_outline").as_bool();
    publish_text_markers_ = get_parameter("publish_text_markers").as_bool();
    publish_ego_marker_ = get_parameter("publish_ego_marker").as_bool();
    ego_marker_x_m_ = get_parameter("ego_marker_x_m").as_double();
    ego_marker_y_m_ = get_parameter("ego_marker_y_m").as_double();
    ego_marker_yaw_deg_ = get_parameter("ego_marker_yaw_deg").as_double();
    ego_marker_length_m_ = get_parameter("ego_marker_length_m").as_double();
    ego_marker_width_m_ = get_parameter("ego_marker_width_m").as_double();
    ego_marker_height_m_ = get_parameter("ego_marker_height_m").as_double();
    ego_marker_base_z_m_ = get_parameter("ego_marker_base_z_m").as_double();

    use_pca_heading_ = get_parameter("use_pca_heading").as_bool();
    pca_min_anisotropy_ratio_ = get_parameter("pca_min_anisotropy_ratio").as_double();
    force_heading_positive_x_ = get_parameter("force_heading_positive_x").as_bool();
    enable_temporal_yaw_stabilization_ = get_parameter("enable_temporal_yaw_stabilization").as_bool();
    temporal_yaw_match_distance_m_ = get_parameter("temporal_yaw_match_distance_m").as_double();
    temporal_yaw_size_weight_ = get_parameter("temporal_yaw_size_weight").as_double();
    temporal_yaw_alpha_ = get_parameter("temporal_yaw_alpha").as_double();
    temporal_position_alpha_ = get_parameter("temporal_position_alpha").as_double();
    temporal_size_alpha_ = get_parameter("temporal_size_alpha").as_double();
    temporal_motion_heading_alpha_ = get_parameter("temporal_motion_heading_alpha").as_double();
    temporal_motion_heading_min_distance_m_ =
      get_parameter("temporal_motion_heading_min_distance_m").as_double();
    yaw_freeze_low_confidence_threshold_ =
      get_parameter("yaw_freeze_low_confidence_threshold").as_double();
    yaw_freeze_point_count_drop_ratio_ =
      get_parameter("yaw_freeze_point_count_drop_ratio").as_double();
    yaw_freeze_size_jump_m_ = get_parameter("yaw_freeze_size_jump_m").as_double();
    yaw_freeze_large_error_rad_ = get_parameter("yaw_freeze_large_error_rad").as_double();
    output_frame_id_ = get_parameter("output_frame_id").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    align_output_stamp_to_tf_ = get_parameter("align_output_stamp_to_tf").as_bool();
    output_stamp_tf_tolerance_sec_ =
      std::max(0.0, get_parameter("output_stamp_tf_tolerance_sec").as_double());
    input_offset_x_m_ = get_parameter("input_offset_x_m").as_double();
    input_offset_y_m_ = get_parameter("input_offset_y_m").as_double();
    input_offset_z_m_ = get_parameter("input_offset_z_m").as_double();

    fit_vehicle_spec_only_ = get_parameter("fit_vehicle_spec_only").as_bool();
    use_vehicle_size_prior_ = get_parameter("use_vehicle_size_prior").as_bool();
    use_vehicle_height_gate_ = get_parameter("use_vehicle_height_gate").as_bool();
    require_wheel_track_consistency_ = get_parameter("require_wheel_track_consistency").as_bool();
    vehicle_size_weight_ = get_parameter("vehicle_size_weight").as_double();
    vehicle_length_m_ = get_parameter("vehicle_length_m").as_double();
    vehicle_width_m_ = get_parameter("vehicle_width_m").as_double();
    vehicle_height_m_ = get_parameter("vehicle_height_m").as_double();
    vehicle_ground_clearance_m_ = get_parameter("vehicle_ground_clearance_m").as_double();
    vehicle_track_front_m_ = get_parameter("vehicle_track_front_m").as_double();
    vehicle_track_rear_m_ = get_parameter("vehicle_track_rear_m").as_double();
    vehicle_wheelbase_m_ = get_parameter("vehicle_wheelbase_m").as_double();
    vehicle_length_tolerance_m_ = get_parameter("vehicle_length_tolerance_m").as_double();
    vehicle_width_tolerance_m_ = get_parameter("vehicle_width_tolerance_m").as_double();
    vehicle_height_tolerance_m_ = get_parameter("vehicle_height_tolerance_m").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const bool known_mode = (score_mode_ == "closeness" || score_mode_ == "variance" ||
      score_mode_ == "area" || score_mode_ == "hybrid");
    if (!known_mode) {
      RCLCPP_WARN(
        get_logger(),
        "unknown score_mode '%s', fallback to 'closeness'",
        score_mode_.c_str());
      score_mode_ = "closeness";
    }

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::QoS(queue_size),
      std::bind(&LShapeFittingNode::cloud_callback, this, _1));

    if (!marker_topic.empty()) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(10));
    }
    if (!pose_vis_topic.empty()) {
      pose_vis_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(pose_vis_topic, rclcpp::QoS(10));
    }
    detection_pub_ = create_publisher<lshape_fitting::msg::Detection2DArray>(detection_topic, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "lshape_fitting started in:%s markers:%s poses_vis:%s detections:%s field:%s mode:%s dtheta:%.2f",
      input_topic.c_str(), marker_topic.c_str(), pose_vis_topic.c_str(),
      detection_topic.c_str(), cluster_field_.c_str(), score_mode_.c_str(), delta_theta_deg_);
  }

private:
  // ----- Fit scoring and output-time helpers --------------------------------

  double calculate_score(double closeness, double variance, double area) const
  {
    if (score_mode_ == "variance") {
      return variance;
    }
    if (score_mode_ == "area") {
      return area;
    }
    if (score_mode_ == "hybrid") {
      return closeness_weight_ * closeness + variance_weight_ * variance;
    }
    return closeness;
  }

  void align_output_header_stamp(std_msgs::msg::Header & header) const
  {
    if (!align_output_stamp_to_tf_ || !tf_buffer_) {
      return;
    }
    if (header.frame_id != base_frame_) {
      return;
    }
    try {
      const auto latest_tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.02));
      const double dt = std::abs((rclcpp::Time(header.stamp) - rclcpp::Time(latest_tf.header.stamp)).seconds());
      if (dt <= output_stamp_tf_tolerance_sec_) {
        header.stamp = latest_tf.header.stamp;
      }
    } catch (const tf2::TransformException &) {
    }
  }

  double vehicle_size_prior(double length, double width) const
  {
    const double len_tol = std::max(vehicle_length_tolerance_m_, 1e-3);
    const double wid_tol = std::max(vehicle_width_tolerance_m_, 1e-3);
    const double len_err = (length - vehicle_length_m_) / len_tol;
    const double wid_err = (width - vehicle_width_m_) / wid_tol;
    return -(len_err * len_err + wid_err * wid_err);
  }

  bool passes_vehicle_spec_gate(double length, double width, double height) const
  {
    if (!fit_vehicle_spec_only_) {
      return true;
    }

    const double min_plausible_height = std::max(0.05, 0.5 * vehicle_ground_clearance_m_);
    if (height < min_plausible_height) {
      return false;
    }

    const bool len_ok = std::abs(length - vehicle_length_m_) <= vehicle_length_tolerance_m_;
    const bool wid_ok = std::abs(width - vehicle_width_m_) <= vehicle_width_tolerance_m_;
    bool hgt_ok = true;
    if (use_vehicle_height_gate_) {
      hgt_ok = std::abs(height - vehicle_height_m_) <= vehicle_height_tolerance_m_;
    }

    if (!(len_ok && wid_ok && hgt_ok)) {
      return false;
    }

    if (require_wheel_track_consistency_) {
      const double max_track = std::max(vehicle_track_front_m_, vehicle_track_rear_m_);
      if (length + 1e-3 < vehicle_wheelbase_m_) {
        return false;
      }
      if (width + 1e-3 < 0.8 * max_track) {
        return false;
      }
    }
    return true;
  }

  // Sweep candidate yaw angles and keep the rectangle with the best L-shape
  // score. Size prior is optional but useful when sparse far points create
  // many plausible rectangles.
  FitResult search_best_fit(const std::vector<PointXYZ> & points) const
  {
    FitResult best;
    if (points.size() < 2U) {
      return best;
    }

    const double delta_deg = std::max(0.1, delta_theta_deg_);
    const double delta_theta = delta_deg * kDegToRad;
    const double d0 = std::max(d0_, 1e-4);

    for (double theta = 0.0; theta < kHalfPi; theta += delta_theta) {
      const double cos_t = std::cos(theta);
      const double sin_t = std::sin(theta);

      std::vector<double> c1;
      std::vector<double> c2;
      c1.reserve(points.size());
      c2.reserve(points.size());

      double c1_min = std::numeric_limits<double>::infinity();
      double c1_max = -std::numeric_limits<double>::infinity();
      double c2_min = std::numeric_limits<double>::infinity();
      double c2_max = -std::numeric_limits<double>::infinity();

      for (const auto & p : points) {
        const double proj1 = p.x * cos_t + p.y * sin_t;
        const double proj2 = -p.x * sin_t + p.y * cos_t;
        c1.push_back(proj1);
        c2.push_back(proj2);
        c1_min = std::min(c1_min, proj1);
        c1_max = std::max(c1_max, proj1);
        c2_min = std::min(c2_min, proj2);
        c2_max = std::max(c2_max, proj2);
      }

      const double extent1 = c1_max - c1_min;
      const double extent2 = c2_max - c2_min;
      if (extent1 < 1e-4 || extent2 < 1e-4) {
        continue;
      }

      double closeness = 0.0;
      std::vector<double> e1;
      std::vector<double> e2;
      e1.reserve(points.size());
      e2.reserve(points.size());

      for (size_t i = 0; i < c1.size(); ++i) {
        const double d1 = std::min(c1_max - c1[i], c1[i] - c1_min);
        const double d2 = std::min(c2_max - c2[i], c2[i] - c2_min);
        const double d = std::max(std::min(d1, d2), d0);
        closeness += 1.0 / d;

        if (d1 <= d2) {
          e1.push_back(d1);
        } else {
          e2.push_back(d2);
        }
      }

      const double variance = -(
        (e1.empty() ? 0.0 : sample_variance(e1)) +
        (e2.empty() ? 0.0 : sample_variance(e2))
      );
      const double area = -(extent1 * extent2);
      double score = calculate_score(closeness, variance, area);
      if (use_vehicle_size_prior_) {
        const double cand_length = std::max(extent1, extent2);
        const double cand_width = std::min(extent1, extent2);
        score += vehicle_size_weight_ * vehicle_size_prior(cand_length, cand_width);
      }
      if (!std::isfinite(score)) {
        continue;
      }

      if (!best.valid || score > best.score) {
        best.valid = true;
        best.theta = theta;
        best.c1_min = c1_min;
        best.c1_max = c1_max;
        best.c2_min = c2_min;
        best.c2_max = c2_max;
        best.score = score;
        best.closeness = closeness;
        best.variance = variance;
        best.area = area;
      }
    }
    return best;
  }

  // Reuse a known yaw, but recompute extents from the current points. PCA uses
  // this path when the cluster is elongated enough to trust its major axis.
  FitResult project_box_for_theta(const std::vector<PointXYZ> & points, double theta) const
  {
    FitResult fit;
    if (points.size() < 2U) {
      return fit;
    }

    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    fit.valid = true;
    fit.theta = theta;
    fit.c1_min = std::numeric_limits<double>::infinity();
    fit.c1_max = -std::numeric_limits<double>::infinity();
    fit.c2_min = std::numeric_limits<double>::infinity();
    fit.c2_max = -std::numeric_limits<double>::infinity();

    for (const auto & p : points) {
      const double proj1 = p.x * cos_t + p.y * sin_t;
      const double proj2 = -p.x * sin_t + p.y * cos_t;
      fit.c1_min = std::min(fit.c1_min, proj1);
      fit.c1_max = std::max(fit.c1_max, proj1);
      fit.c2_min = std::min(fit.c2_min, proj2);
      fit.c2_max = std::max(fit.c2_max, proj2);
    }
    return fit;
  }

  int required_min_points_for_range(double range_m) const
  {
    if (range_m >= far_min_cluster_points_range_m_) {
      return std::max(1, std::min(min_cluster_points_, far_min_cluster_points_));
    }
    return min_cluster_points_;
  }

  static double cluster_centroid_range(const ClusterData & cluster)
  {
    if (cluster.points.empty()) {
      return 0.0;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto & point : cluster.points) {
      sum_x += point.x;
      sum_y += point.y;
    }

    const double inv_n = 1.0 / static_cast<double>(cluster.points.size());
    return std::hypot(sum_x * inv_n, sum_y * inv_n);
  }

  // Main per-cluster fitting gate: point count -> L-shape/PCA fit -> dimension
  // checks -> vehicle-spec checks. If a target never reaches the tracker, this
  // is the most useful function to debug first.
  bool fit_cluster_to_box(int32_t cluster_id, const ClusterData & cluster, BoxResult & box) const
  {
    const double cluster_range_m = cluster_centroid_range(cluster);
    const int required_min_points = required_min_points_for_range(cluster_range_m);
    if (cluster.points.size() < static_cast<size_t>(required_min_points) ||
      cluster.points.size() > static_cast<size_t>(max_cluster_points_))
    {
      return false;
    }

    const FitResult lshape_fit = search_best_fit(cluster.points);
    if (!lshape_fit.valid) {
      return false;
    }

    FitResult fit = lshape_fit;
    bool used_pca_heading = false;
    double heading_confidence = 0.0;

    const PcaOrientation pca = compute_pca_orientation(cluster.points);
    if (use_pca_heading_ && pca.valid && pca.anisotropy_ratio >= pca_min_anisotropy_ratio_) {
      fit = project_box_for_theta(cluster.points, pca.yaw);
      if (!fit.valid) {
        return false;
      }
      used_pca_heading = true;
      heading_confidence = clamp01((pca.anisotropy_ratio - pca_min_anisotropy_ratio_) / 4.0);
    }

    const double cos_t = std::cos(fit.theta);
    const double sin_t = std::sin(fit.theta);

    auto from_projection = [&](double c1, double c2) {
        PointXYZ p;
        p.x = c1 * cos_t + c2 * (-sin_t);
        p.y = c1 * sin_t + c2 * cos_t;
        p.z = 0.0;
        return p;
      };

    const PointXYZ p1 = from_projection(fit.c1_min, fit.c2_min);
    const PointXYZ p2 = from_projection(fit.c1_max, fit.c2_min);
    const PointXYZ p3 = from_projection(fit.c1_max, fit.c2_max);
    const PointXYZ p4 = from_projection(fit.c1_min, fit.c2_max);

    double length = std::hypot(p2.x - p1.x, p2.y - p1.y);
    double width = std::hypot(p3.x - p2.x, p3.y - p2.y);
    double yaw = fit.theta;
    if (width > length) {
      std::swap(length, width);
      yaw = fit.theta + kHalfPi;
    }
    yaw = normalize_yaw(yaw);
    if (force_heading_positive_x_) {
      yaw = force_positive_x_heading(yaw);
    }

    const double height = std::max(0.0, cluster.z_max - cluster.z_min);
    if (length < min_length_ || length > max_length_ ||
      width < min_width_ || width > max_width_ ||
      height < min_height_ || height > max_height_)
    {
      return false;
    }

    const double center_z = 0.5 * (cluster.z_min + cluster.z_max);
    if (reject_low_flat_clusters_ &&
      height <= low_flat_max_height_m_ &&
      center_z <= low_flat_max_center_z_m_ &&
      length >= low_flat_min_length_m_)
    {
      return false;
    }

    const double aspect = length / std::max(width, 1e-3);
    if (aspect > max_aspect_ratio_) {
      return false;
    }
    if (!passes_vehicle_spec_gate(length, width, height)) {
      return false;
    }

    box.cluster_id = cluster_id;
    box.point_count = cluster.points.size();
    box.cx = 0.25 * (p1.x + p2.x + p3.x + p4.x);
    box.cy = 0.25 * (p1.y + p2.y + p3.y + p4.y);
    box.cz = 0.5 * (cluster.z_min + cluster.z_max);
    box.yaw = yaw;
    box.length = length;
    box.width = width;
    box.height = height;
    box.heading_confidence = used_pca_heading ? heading_confidence : 0.5;
    box.used_pca_heading = used_pca_heading;
    box.corners = {p1, p2, p3, p4};
    return true;
  }

  double compute_confidence(const BoxResult & box) const
  {
    const double len_tol = std::max(vehicle_length_tolerance_m_, 1e-3);
    const double wid_tol = std::max(vehicle_width_tolerance_m_, 1e-3);

    const double len_err = (box.length - vehicle_length_m_) / len_tol;
    const double wid_err = (box.width - vehicle_width_m_) / wid_tol;
    const double z = len_err * len_err + wid_err * wid_err;
    const double c_size = std::exp(-0.5 * z);

    const double n = static_cast<double>(box.point_count);
    const int required_min_points = required_min_points_for_range(std::hypot(box.cx, box.cy));
    const double n_min = static_cast<double>(std::max(1, required_min_points));
    const double c_points = clamp01((n - n_min) / (3.0 * n_min));

    const double aspect = box.length / std::max(box.width, 1e-3);
    const double aspect_denom = std::max(1e-3, max_aspect_ratio_ - 1.0);
    const double c_aspect = clamp01((max_aspect_ratio_ - aspect) / aspect_denom);

    return clamp01(0.6 * c_size + 0.3 * c_points + 0.1 * c_aspect);
  }

  // ----- Visualization geometry ---------------------------------------------

  static std::array<float, 3> id_to_color(int32_t id)
  {
    const uint32_t h = static_cast<uint32_t>(id) * 2654435761U;
    const float r = 0.2F + 0.8F * static_cast<float>((h >> 16) & 0xFFU) / 255.0F;
    const float g = 0.2F + 0.8F * static_cast<float>((h >> 8) & 0xFFU) / 255.0F;
    const float b = 0.2F + 0.8F * static_cast<float>(h & 0xFFU) / 255.0F;
    return {r, g, b};
  }

  static std::array<PointXYZ, 4> compute_corners(
    double cx,
    double cy,
    double yaw,
    double length,
    double width)
  {
    const double half_length = 0.5 * length;
    const double half_width = 0.5 * width;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    auto make_corner = [&](double local_x, double local_y) {
        PointXYZ point;
        point.x = cx + local_x * cos_yaw - local_y * sin_yaw;
        point.y = cy + local_x * sin_yaw + local_y * cos_yaw;
        point.z = 0.0;
        return point;
      };

    return {
      make_corner(-half_length, -half_width),
      make_corner(half_length, -half_width),
      make_corner(half_length, half_width),
      make_corner(-half_length, half_width)};
  }

  // The raw L-shape yaw can flip by pi/2 or pi when points are sparse. This
  // keeps short-term yaw continuity without changing the fitted center.
  void stabilize_box_yaws(std::vector<BoxResult> & boxes) const
  {
    if (!enable_temporal_yaw_stabilization_ || previous_boxes_.empty() || boxes.empty()) {
      return;
    }

    std::vector<bool> previous_used(previous_boxes_.size(), false);
    for (auto & box : boxes) {
      std::optional<std::size_t> best_index;
      double best_cost = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < previous_boxes_.size(); ++i) {
        if (previous_used[i]) {
          continue;
        }

        const auto & previous = previous_boxes_[i];
        const double dx = box.cx - previous.cx;
        const double dy = box.cy - previous.cy;
        const double distance = std::hypot(dx, dy);
        if (distance > temporal_yaw_match_distance_m_) {
          continue;
        }

        const double size_cost =
          std::abs(box.length - previous.length) + std::abs(box.width - previous.width);
        const double total_cost = distance + temporal_yaw_size_weight_ * size_cost;
        if (total_cost < best_cost) {
          best_cost = total_cost;
          best_index = i;
        }
      }

      if (!best_index.has_value()) {
        continue;
      }

      previous_used[*best_index] = true;
      const auto & previous = previous_boxes_[*best_index];
      const double dx = box.cx - previous.cx;
      const double dy = box.cy - previous.cy;
      const double distance = std::hypot(dx, dy);

      const double pos_alpha = clamp01(temporal_position_alpha_);
      const double size_alpha = clamp01(temporal_size_alpha_);
      box.cx = (1.0 - pos_alpha) * previous.cx + pos_alpha * box.cx;
      box.cy = (1.0 - pos_alpha) * previous.cy + pos_alpha * box.cy;
      box.length = (1.0 - size_alpha) * previous.length + size_alpha * box.length;
      box.width = (1.0 - size_alpha) * previous.width + size_alpha * box.width;

      double reference_yaw = previous.yaw;
      if (distance >= temporal_motion_heading_min_distance_m_) {
        const double motion_heading = std::atan2(dy, dx);
        const double aligned_motion_heading =
          choose_yaw_closest_to_reference(motion_heading, previous.yaw);
        reference_yaw = normalize_yaw(
          (1.0 - temporal_motion_heading_alpha_) * previous.yaw +
          temporal_motion_heading_alpha_ * aligned_motion_heading);
      }

      const double aligned_yaw = choose_yaw_closest_to_reference(box.yaw, reference_yaw);
      const double yaw_error = normalize_yaw(aligned_yaw - reference_yaw);
      const double size_jump =
        std::abs(box.length - previous.length) + std::abs(box.width - previous.width);
      const double point_ratio =
        previous.point_count > 0U ?
        static_cast<double>(box.point_count) / static_cast<double>(previous.point_count) :
        1.0;
      const bool low_heading_confidence =
        !box.used_pca_heading || box.heading_confidence < yaw_freeze_low_confidence_threshold_;
      const bool point_count_dropped = point_ratio < yaw_freeze_point_count_drop_ratio_;
      const bool size_jumped = size_jump > yaw_freeze_size_jump_m_;
      const bool large_yaw_jump = std::abs(yaw_error) > yaw_freeze_large_error_rad_;
      if (large_yaw_jump && (low_heading_confidence || point_count_dropped || size_jumped)) {
        box.yaw = previous.yaw;
        box.corners = compute_corners(box.cx, box.cy, box.yaw, box.length, box.width);
        continue;
      }
      box.yaw = normalize_yaw(reference_yaw + temporal_yaw_alpha_ * yaw_error);
      box.corners = compute_corners(box.cx, box.cy, box.yaw, box.length, box.width);
    }
  }

  // ----- ROS callback --------------------------------------------------------

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const size_t total_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (total_points == 0U) {
      return;
    }
    if (msg->point_step == 0U || msg->data.size() < total_points * static_cast<size_t>(msg->point_step)) {
      RCLCPP_WARN(get_logger(), "Invalid PointCloud2 layout");
      return;
    }

    FieldMeta meta_x = find_field(msg->fields, "x");
    FieldMeta meta_y = find_field(msg->fields, "y");
    FieldMeta meta_z = find_field(msg->fields, "z");
    FieldMeta meta_cluster = find_field(msg->fields, cluster_field_);
    if (meta_cluster.offset < 0 && cluster_field_ != "cluster_id") {
      meta_cluster = find_field(msg->fields, "cluster_id");
    }
    if (meta_cluster.offset < 0 && cluster_field_ != "cluster") {
      meta_cluster = find_field(msg->fields, "cluster");
    }

    if (meta_x.offset < 0 || meta_y.offset < 0 || meta_z.offset < 0 || meta_cluster.offset < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Required fields not found. Need x/y/z and cluster field '%s' (or fallback).",
        cluster_field_.c_str());
      return;
    }

    auto out_header = msg->header;
    if (!output_frame_id_.empty()) {
      out_header.frame_id = output_frame_id_;
    }
    align_output_header_stamp(out_header);

    std::unordered_map<int32_t, ClusterData> clusters;
    clusters.reserve(256);

    // 1) Decode clustered cloud into compact per-cluster point lists.
    const size_t point_step = static_cast<size_t>(msg->point_step);
    for (size_t i = 0; i < total_points; ++i) {
      const uint8_t * point_ptr = &msg->data[i * point_step];

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      int32_t cluster_id = -1;
      if (!read_float_field(point_ptr, meta_x, x) ||
        !read_float_field(point_ptr, meta_y, y) ||
        !read_float_field(point_ptr, meta_z, z) ||
        !read_int32_field(point_ptr, meta_cluster, cluster_id))
      {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || cluster_id < 0) {
        continue;
      }

      const double px = static_cast<double>(x) + input_offset_x_m_;
      const double py = static_cast<double>(y) + input_offset_y_m_;
      const double pz = static_cast<double>(z) + input_offset_z_m_;

      auto & data = clusters[cluster_id];
      data.points.push_back(PointXYZ{px, py, pz});
      data.z_min = std::min(data.z_min, pz);
      data.z_max = std::max(data.z_max, pz);
    }

    // 2) Prepare all outputs. DetectionArray is the functional output used by
    // obstacle_tracking; MarkerArray/PoseArray are only for RViz inspection.
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_vis_array;
    pose_vis_array.header = out_header;
    lshape_fitting::msg::Detection2DArray detection_array;
    detection_array.header = out_header;

    // DELETEALL은 한 번만 보내면 충분함
    visualization_msgs::msg::Marker clear_all;
    clear_all.header = out_header;
    clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_all);

    if (publish_ego_marker_) {
      const double ego_yaw = ego_marker_yaw_deg_ * kDegToRad;
      tf2::Quaternion ego_q;
      ego_q.setRPY(0.0, 0.0, ego_yaw);

      const double ego_vis_height = std::max(0.01, ego_marker_height_m_);
      const double ego_vis_center_z = ego_marker_base_z_m_ + 0.5 * ego_vis_height;
      const auto ego_corners = compute_corners(
        ego_marker_x_m_, ego_marker_y_m_, ego_yaw, ego_marker_length_m_, ego_marker_width_m_);

      visualization_msgs::msg::Marker ego_cube;
      ego_cube.header = out_header;
      ego_cube.ns = "ego_cube";
      ego_cube.id = 0;
      ego_cube.type = visualization_msgs::msg::Marker::CUBE;
      ego_cube.action = visualization_msgs::msg::Marker::ADD;
      ego_cube.pose.position.x = ego_marker_x_m_;
      ego_cube.pose.position.y = ego_marker_y_m_;
      ego_cube.pose.position.z = ego_vis_center_z;
      ego_cube.pose.orientation = tf2::toMsg(ego_q);
      ego_cube.scale.x = ego_marker_length_m_;
      ego_cube.scale.y = ego_marker_width_m_;
      ego_cube.scale.z = ego_vis_height;
      ego_cube.color.r = 1.0F;
      ego_cube.color.g = 1.0F;
      ego_cube.color.b = 1.0F;
      ego_cube.color.a = static_cast<float>(marker_alpha_);
      marker_array.markers.push_back(ego_cube);

      if (publish_outline_) {
        visualization_msgs::msg::Marker ego_outline;
        ego_outline.header = out_header;
        ego_outline.ns = "ego_outline";
        ego_outline.id = 0;
        ego_outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ego_outline.action = visualization_msgs::msg::Marker::ADD;
        ego_outline.scale.x = std::max(0.001, outline_width_);
        ego_outline.color.r = 1.0F;
        ego_outline.color.g = 1.0F;
        ego_outline.color.b = 1.0F;
        ego_outline.color.a = 1.0F;
        ego_outline.pose.orientation.w = 1.0;
        for (size_t i = 0; i < 5U; ++i) {
          const auto & corner = ego_corners[i % 4U];
          geometry_msgs::msg::Point point;
          point.x = corner.x;
          point.y = corner.y;
          point.z = ego_marker_base_z_m_;
          ego_outline.points.push_back(point);
        }
        marker_array.markers.push_back(ego_outline);
      }

      if (publish_text_markers_) {
        visualization_msgs::msg::Marker ego_text;
        ego_text.header = out_header;
        ego_text.ns = "ego_text";
        ego_text.id = 0;
        ego_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        ego_text.action = visualization_msgs::msg::Marker::ADD;
        ego_text.pose.position.x = ego_marker_x_m_;
        ego_text.pose.position.y = ego_marker_y_m_;
        ego_text.pose.position.z = ego_vis_center_z + ego_vis_height * 0.5 + 0.25;
        ego_text.pose.orientation.w = 1.0;
        ego_text.scale.z = 0.3;
        ego_text.color.r = 1.0F;
        ego_text.color.g = 1.0F;
        ego_text.color.b = 1.0F;
        ego_text.color.a = 1.0F;
        ego_text.text =
          "id=ego yaw=" + format_fixed(ego_marker_yaw_deg_, 1) +
          " x=" + format_fixed(ego_marker_x_m_) +
          " y=" + format_fixed(ego_marker_y_m_);
        marker_array.markers.push_back(ego_text);
      }
    }

    std::vector<int32_t> keys;
    keys.reserve(clusters.size());
    for (const auto & kv : clusters) {
      keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end());

    // 3) Fit each source cluster independently, then apply temporal yaw
    // stabilization across fitted boxes.
    size_t fitted = 0U;
    std::vector<BoxResult> fitted_boxes;
    fitted_boxes.reserve(keys.size());
    for (const auto cluster_id : keys) {
      const auto it = clusters.find(cluster_id);
      if (it == clusters.end()) {
        continue;
      }

      BoxResult box;
      if (!fit_cluster_to_box(cluster_id, it->second, box)) {
        continue;
      }

      ++fitted;
      fitted_boxes.push_back(box);
    }

    stabilize_box_yaws(fitted_boxes);

    previous_boxes_.clear();
    previous_boxes_.reserve(fitted_boxes.size());
    for (const auto & box : fitted_boxes) {
      previous_boxes_.push_back(
        PreviousBoxState{
          box.cx,
          box.cy,
          box.yaw,
          box.length,
          box.width,
          box.point_count,
          box.heading_confidence,
          box.used_pca_heading});
    }

    // 4) Publish one detection and matching RViz markers for every accepted box.
    for (const auto & box : fitted_boxes) {
      const auto [r, g, b] = id_to_color(box.cluster_id);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, box.yaw);

      const double vis_height = use_fixed_output_height_ ?
        std::max(0.01, fixed_output_height_m_) :
        std::max(box.height, marker_min_height_);
      const double vis_center_z = use_fixed_output_height_ ?
        (fixed_output_base_z_m_ + 0.5 * vis_height) :
        box.cz;

      // 각 namespace 안에서는 cluster_id를 그대로 marker id로 사용
      visualization_msgs::msg::Marker cube;
      cube.header = out_header;
      cube.ns = "lshape_cube";
      cube.id = box.cluster_id;
      cube.type = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose.position.x = box.cx;
      cube.pose.position.y = box.cy;
      cube.pose.position.z = vis_center_z;
      cube.pose.orientation = tf2::toMsg(q);
      cube.scale.x = box.length;
      cube.scale.y = box.width;
      cube.scale.z = vis_height;
      cube.color.r = r;
      cube.color.g = g;
      cube.color.b = b;
      cube.color.a = static_cast<float>(marker_alpha_);
      marker_array.markers.push_back(cube);

      if (publish_outline_) {
        visualization_msgs::msg::Marker outline;
        outline.header = out_header;
        outline.ns = "lshape_outline";
        outline.id = box.cluster_id;
        outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
        outline.action = visualization_msgs::msg::Marker::ADD;
        outline.scale.x = std::max(0.001, outline_width_);
        outline.color.r = r;
        outline.color.g = g;
        outline.color.b = b;
        outline.color.a = 1.0F;
        outline.pose.orientation.w = 1.0;
        const double z_draw = use_fixed_output_height_ ? fixed_output_base_z_m_ : (box.cz - 0.5 * box.height + 0.05);
        for (size_t i = 0; i < 5U; ++i) {
          const auto & corner = box.corners[i % 4U];
          geometry_msgs::msg::Point p;
          p.x = corner.x;
          p.y = corner.y;
          p.z = z_draw;
          outline.points.push_back(p);
        }
        marker_array.markers.push_back(outline);
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = box.cx;
      pose.position.y = box.cy;
      pose.position.z = 0.0;
      pose.orientation = tf2::toMsg(q);
      pose_vis_array.poses.push_back(pose);

      lshape_fitting::msg::Detection2D detection;
      detection.header = out_header;
      detection.vehicle_id = box.cluster_id;
      detection.pose = pose;
      detection.length = static_cast<float>(box.length);
      detection.width = static_cast<float>(box.width);
      detection.confidence = static_cast<float>(compute_confidence(box));
      detection_array.detections.push_back(detection);

      if (publish_text_markers_) {
        visualization_msgs::msg::Marker text;
        text.header = out_header;
        text.ns = "lshape_text";
        text.id = box.cluster_id;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = box.cx;
        text.pose.position.y = box.cy;
        text.pose.position.z = vis_center_z + vis_height * 0.5 + 0.25;
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.3;
        text.color.r = 1.0F;
        text.color.g = 1.0F;
        text.color.b = 1.0F;
        text.color.a = 1.0F;
        text.text =
          "id=" + std::to_string(box.cluster_id) +
          " yaw=" + format_fixed(box.yaw * 180.0 / kPi, 1) +
          (box.used_pca_heading ? " pca" : " lshape") +
          " x=" + format_fixed(box.cx) +
          " y=" + format_fixed(box.cy);
        marker_array.markers.push_back(text);
      }
    }

    if (marker_pub_) {
      marker_pub_->publish(marker_array);
    }
    if (pose_vis_pub_) {
      pose_vis_pub_->publish(pose_vis_array);
    }
    detection_pub_->publish(detection_array);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "clusters:%zu fitted:%zu poses_vis:%zu detections:%zu mode:%s dtheta:%.2f",
      clusters.size(), fitted, pose_vis_array.poses.size(), detection_array.detections.size(),
      score_mode_.c_str(), delta_theta_deg_);
  }

  // ----- ROS interfaces ------------------------------------------------------

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_vis_pub_;
  rclcpp::Publisher<lshape_fitting::msg::Detection2DArray>::SharedPtr detection_pub_;

  std::string cluster_field_{"cluster_id"};
  std::string score_mode_{"closeness"};

  // ----- Fitting gates and scoring parameters --------------------------------

  int min_cluster_points_{10};
  int far_min_cluster_points_{6};
  double far_min_cluster_points_range_m_{15.0};
  int max_cluster_points_{5000};
  double delta_theta_deg_{1.0};
  double d0_{0.1};
  double closeness_weight_{1.0};
  double variance_weight_{1.0};

  double min_length_{0.2};
  double max_length_{5.0};
  double min_width_{0.15};
  double max_width_{2.4};
  double min_height_{0.05};
  double max_height_{3.0};
  double max_aspect_ratio_{8.0};
  bool reject_low_flat_clusters_{true};
  double low_flat_max_height_m_{0.22};
  double low_flat_max_center_z_m_{-0.35};
  double low_flat_min_length_m_{0.6};

  // ----- RViz marker parameters ---------------------------------------------

  double marker_alpha_{0.45};
  double marker_min_height_{0.2};
  bool use_fixed_output_height_{true};
  double fixed_output_height_m_{1.3};
  double fixed_output_base_z_m_{0.0};
  double outline_width_{0.05};
  bool publish_outline_{true};
  bool publish_text_markers_{true};
  bool publish_ego_marker_{true};
  double ego_marker_x_m_{0.20};
  double ego_marker_y_m_{0.0};
  double ego_marker_yaw_deg_{0.0};
  double ego_marker_length_m_{2.60};
  double ego_marker_width_m_{1.50};
  double ego_marker_height_m_{1.10};
  double ego_marker_base_z_m_{-0.67};

  // ----- Heading stabilization and frame handling ---------------------------

  bool use_pca_heading_{true};
  double pca_min_anisotropy_ratio_{1.15};
  bool force_heading_positive_x_{true};
  bool enable_temporal_yaw_stabilization_{true};
  double temporal_yaw_match_distance_m_{1.5};
  double temporal_yaw_size_weight_{0.5};
  double temporal_yaw_alpha_{0.35};
  double temporal_position_alpha_{0.45};
  double temporal_size_alpha_{0.35};
  double temporal_motion_heading_alpha_{0.50};
  double temporal_motion_heading_min_distance_m_{0.20};
  double yaw_freeze_low_confidence_threshold_{0.45};
  double yaw_freeze_point_count_drop_ratio_{0.55};
  double yaw_freeze_size_jump_m_{0.90};
  double yaw_freeze_large_error_rad_{0.70};
  std::string output_frame_id_{};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  bool align_output_stamp_to_tf_{true};
  double output_stamp_tf_tolerance_sec_{0.05};
  double input_offset_x_m_{0.0};
  double input_offset_y_m_{0.0};
  double input_offset_z_m_{0.0};

  // ----- Vehicle-size prior --------------------------------------------------

  bool fit_vehicle_spec_only_{true};
  bool use_vehicle_size_prior_{true};
  bool use_vehicle_height_gate_{true};
  bool require_wheel_track_consistency_{true};
  double vehicle_size_weight_{25.0};
  double vehicle_length_m_{2.060};
  double vehicle_width_m_{1.160};
  double vehicle_height_m_{0.822};
  double vehicle_ground_clearance_m_{0.160};
  double vehicle_track_front_m_{0.970};
  double vehicle_track_rear_m_{0.938};
  double vehicle_wheelbase_m_{1.212};
  double vehicle_length_tolerance_m_{0.800};
  double vehicle_width_tolerance_m_{0.500};
  double vehicle_height_tolerance_m_{0.800};
  std::vector<PreviousBoxState> previous_boxes_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LShapeFittingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
