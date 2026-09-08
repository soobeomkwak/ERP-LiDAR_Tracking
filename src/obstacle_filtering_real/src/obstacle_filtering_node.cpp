#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <filesystem>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "obstacle_filtering/waypoint_graph.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

using std::placeholders::_1;
using obstacle_filtering::Point2D;

namespace
{

// Pipeline:
//   /pointcloud/clustered
//     -> transform cluster representatives into map frame
//     -> compare each cluster against the local L1/R1 waypoint corridor
//     -> publish road-side clusters on /obstacle_in and the rest on /obstacle_out.
//
// When too many objects appear after replay reset, inspect TF freshness first:
// stale map->base transforms intentionally fail closed by blocking filtered output.
constexpr const char * kColorGreen = "\033[1;32m";
constexpr const char * kColorOrange = "\033[1;33m";
constexpr const char * kColorRed = "\033[1;31m";
constexpr const char * kColorReset = "\033[0m";
constexpr double kTfLookupTimeoutSec = 0.02;
constexpr double kTfFreshAgeSec = 0.05;
constexpr double kTfWarnAgeSec = 0.10;
// How long a cached good TF pose can be reused when TF goes stale during operation.
// At startup (before first valid TF), cache is empty so fail-closed still applies.
constexpr double kTfCacheFallbackMaxAgeSec = 0.50;

struct FieldMeta
{
  int offset{-1};
  uint8_t datatype{0};
};

struct ClusterAggregate
{
  int total{0};
  int inside_count{0};
  double sum_lx{0.0};
  double sum_ly{0.0};
  double sum_lz{0.0};

  double min_by{std::numeric_limits<double>::infinity()};
  double max_by{-std::numeric_limits<double>::infinity()};
  double min_bz{std::numeric_limits<double>::infinity()};
  double max_bz{-std::numeric_limits<double>::infinity()};
};

struct ClusterState
{
  double x{0.0};
  double y{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double heading_base{0.0};
  double heading_map{0.0};
  bool valid{false};
};

struct ObstacleSummary
{
  int32_t id{-1};
  int32_t source_cluster_id{-1};
  double map_x{0.0};
  double map_y{0.0};
  double map_z{0.0};
  double base_x{0.0};
  double base_y{0.0};
  double base_z{0.0};
  double distance{0.0};
  double width{0.0};
  double height{0.0};
  double velocity_x{0.0};
  double velocity_y{0.0};
  double speed{0.0};
  double heading_base{0.0};
  double heading_map{0.0};
  double nearest_wp_distance{0.0};
  double nearest_left_distance{0.0};
  double nearest_right_distance{0.0};
  uint32_t point_count{0U};
  double inside_point_ratio{0.0};
};

struct OutputPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  int32_t id{-1};
  float value{0.0F};
};

struct ValidPoint
{
  int index{0};
  int32_t cluster_id{-1};
};

// Lightweight static 2D KD-tree for nearest-distance queries.
class KdTree2D
{
public:
  void build(const std::vector<Point2D> & pts)
  {
    pts_ = &pts;
    nodes_.clear();
    if (pts.empty()) {
      root_ = -1;
      return;
    }

    indices_.resize(pts.size());
    std::iota(indices_.begin(), indices_.end(), 0U);
    nodes_.reserve(pts.size());
    root_ = build_recursive(0, static_cast<int>(indices_.size()), 0);
  }

  float nearest_sq(float qx, float qy) const
  {
    if (root_ < 0) {
      return std::numeric_limits<float>::infinity();
    }
    float best = std::numeric_limits<float>::infinity();
    nearest_recursive(root_, qx, qy, best);
    return best;
  }

private:
  struct Node
  {
    float x{0.0F};
    float y{0.0F};
    int left{-1};
    int right{-1};
    int axis{0};
  };

  int build_recursive(int begin, int end, int depth)
  {
    if (begin >= end) {
      return -1;
    }
    const int axis = depth % 2;
    const int mid = begin + (end - begin) / 2;

    auto comp = [&](std::size_t a, std::size_t b) {
      if (axis == 0) {
        return (*pts_)[a].x < (*pts_)[b].x;
      }
      return (*pts_)[a].y < (*pts_)[b].y;
    };
    std::nth_element(indices_.begin() + begin, indices_.begin() + mid, indices_.begin() + end, comp);

    const auto pi = indices_[static_cast<std::size_t>(mid)];
    Node n;
    n.x = static_cast<float>((*pts_)[pi].x);
    n.y = static_cast<float>((*pts_)[pi].y);
    n.axis = axis;

    const int node_idx = static_cast<int>(nodes_.size());
    nodes_.push_back(n);
    nodes_[static_cast<std::size_t>(node_idx)].left = build_recursive(begin, mid, depth + 1);
    nodes_[static_cast<std::size_t>(node_idx)].right = build_recursive(mid + 1, end, depth + 1);
    return node_idx;
  }

  void nearest_recursive(int node_idx, float qx, float qy, float & best) const
  {
    const Node & n = nodes_[static_cast<std::size_t>(node_idx)];
    const float dx = qx - n.x;
    const float dy = qy - n.y;
    const float d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
    }

    const float delta = (n.axis == 0) ? dx : dy;
    const int near = (delta <= 0.0F) ? n.left : n.right;
    const int far = (delta <= 0.0F) ? n.right : n.left;

    if (near >= 0) {
      nearest_recursive(near, qx, qy, best);
    }
    if (far >= 0 && delta * delta < best) {
      nearest_recursive(far, qx, qy, best);
    }
  }

  const std::vector<Point2D> * pts_{nullptr};
  int root_{-1};
  std::vector<std::size_t> indices_;
  std::vector<Node> nodes_;
};

struct PoseState
{
  bool valid{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double map_x{0.0};
  double map_y{0.0};
  double map_z{0.0};
  double yaw{0.0};
};

struct TfPoseLookup
{
  PoseState pose;
  bool usable{false};
  bool warn_stale{false};
  double age_sec{std::numeric_limits<double>::infinity()};
  std::string reason;
};

// PointCloud2 helpers are datatype-tolerant because upstream cloud producers
// may encode cluster ids as signed/unsigned integer fields.
bool read_float_field(const uint8_t * ptr, const FieldMeta & meta, float & out)
{
  if (meta.offset < 0) {
    return false;
  }

  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    std::memcpy(&out, ptr + meta.offset, sizeof(float));
    return true;
  }

  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    double tmp = 0.0;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(double));
    out = static_cast<float>(tmp);
    return true;
  }

  return false;
}

bool write_float_field(uint8_t * ptr, const FieldMeta & meta, float value)
{
  if (meta.offset < 0) {
    return false;
  }

  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    std::memcpy(ptr + meta.offset, &value, sizeof(float));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    const double tmp = static_cast<double>(value);
    std::memcpy(ptr + meta.offset, &tmp, sizeof(double));
    return true;
  }
  return false;
}

bool read_int32_field(const uint8_t * ptr, const FieldMeta & meta, int32_t & out)
{
  if (meta.offset < 0) {
    return false;
  }

  if (meta.datatype == sensor_msgs::msg::PointField::INT32) {
    std::memcpy(&out, ptr + meta.offset, sizeof(int32_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT32) {
    uint32_t tmp = 0U;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(uint32_t));
    out = static_cast<int32_t>(tmp);
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::INT16) {
    int16_t tmp = 0;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(int16_t));
    out = static_cast<int32_t>(tmp);
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT16) {
    uint16_t tmp = 0U;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(uint16_t));
    out = static_cast<int32_t>(tmp);
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::INT8) {
    int8_t tmp = 0;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(int8_t));
    out = static_cast<int32_t>(tmp);
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT8) {
    uint8_t tmp = 0U;
    std::memcpy(&tmp, ptr + meta.offset, sizeof(uint8_t));
    out = static_cast<int32_t>(tmp);
    return true;
  }

  return false;
}

bool write_int32_field(uint8_t * ptr, const FieldMeta & meta, int32_t value)
{
  if (meta.offset < 0) {
    return false;
  }

  if (meta.datatype == sensor_msgs::msg::PointField::INT32) {
    std::memcpy(ptr + meta.offset, &value, sizeof(int32_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT32) {
    const uint32_t tmp = (value < 0) ? 0U : static_cast<uint32_t>(value);
    std::memcpy(ptr + meta.offset, &tmp, sizeof(uint32_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::INT16) {
    const int16_t tmp = static_cast<int16_t>(std::max<int32_t>(std::numeric_limits<int16_t>::min(),
        std::min<int32_t>(std::numeric_limits<int16_t>::max(), value)));
    std::memcpy(ptr + meta.offset, &tmp, sizeof(int16_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT16) {
    const uint16_t tmp = static_cast<uint16_t>(std::max<int32_t>(0,
        std::min<int32_t>(std::numeric_limits<uint16_t>::max(), value)));
    std::memcpy(ptr + meta.offset, &tmp, sizeof(uint16_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::INT8) {
    const int8_t tmp = static_cast<int8_t>(std::max<int32_t>(std::numeric_limits<int8_t>::min(),
        std::min<int32_t>(std::numeric_limits<int8_t>::max(), value)));
    std::memcpy(ptr + meta.offset, &tmp, sizeof(int8_t));
    return true;
  }
  if (meta.datatype == sensor_msgs::msg::PointField::UINT8) {
    const uint8_t tmp = static_cast<uint8_t>(std::max<int32_t>(0,
        std::min<int32_t>(std::numeric_limits<uint8_t>::max(), value)));
    std::memcpy(ptr + meta.offset, &tmp, sizeof(uint8_t));
    return true;
  }

  return false;
}

double yaw_from_quaternion(double w, double x, double y, double z)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline void rotate_xy(double yaw, double x, double y, double & ox, double & oy)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  ox = c * x - s * y;
  oy = s * x + c * y;
}

std::string to_pretty_text(
  const builtin_interfaces::msg::Time & stamp,
  const std::vector<ObstacleSummary> & obstacles)
{
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(3);
  oss << "stamp: " << stamp.sec << "." << stamp.nanosec << "\n";
  oss << "frame: map\n";
  oss << "count: " << obstacles.size();

  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const auto & o = obstacles[i];
    oss << "\n- obstacle[" << i << "]\n";
    oss << "  id: " << o.id << "\n";
    oss << "  source_cluster_id: " << o.source_cluster_id << "\n";
    oss << "  map_xyz: (" << o.map_x << ", " << o.map_y << ", " << o.map_z << ")\n";
    oss << "  base_xyz: (" << o.base_x << ", " << o.base_y << ", " << o.base_z << ")\n";
    oss << "  distance: " << o.distance << "\n";
    oss << "  width: " << o.width << "\n";
    oss << "  height: " << o.height << "\n";
    oss << "  velocity_xy: (" << o.velocity_x << ", " << o.velocity_y << ")\n";
    oss << "  speed: " << o.speed << "\n";
    oss << "  heading_base_rad: " << o.heading_base << "\n";
    oss << "  heading_map_rad: " << o.heading_map << "\n";
    oss << "  nearest_wp_distance: " << o.nearest_wp_distance << "\n";
    oss << "  nearest_left_distance: " << o.nearest_left_distance << "\n";
    oss << "  nearest_right_distance: " << o.nearest_right_distance << "\n";
    oss << "  point_count: " << o.point_count << "\n";
    oss << "  inside_point_ratio: " << o.inside_point_ratio;
  }
  return oss.str();
}

std::string resolve_waypoint_csv_path(const std::string & configured_path)
{
  if (!configured_path.empty()) {
    return configured_path;
  }

  const auto planning_share =
    ament_index_cpp::get_package_share_directory("erp42_racing_planning");
  const auto planning_csv =
    std::filesystem::path(planning_share) / "resource" / "waypoint_true.csv";
  return planning_csv.string();
}

}  // namespace

class ObstacleFilteringNode : public rclcpp::Node
{
public:
  explicit ObstacleFilteringNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("obstacle_filtering", options)
  {
    declare_parameter<std::string>("input_cluster_topic", "/pointcloud/clustered");

    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("waypoint_csv_path", "");

    declare_parameter<int>("front_waypoint_count", 280);
    declare_parameter<int>("rear_waypoint_count", 280);
    declare_parameter<double>("waypoint_interp_step_m", 0.3);
    declare_parameter<bool>("waypoint_window_use_lidar_roi", true);
    declare_parameter<double>("waypoint_roi_min_x_m", -30.0);
    declare_parameter<double>("waypoint_roi_max_x_m", 50.0);
    declare_parameter<double>("waypoint_roi_min_y_m", -20.0);
    declare_parameter<double>("waypoint_roi_max_y_m", 20.0);
    declare_parameter<double>("max_detection_range_m", 80.0);
    declare_parameter<double>("min_obstacle_distance_m", 1.5);
    // Hysteresis in/out rule from nearest L1/R1 waypoint distance.
    // L1/R1 are lane-center waypoint lines. With 3 m lanes, the geometric
    // half-width is 1.5 m; use a slightly larger in_radius_m when missing
    // road obstacles is worse than admitting some shoulder/curb clutter.
    declare_parameter<double>("in_radius_m", 1.8);
    declare_parameter<double>("out_radius_m", 2.1);
    declare_parameter<double>("far_filter_range_m", 12.0);
    declare_parameter<double>("far_in_radius_m", 2.6);
    declare_parameter<double>("far_out_radius_m", 3.0);
    declare_parameter<double>("inside_point_ratio_threshold", 0.20);
    declare_parameter<double>("far_inside_point_ratio_threshold", 0.10);

    declare_parameter<double>("base_to_lidar_x_m", -0.2);
    declare_parameter<double>("base_to_lidar_y_m", 0.0);
    declare_parameter<double>("base_to_lidar_z_m", 0.57);

    declare_parameter<std::string>("output_obstacle_in_topic", "/obstacle_in");
    declare_parameter<std::string>("output_obstacle_out_topic", "/obstacle_out");
    declare_parameter<std::string>("output_obstacle_in_debug_topic", "/obstacle_in_debug");
    declare_parameter<std::string>("output_obstacle_out_debug_topic", "/obstacle_out_debug");
    declare_parameter<bool>("publish_debug_text", false);
    declare_parameter<bool>("publish_debug_cloud", false);
    declare_parameter<std::string>("output_obstacle_in_cloud_topic", "/obstacle_in_repr");
    declare_parameter<std::string>("output_obstacle_out_cloud_topic", "/obstacle_out_repr");
    declare_parameter<std::string>("cluster_field_name", "cluster_id");
    declare_parameter<int>("queue_size", 10);

    const auto input_cluster_topic = get_parameter("input_cluster_topic").as_string();
    const auto waypoint_csv_path =
      resolve_waypoint_csv_path(get_parameter("waypoint_csv_path").as_string());

    base_frame_ = get_parameter("base_frame").as_string();
    map_frame_ = get_parameter("map_frame").as_string();

    front_waypoint_count_ =
      std::max(0, static_cast<int>(get_parameter("front_waypoint_count").as_int()));
    rear_waypoint_count_ =
      std::max(0, static_cast<int>(get_parameter("rear_waypoint_count").as_int()));
    waypoint_interp_step_m_ = std::max(0.05, get_parameter("waypoint_interp_step_m").as_double());
    waypoint_window_use_lidar_roi_ = get_parameter("waypoint_window_use_lidar_roi").as_bool();
    waypoint_roi_min_x_m_ = get_parameter("waypoint_roi_min_x_m").as_double();
    waypoint_roi_max_x_m_ = get_parameter("waypoint_roi_max_x_m").as_double();
    waypoint_roi_min_y_m_ = get_parameter("waypoint_roi_min_y_m").as_double();
    waypoint_roi_max_y_m_ = get_parameter("waypoint_roi_max_y_m").as_double();
    max_detection_range_m_ = std::max(0.0, get_parameter("max_detection_range_m").as_double());
    min_obstacle_distance_m_ = std::max(0.0, get_parameter("min_obstacle_distance_m").as_double());
    in_radius_m_ = std::max(0.0, get_parameter("in_radius_m").as_double());
    out_radius_m_ = std::max(in_radius_m_, get_parameter("out_radius_m").as_double());
    far_filter_range_m_ = std::max(0.0, get_parameter("far_filter_range_m").as_double());
    far_in_radius_m_ = std::max(in_radius_m_, get_parameter("far_in_radius_m").as_double());
    far_out_radius_m_ = std::max(far_in_radius_m_, get_parameter("far_out_radius_m").as_double());
    inside_point_ratio_threshold_ = std::clamp(
      get_parameter("inside_point_ratio_threshold").as_double(), 0.0, 1.0);
    far_inside_point_ratio_threshold_ = std::clamp(
      get_parameter("far_inside_point_ratio_threshold").as_double(), 0.0, 1.0);

    base_to_lidar_x_m_ = get_parameter("base_to_lidar_x_m").as_double();
    base_to_lidar_y_m_ = get_parameter("base_to_lidar_y_m").as_double();
    base_to_lidar_z_m_ = get_parameter("base_to_lidar_z_m").as_double();

    cluster_field_name_ = get_parameter("cluster_field_name").as_string();

    const auto output_obstacle_in_topic = get_parameter("output_obstacle_in_topic").as_string();
    const auto output_obstacle_out_topic = get_parameter("output_obstacle_out_topic").as_string();
    const auto output_obstacle_in_debug_topic = get_parameter("output_obstacle_in_debug_topic").as_string();
    const auto output_obstacle_out_debug_topic = get_parameter("output_obstacle_out_debug_topic").as_string();

    publish_debug_text_ = get_parameter("publish_debug_text").as_bool();
    publish_debug_cloud_ = get_parameter("publish_debug_cloud").as_bool();
    const auto in_cloud_topic = get_parameter("output_obstacle_in_cloud_topic").as_string();
    const auto out_cloud_topic = get_parameter("output_obstacle_out_cloud_topic").as_string();

    const int queue_size = std::max(1, static_cast<int>(get_parameter("queue_size").as_int()));

    aggregates_buf_.reserve(512);
    valid_points_buf_.reserve(200000);
    in_obstacles_buf_.reserve(512);
    out_obstacles_buf_.reserve(512);
    points_by_cluster_buf_.reserve(512);
    in_repr_buf_.reserve(512);
    out_repr_buf_.reserve(512);

    std::string load_error;
    if (!waypoint_graph_.load_from_utm_csv(waypoint_csv_path, load_error)) {
      throw std::runtime_error(load_error);
    }

    // Use the first L1 waypoint(UTM) as a fixed ENU map origin to match planning/localization.
    if (waypoint_graph_.nodes().empty()) {
      throw std::runtime_error("Waypoint CSV parsed but has no nodes");
    }
    const auto & first_right_utm = waypoint_graph_.nodes().front().right_utm;
    origin_e0_ = first_right_utm.x;
    origin_n0_ = first_right_utm.y;
    origin_u0_ = first_right_utm.z;
    origin_ready_ = waypoint_graph_.set_enu_origin_from_utm(origin_e0_, origin_n0_, origin_u0_);
    if (!origin_ready_) {
      throw std::runtime_error("Failed to set ENU origin from first R1 waypoint");
    }
    waypoint_subtraction_ok_ = verify_waypoint_origin_subtraction(
      origin_e0_, origin_n0_, origin_u0_, waypoint_subtraction_bad_count_, waypoint_subtraction_max_error_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(queue_size);

    sub_cluster_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cluster_topic,
      sensor_qos,
      std::bind(&ObstacleFilteringNode::on_cluster_cloud, this, _1));

    // Runtime output for downstream modules: PointCloud2 with re-assigned distance-sorted cluster ids.
    pub_obstacle_in_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_obstacle_in_topic,
      rclcpp::QoS(queue_size));

    pub_obstacle_out_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_obstacle_out_topic,
      rclcpp::QoS(queue_size));

    // Human-readable debug summaries are emitted on dedicated debug topics.
    pub_obstacle_in_debug_ = create_publisher<std_msgs::msg::String>(
      output_obstacle_in_debug_topic,
      rclcpp::QoS(queue_size));
    pub_obstacle_out_debug_ = create_publisher<std_msgs::msg::String>(
      output_obstacle_out_debug_topic,
      rclcpp::QoS(queue_size));

    if (publish_debug_cloud_) {
      pub_obstacle_in_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(in_cloud_topic, sensor_qos);
      pub_obstacle_out_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_cloud_topic, sensor_qos);
    }

    RCLCPP_INFO(
      get_logger(),
      "obstacle_filtering loaded %zu waypoint rows. csv:%s map:%s base:%s ego_pose:tf_lookup",
      waypoint_graph_.size(),
      waypoint_csv_path.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "%s[TF OK] lookup source: %s->%s timeout=%.0fms fresh<=%.0fms warn<=%.0fms%s",
      kColorGreen, map_frame_.c_str(), base_frame_.c_str(),
      kTfLookupTimeoutSec * 1000.0,
      kTfFreshAgeSec * 1000.0,
      kTfWarnAgeSec * 1000.0,
      kColorReset);
  }

private:
  // ----- Waypoint/frame validation ------------------------------------------

  bool verify_waypoint_origin_subtraction(double e0, double n0, double u0, std::size_t & bad_count, double & max_err) const
  {
    bad_count = 0U;
    max_err = 0.0;
    const auto & nodes = waypoint_graph_.nodes();
    for (const auto & n : nodes) {
      const double el = std::hypot(
        n.left_enu.x - (n.left_utm.x - e0),
        n.left_enu.y - (n.left_utm.y - n0));
      const double er = std::hypot(
        n.right_enu.x - (n.right_utm.x - e0),
        n.right_enu.y - (n.right_utm.y - n0));
      const double ec = std::hypot(
        n.center_enu.x - (n.center_utm.x - e0),
        n.center_enu.y - (n.center_utm.y - n0));
      const double ez = std::max(
        std::max(std::abs(n.left_enu.z - (n.left_utm.z - u0)), std::abs(n.right_enu.z - (n.right_utm.z - u0))),
        std::abs(n.center_enu.z - (n.center_utm.z - u0)));

      const double e_xy = std::max(std::max(el, er), ec);
      const double e = std::max(e_xy, ez);
      if (!std::isfinite(e) || e > 1e-6) {
        ++bad_count;
      }
      if (std::isfinite(e)) {
        max_err = std::max(max_err, e);
      }
    }
    return bad_count == 0U;
  }

  // ----- TF lookup -----------------------------------------------------------

  PoseState pose_from_transform(const geometry_msgs::msg::TransformStamped & tf) const
  {
    PoseState pose;
    pose.valid = true;
    pose.stamp = rclcpp::Time(tf.header.stamp);
    pose.map_x = tf.transform.translation.x;
    pose.map_y = tf.transform.translation.y;
    pose.map_z = tf.transform.translation.z;
    const auto & q = tf.transform.rotation;
    pose.yaw = yaw_from_quaternion(q.w, q.x, q.y, q.z);
    return pose;
  }

  // Prefer exact timestamp TF. If it is unavailable, try latest TF but only
  // accept it while it is fresh enough for the current cloud stamp.
  TfPoseLookup lookup_ego_pose(const rclcpp::Time & cloud_stamp)
  {
    TfPoseLookup result;
    if (!tf_buffer_) {
      result.reason = "tf_buffer_not_ready";
      return result;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, cloud_stamp, tf2::durationFromSec(kTfLookupTimeoutSec));
      result.reason = "exact";
    } catch (const tf2::TransformException &) {
      try {
        tf = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(kTfLookupTimeoutSec));
        result.reason = "latest";
      } catch (const tf2::TransformException &) {
        result.reason = "tf_unavailable";
        return result;
      }
    }

    result.pose = pose_from_transform(tf);
    if (!result.pose.valid || result.pose.stamp.nanoseconds() == 0) {
      result.reason = "invalid_tf_stamp";
      return result;
    }

    result.age_sec = std::abs((cloud_stamp - result.pose.stamp).seconds());
    if (result.age_sec <= kTfFreshAgeSec) {
      result.usable = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s[TF OK] age=%.0fms source=%s%s",
        kColorGreen, result.age_sec * 1000.0, result.reason.c_str(), kColorReset);
      return result;
    }

    if (result.age_sec <= kTfWarnAgeSec) {
      result.usable = true;
      result.warn_stale = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s[TF WARN] age=%.0fms using fallback source=%s%s",
        kColorOrange, result.age_sec * 1000.0, result.reason.c_str(), kColorReset);
      return result;
    }

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "%s[TF FAIL] age=%.0fms > %.0fms source=%s%s",
      kColorRed, result.age_sec * 1000.0, kTfWarnAgeSec * 1000.0, result.reason.c_str(), kColorReset);
    result.reason = "tf_too_old";
    return result;
  }

  // If TF is unusable, do not pass map-dependent classifications downstream.
  // Publishing raw clusters here creates false tracks during rosbag startup or
  // loop restarts, so fail closed by emitting empty obstacle clouds.
  void publish_fail_closed_empty(
    const sensor_msgs::msg::PointCloud2 & msg,
    const FieldMeta & x_meta,
    const FieldMeta & y_meta,
    const FieldMeta & z_meta,
    const FieldMeta & cluster_meta,
    const std::string & reason)
  {
    auto & aggregates = aggregates_buf_;
    auto & valid_points = valid_points_buf_;
    auto & points_by_cluster = points_by_cluster_buf_;
    auto & in_obstacles = in_obstacles_buf_;
    auto & out_obstacles = out_obstacles_buf_;
    auto & in_repr = in_repr_buf_;
    auto & out_repr = out_repr_buf_;

    aggregates.clear();
    valid_points.clear();
    points_by_cluster.clear();
    in_obstacles.clear();
    out_obstacles.clear();
    in_repr.clear();
    out_repr.clear();

    pub_obstacle_in_->publish(build_reindexed_cloud(
        msg, in_obstacles, points_by_cluster, x_meta, y_meta, z_meta, cluster_meta));
    pub_obstacle_out_->publish(build_reindexed_cloud(
        msg, out_obstacles, points_by_cluster, x_meta, y_meta, z_meta, cluster_meta));

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "%s[FAIL-CLOSED] %s; tf-dependent output blocked%s",
      kColorRed, reason.c_str(), kColorReset);
  }

  // ----- ROS callback --------------------------------------------------------

  bool use_far_filter(double ego_distance_m) const
  {
    return ego_distance_m >= far_filter_range_m_;
  }

  double in_radius_for_distance(double ego_distance_m) const
  {
    return use_far_filter(ego_distance_m) ? far_in_radius_m_ : in_radius_m_;
  }

  double out_radius_for_distance(double ego_distance_m) const
  {
    return use_far_filter(ego_distance_m) ? far_out_radius_m_ : out_radius_m_;
  }

  double inside_ratio_for_distance(double ego_distance_m) const
  {
    return use_far_filter(ego_distance_m) ?
           far_inside_point_ratio_threshold_ :
           inside_point_ratio_threshold_;
  }

  void on_cluster_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const rclcpp::Time cloud_stamp(msg->header.stamp);
    FieldMeta meta_x;
    FieldMeta meta_y;
    FieldMeta meta_z;
    FieldMeta meta_cluster;

    for (const auto & field : msg->fields) {
      if (field.name == "x") {
        meta_x.offset = static_cast<int>(field.offset);
        meta_x.datatype = field.datatype;
      } else if (field.name == "y") {
        meta_y.offset = static_cast<int>(field.offset);
        meta_y.datatype = field.datatype;
      } else if (field.name == "z") {
        meta_z.offset = static_cast<int>(field.offset);
        meta_z.datatype = field.datatype;
      } else if (field.name == cluster_field_name_ || field.name == "cluster") {
        meta_cluster.offset = static_cast<int>(field.offset);
        meta_cluster.datatype = field.datatype;
      }
    }

    if (meta_x.offset < 0 || meta_y.offset < 0 || meta_z.offset < 0 || meta_cluster.offset < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input cloud missing x/y/z/cluster_id fields");
      return;
    }

    const int n_points = static_cast<int>(msg->width * msg->height);
    if (n_points <= 0) {
      return;
    }

    const std::size_t point_step = static_cast<std::size_t>(msg->point_step);
    if (point_step == 0U || msg->data.size() < static_cast<std::size_t>(n_points) * point_step) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid input PointCloud2 layout");
      return;
    }

    const auto tf_lookup = lookup_ego_pose(cloud_stamp);
    PoseState pose;
    if (tf_lookup.usable) {
      pose = tf_lookup.pose;
      last_good_pose_ = pose;
      last_good_pose_cached_at_ = now();
    } else {
      const double cache_age_sec = last_good_pose_.valid
        ? (now() - last_good_pose_cached_at_).seconds()
        : std::numeric_limits<double>::infinity();
      if (last_good_pose_.valid && cache_age_sec <= kTfCacheFallbackMaxAgeSec) {
        pose = last_good_pose_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 500,
          "%s[TF CACHE] %s; reusing cached pose age=%.0fms%s",
          kColorOrange, tf_lookup.reason.c_str(), cache_age_sec * 1000.0, kColorReset);
      } else {
        publish_fail_closed_empty(*msg, meta_x, meta_y, meta_z, meta_cluster, tf_lookup.reason);
        return;
      }
    }

    obstacle_filtering::WaypointWindow wp_window;
    const bool waypoint_window_ok = waypoint_window_use_lidar_roi_ ?
      waypoint_graph_.make_lidar_roi_window(
        pose.map_x,
        pose.map_y,
        pose.yaw,
        base_to_lidar_x_m_,
        base_to_lidar_y_m_,
        waypoint_roi_min_x_m_,
        waypoint_roi_max_x_m_,
        waypoint_roi_min_y_m_,
        waypoint_roi_max_y_m_,
        wp_window) :
      waypoint_graph_.make_vehicle_window(
        pose.map_x, pose.map_y, rear_waypoint_count_, front_waypoint_count_, wp_window);
    if (!waypoint_window_ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waypoint %s window generation failed",
        waypoint_window_use_lidar_roi_ ? "ROI" : "count");
      return;
    }

    const auto left_dense = waypoint_graph_.left_points_dense(wp_window, waypoint_interp_step_m_);
    const auto right_dense = waypoint_graph_.right_points_dense(wp_window, waypoint_interp_step_m_);
    if (left_dense.empty() && right_dense.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No waypoint points in current window");
      return;
    }
    left_kd_.build(left_dense);
    right_kd_.build(right_dense);

    // 1) Aggregate point-level statistics by source cluster_id.
    auto & aggregates = aggregates_buf_;
    auto & valid_points = valid_points_buf_;
    aggregates.clear();
    valid_points.clear();

    // 2) Aggregate cluster statistics and keep valid source point indices for later cloud reconstruction.
    //
    // Road membership is counted per source cluster_id: each point votes in if it
    // is close to either dense L1/R1 waypoint line. This is easy to tune, but it
    // assumes DBSCAN did not merge a road object with off-road points. If merged
    // clusters become common, prefer a stricter representative check, split by
    // road membership before L-shape fitting, or raise inside_point_ratio_threshold_.
    for (int i = 0; i < n_points; ++i) {
      const uint8_t * ptr = &msg->data[static_cast<std::size_t>(i) * point_step];

      float lx = 0.0F;
      float ly = 0.0F;
      float lz = 0.0F;
      int32_t cid = -1;
      if (!read_float_field(ptr, meta_x, lx) || !read_float_field(ptr, meta_y, ly) ||
        !read_float_field(ptr, meta_z, lz) || !read_int32_field(ptr, meta_cluster, cid))
      {
        continue;
      }
      if (!std::isfinite(lx) || !std::isfinite(ly) || !std::isfinite(lz) || cid < 0) {
        continue;
      }

      const double bx = static_cast<double>(lx) + base_to_lidar_x_m_;
      const double by = static_cast<double>(ly) + base_to_lidar_y_m_;
      const double bz = static_cast<double>(lz) + base_to_lidar_z_m_;

      double rmx = 0.0;
      double rmy = 0.0;
      rotate_xy(pose.yaw, bx, by, rmx, rmy);
      const double mx = pose.map_x + rmx;
      const double my = pose.map_y + rmy;
      const float point_d2_left = left_kd_.nearest_sq(static_cast<float>(mx), static_cast<float>(my));
      const float point_d2_right = right_kd_.nearest_sq(static_cast<float>(mx), static_cast<float>(my));
      const double point_nearest_wp_distance = std::sqrt(std::min(point_d2_left, point_d2_right));
      const double point_ego_distance = std::hypot(bx, by);
      const double point_in_radius = in_radius_for_distance(point_ego_distance);

      auto & agg = aggregates[cid];
      ++agg.total;
      if (point_nearest_wp_distance <= point_in_radius) {
        ++agg.inside_count;
      }
      agg.sum_lx += static_cast<double>(lx);
      agg.sum_ly += static_cast<double>(ly);
      agg.sum_lz += static_cast<double>(lz);
      agg.min_by = std::min(agg.min_by, by);
      agg.max_by = std::max(agg.max_by, by);
      agg.min_bz = std::min(agg.min_bz, bz);
      agg.max_bz = std::max(agg.max_bz, bz);
      valid_points.push_back(ValidPoint{i, cid});
    }

    // 3) Build index lists so selected clusters can be reconstructed as compact
    // output clouds with new distance-sorted ids.
    auto & points_by_cluster = points_by_cluster_buf_;
    points_by_cluster.clear();
    for (const auto & vp : valid_points) {
      points_by_cluster[vp.cluster_id].push_back(vp.index);
    }

    auto & in_obstacles = in_obstacles_buf_;
    auto & out_obstacles = out_obstacles_buf_;
    in_obstacles.clear();
    out_obstacles.clear();

    auto & in_repr = in_repr_buf_;
    auto & out_repr = out_repr_buf_;
    in_repr.clear();
    out_repr.clear();

    // 4) Cluster-level in/out decision from representative(mean) point plus point ratio.
    // A cluster is accepted only when its mean point is within in_radius_m and
    // enough of its raw points are also within the many L1/R1 waypoint-radius
    // circles. The out_radius_m band keeps prior state to reduce boundary flicker.
    for (const auto & kv : aggregates) {
      const int32_t source_cluster_id = kv.first;
      const ClusterAggregate & agg = kv.second;
      if (agg.total <= 0) {
        continue;
      }

      const double rep_lx = agg.sum_lx / static_cast<double>(agg.total);
      const double rep_ly = agg.sum_ly / static_cast<double>(agg.total);
      const double rep_lz = agg.sum_lz / static_cast<double>(agg.total);

      const double rep_bx = rep_lx + base_to_lidar_x_m_;
      const double rep_by = rep_ly + base_to_lidar_y_m_;
      const double rep_bz = rep_lz + base_to_lidar_z_m_;

      const double max_range_sq = max_detection_range_m_ * max_detection_range_m_;
      const double d2_ego = rep_bx * rep_bx + rep_by * rep_by;
      if (max_detection_range_m_ > 0.0 && d2_ego > max_range_sq) {
        continue;
      }

      const double dist_ego = std::sqrt(d2_ego);
      if (dist_ego < min_obstacle_distance_m_) {
        continue;
      }

      double rbmx = 0.0;
      double rbmy = 0.0;
      rotate_xy(pose.yaw, rep_bx, rep_by, rbmx, rbmy);

      const double rep_mx = pose.map_x + rbmx;
      const double rep_my = pose.map_y + rbmy;
      const double rep_mz = pose.map_z + rep_bz;

      const float d2_left = left_kd_.nearest_sq(static_cast<float>(rep_mx), static_cast<float>(rep_my));
      const float d2_right = right_kd_.nearest_sq(static_cast<float>(rep_mx), static_cast<float>(rep_my));

      const double nearest_wp_distance = std::sqrt(std::min(d2_left, d2_right));
      const double inside_point_ratio =
        static_cast<double>(agg.inside_count) / static_cast<double>(agg.total);
      const double filter_in_radius = in_radius_for_distance(dist_ego);
      const double filter_out_radius = out_radius_for_distance(dist_ego);
      const double filter_ratio_threshold = inside_ratio_for_distance(dist_ego);
      const bool ratio_pass = inside_point_ratio >= filter_ratio_threshold;
      bool is_inside = false;
      const auto it_in_state = in_state_by_cluster_id_.find(source_cluster_id);
      if (nearest_wp_distance <= filter_in_radius && ratio_pass) {
        is_inside = true;
      } else if (nearest_wp_distance >= filter_out_radius || !ratio_pass) {
        is_inside = false;
      } else if (it_in_state != in_state_by_cluster_id_.end()) {
        is_inside = it_in_state->second;
      }

      ObstacleSummary info;
      info.source_cluster_id = source_cluster_id;
      info.map_x = rep_mx;
      info.map_y = rep_my;
      info.map_z = rep_mz;
      info.base_x = rep_bx;
      info.base_y = rep_by;
      info.base_z = rep_bz;
      info.distance = dist_ego;
      info.width = std::max(0.0, agg.max_by - agg.min_by);
      info.height = std::max(0.0, agg.max_bz - agg.min_bz);
      info.nearest_wp_distance = nearest_wp_distance;
      info.nearest_left_distance = std::sqrt(d2_left);
      info.nearest_right_distance = std::sqrt(d2_right);
      info.point_count = static_cast<uint32_t>(agg.total);
      info.inside_point_ratio = inside_point_ratio;

      double vx = 0.0;
      double vy = 0.0;
      double heading_base = 0.0;
      double heading_map = 0.0;
      bool has_prev_heading = false;

      const auto it_prev = prev_states_.find(source_cluster_id);
      if (it_prev != prev_states_.end() && it_prev->second.valid) {
        const double dt = (pose.stamp - it_prev->second.stamp).seconds();
        if (dt > 1e-3) {
          vx = (rep_bx - it_prev->second.x) / dt;
          vy = (rep_by - it_prev->second.y) / dt;
        }
        heading_base = it_prev->second.heading_base;
        heading_map = it_prev->second.heading_map;
        has_prev_heading = true;
      }

      info.velocity_x = vx;
      info.velocity_y = vy;
      info.speed = std::hypot(vx, vy);

      // Heading is obstacle-motion heading, not ego yaw.
      // When speed is tiny, keep previous heading to reduce jitter.
      constexpr double kHeadingSpeedThresh = 0.2;
      if (info.speed > kHeadingSpeedThresh) {
        info.heading_base = std::atan2(vy, vx);
        double vmx = 0.0;
        double vmy = 0.0;
        rotate_xy(pose.yaw, vx, vy, vmx, vmy);
        info.heading_map = std::atan2(vmy, vmx);
      } else if (has_prev_heading) {
        info.heading_base = heading_base;
        info.heading_map = heading_map;
      } else {
        info.heading_base = 0.0;
        info.heading_map = 0.0;
      }

      ClusterState st;
      st.x = rep_bx;
      st.y = rep_by;
      st.stamp = pose.stamp;
      st.heading_base = info.heading_base;
      st.heading_map = info.heading_map;
      st.valid = true;
      prev_states_[source_cluster_id] = st;
      in_state_by_cluster_id_[source_cluster_id] = is_inside;

      if (is_inside) {
        in_obstacles.push_back(info);
      } else {
        out_obstacles.push_back(info);
      }
    }

    // 5) Sort by ego distance, reindex, and publish runtime/debug outputs.
    const auto by_distance = [](const ObstacleSummary & a, const ObstacleSummary & b) {
      return a.distance < b.distance;
    };
    std::sort(in_obstacles.begin(), in_obstacles.end(), by_distance);
    std::sort(out_obstacles.begin(), out_obstacles.end(), by_distance);

    for (std::size_t i = 0; i < in_obstacles.size(); ++i) {
      in_obstacles[i].id = static_cast<int32_t>(i);
      in_repr.push_back(OutputPoint{
        static_cast<float>(in_obstacles[i].map_x),
        static_cast<float>(in_obstacles[i].map_y),
        static_cast<float>(in_obstacles[i].map_z),
        in_obstacles[i].id,
        static_cast<float>(in_obstacles[i].distance)});
    }
    for (std::size_t i = 0; i < out_obstacles.size(); ++i) {
      out_obstacles[i].id = static_cast<int32_t>(i);
      out_repr.push_back(OutputPoint{
        static_cast<float>(out_obstacles[i].map_x),
        static_cast<float>(out_obstacles[i].map_y),
        static_cast<float>(out_obstacles[i].map_z),
        out_obstacles[i].id,
        static_cast<float>(out_obstacles[i].distance)});
    }

    pub_obstacle_in_->publish(build_reindexed_cloud(
        *msg, in_obstacles, points_by_cluster, meta_x, meta_y, meta_z, meta_cluster));
    pub_obstacle_out_->publish(build_reindexed_cloud(
        *msg, out_obstacles, points_by_cluster, meta_x, meta_y, meta_z, meta_cluster));

    if (publish_debug_text_ && pub_obstacle_in_debug_ && pub_obstacle_out_debug_) {
      std_msgs::msg::String in_debug_msg;
      std_msgs::msg::String out_debug_msg;
      in_debug_msg.data = to_pretty_text(msg->header.stamp, in_obstacles);
      out_debug_msg.data = to_pretty_text(msg->header.stamp, out_obstacles);
      pub_obstacle_in_debug_->publish(in_debug_msg);
      pub_obstacle_out_debug_->publish(out_debug_msg);
    }

    if (publish_debug_cloud_ && pub_obstacle_in_cloud_ && pub_obstacle_out_cloud_) {
      pub_obstacle_in_cloud_->publish(build_cloud(msg->header.stamp, map_frame_, in_repr));
      pub_obstacle_out_cloud_->publish(build_cloud(msg->header.stamp, map_frame_, out_repr));
    }
  }

  // ----- PointCloud2 builders ------------------------------------------------

  sensor_msgs::msg::PointCloud2 build_cloud(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & frame_id,
    const std::vector<OutputPoint> & points) const
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_bigendian = false;
    out.is_dense = false;

    sensor_msgs::msg::PointField fx;
    fx.name = "x";
    fx.offset = 0;
    fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fx.count = 1;

    sensor_msgs::msg::PointField fy;
    fy.name = "y";
    fy.offset = 4;
    fy.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fy.count = 1;

    sensor_msgs::msg::PointField fz;
    fz.name = "z";
    fz.offset = 8;
    fz.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fz.count = 1;

    sensor_msgs::msg::PointField fid;
    fid.name = "id";
    fid.offset = 12;
    fid.datatype = sensor_msgs::msg::PointField::INT32;
    fid.count = 1;

    sensor_msgs::msg::PointField fval;
    fval.name = "distance";
    fval.offset = 16;
    fval.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fval.count = 1;

    out.fields = {fx, fy, fz, fid, fval};
    out.point_step = 20;
    out.row_step = out.point_step * out.width;
    out.data.resize(static_cast<std::size_t>(out.row_step) * out.height);

    uint8_t * data_ptr = out.data.data();
    for (std::size_t i = 0; i < points.size(); ++i) {
      const std::size_t offset = i * out.point_step;
      std::memcpy(data_ptr + offset + 0, &points[i].x, sizeof(float));
      std::memcpy(data_ptr + offset + 4, &points[i].y, sizeof(float));
      std::memcpy(data_ptr + offset + 8, &points[i].z, sizeof(float));
      std::memcpy(data_ptr + offset + 12, &points[i].id, sizeof(int32_t));
      std::memcpy(data_ptr + offset + 16, &points[i].value, sizeof(float));
    }

    return out;
  }

  // Copy all points belonging to selected source clusters. The output cluster
  // field is rewritten to the distance-sorted obstacle id used downstream.
  sensor_msgs::msg::PointCloud2 build_reindexed_cloud(
    const sensor_msgs::msg::PointCloud2 & src,
    const std::vector<ObstacleSummary> & obstacles,
    const std::unordered_map<int32_t, std::vector<int>> & points_by_cluster,
    const FieldMeta & x_meta,
    const FieldMeta & y_meta,
    const FieldMeta & z_meta,
    const FieldMeta & cluster_meta) const
  {
    std::size_t total_selected = 0U;
    for (const auto & o : obstacles) {
      const auto it_points = points_by_cluster.find(o.source_cluster_id);
      if (it_points != points_by_cluster.end()) {
        total_selected += it_points->second.size();
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = src.header;
    out.header.frame_id = base_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(total_selected);
    out.fields = src.fields;
    out.is_bigendian = src.is_bigendian;
    out.is_dense = src.is_dense;
    out.point_step = src.point_step;
    out.row_step = out.point_step * out.width;
    out.data.resize(static_cast<std::size_t>(out.row_step) * out.height);

    const std::size_t src_step = static_cast<std::size_t>(src.point_step);
    const bool fast_path =
      x_meta.offset >= 0 && y_meta.offset >= 0 && z_meta.offset >= 0 && cluster_meta.offset >= 0 &&
      x_meta.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      y_meta.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      z_meta.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      cluster_meta.datatype == sensor_msgs::msg::PointField::INT32 &&
      static_cast<std::size_t>(x_meta.offset + static_cast<int>(sizeof(float))) <= src_step &&
      static_cast<std::size_t>(y_meta.offset + static_cast<int>(sizeof(float))) <= src_step &&
      static_cast<std::size_t>(z_meta.offset + static_cast<int>(sizeof(float))) <= src_step &&
      static_cast<std::size_t>(cluster_meta.offset + static_cast<int>(sizeof(int32_t))) <= src_step;

    const float dx = static_cast<float>(base_to_lidar_x_m_);
    const float dy = static_cast<float>(base_to_lidar_y_m_);
    const float dz = static_cast<float>(base_to_lidar_z_m_);

    std::size_t out_index = 0U;
    for (const auto & o : obstacles) {
      const auto it_points = points_by_cluster.find(o.source_cluster_id);
      if (it_points == points_by_cluster.end()) {
        continue;
      }

      for (const int src_index : it_points->second) {
        const std::size_t src_offset = static_cast<std::size_t>(src_index) * src_step;
        const std::size_t dst_offset = out_index * src_step;
        std::memcpy(out.data.data() + dst_offset, src.data.data() + src_offset, src_step);

        uint8_t * const dst_ptr = out.data.data() + dst_offset;

        if (fast_path) {
          float x = 0.0F;
          float y = 0.0F;
          float z = 0.0F;
          std::memcpy(&x, dst_ptr + x_meta.offset, sizeof(float));
          std::memcpy(&y, dst_ptr + y_meta.offset, sizeof(float));
          std::memcpy(&z, dst_ptr + z_meta.offset, sizeof(float));
          x += dx;
          y += dy;
          z += dz;
          std::memcpy(dst_ptr + x_meta.offset, &x, sizeof(float));
          std::memcpy(dst_ptr + y_meta.offset, &y, sizeof(float));
          std::memcpy(dst_ptr + z_meta.offset, &z, sizeof(float));
          std::memcpy(dst_ptr + cluster_meta.offset, &o.id, sizeof(int32_t));
        } else {
          // Convert point position from LiDAR frame to base_link frame for /obstacle_in,/obstacle_out.
          float x = 0.0F;
          float y = 0.0F;
          float z = 0.0F;
          if (read_float_field(dst_ptr, x_meta, x) &&
            read_float_field(dst_ptr, y_meta, y) &&
            read_float_field(dst_ptr, z_meta, z))
          {
            x += dx;
            y += dy;
            z += dz;
            (void)write_float_field(dst_ptr, x_meta, x);
            (void)write_float_field(dst_ptr, y_meta, y);
            (void)write_float_field(dst_ptr, z_meta, z);
          }
          (void)write_int32_field(dst_ptr, cluster_meta, o.id);
        }
        ++out_index;
      }
    }
    return out;
  }

  // ----- Map/corridor configuration -----------------------------------------

  obstacle_filtering::WaypointGraph waypoint_graph_;

  std::string base_frame_;
  std::string map_frame_;
  std::string cluster_field_name_;

  int front_waypoint_count_{280};
  int rear_waypoint_count_{280};

  double waypoint_interp_step_m_{0.3};
  bool waypoint_window_use_lidar_roi_{true};
  double waypoint_roi_min_x_m_{-30.0};
  double waypoint_roi_max_x_m_{50.0};
  double waypoint_roi_min_y_m_{-20.0};
  double waypoint_roi_max_y_m_{20.0};
  double max_detection_range_m_{80.0};
  double min_obstacle_distance_m_{1.5};
  double in_radius_m_{1.8};
  double out_radius_m_{2.1};
  double far_filter_range_m_{12.0};
  double far_in_radius_m_{2.6};
  double far_out_radius_m_{3.0};
  double inside_point_ratio_threshold_{0.20};
  double far_inside_point_ratio_threshold_{0.10};

  // ----- Sensor-frame calibration -------------------------------------------

  double base_to_lidar_x_m_{-0.2};
  double base_to_lidar_y_m_{0.0};
  double base_to_lidar_z_m_{0.57};

  bool origin_ready_{false};
  double origin_e0_{0.0};
  double origin_n0_{0.0};
  double origin_u0_{0.0};
  bool waypoint_subtraction_ok_{false};
  std::size_t waypoint_subtraction_bad_count_{0U};
  double waypoint_subtraction_max_error_{std::numeric_limits<double>::infinity()};

  // ----- Debug output switches ----------------------------------------------

  bool publish_debug_text_{false};
  bool publish_debug_cloud_{true};

  // ----- TF pose cache for brief-outage fallback ----------------------------

  PoseState last_good_pose_;
  rclcpp::Time last_good_pose_cached_at_{0, 0, RCL_ROS_TIME};

  // ----- Per-frame reusable buffers/state -----------------------------------

  KdTree2D left_kd_;
  KdTree2D right_kd_;

  std::unordered_map<int32_t, ClusterAggregate> aggregates_buf_;
  std::vector<ValidPoint> valid_points_buf_;
  std::unordered_map<int32_t, std::vector<int>> points_by_cluster_buf_;
  std::vector<ObstacleSummary> in_obstacles_buf_;
  std::vector<ObstacleSummary> out_obstacles_buf_;
  std::unordered_map<int32_t, bool> in_state_by_cluster_id_;
  std::unordered_map<int32_t, ClusterState> prev_states_;
  std::vector<OutputPoint> in_repr_buf_;
  std::vector<OutputPoint> out_repr_buf_;

  // ----- ROS interfaces ------------------------------------------------------

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_in_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_obstacle_in_debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_obstacle_out_debug_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_in_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_out_cloud_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleFilteringNode>());
  rclcpp::shutdown();
  return 0;
}
