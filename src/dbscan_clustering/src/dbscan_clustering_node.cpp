#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "dbscan_clustering/dbscan_gpu.cuh"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using std::placeholders::_1;

class DbscanClusteringNode : public rclcpp::Node {
public:
  explicit DbscanClusteringNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("dbscan_clustering", options)
  {
    declare_parameter<std::string>("input_topic", "/patchworkpp/nonground");
    declare_parameter<std::string>("output_topic", "/pointcloud/clustered");
    declare_parameter<double>("eps", 0.35);
    declare_parameter<int>("min_points", 12);
    declare_parameter<int>("max_neighbors", 256);
    declare_parameter<int>("max_points", 200000);
    declare_parameter<int>("queue_size", 1);

    declare_parameter<double>("roi_min_x", 0.5);
    declare_parameter<double>("roi_max_x", 20.0);
    declare_parameter<double>("roi_min_y", -10.0);
    declare_parameter<double>("roi_max_y", 10.0);
    declare_parameter<double>("roi_min_z", -2.0);
    declare_parameter<double>("roi_max_z", 2.0);

    declare_parameter<bool>("enable_ego_box_filter", false);
    declare_parameter<bool>("ego_box_xy_only", true);
    declare_parameter<double>("ego_min_x", -2.0);
    declare_parameter<double>("ego_max_x", 2.0);
    declare_parameter<double>("ego_min_y", -1.0);
    declare_parameter<double>("ego_max_y", 1.0);
    declare_parameter<double>("ego_min_z", -3.0);
    declare_parameter<double>("ego_max_z", 1.0);
    declare_parameter<double>("ego_box_offset_x", 0.0);
    declare_parameter<double>("ego_box_offset_y", 0.0);
    declare_parameter<double>("ego_box_offset_z", 0.0);

    declare_parameter<bool>("cluster_xy_only", true);
    declare_parameter<double>("cluster_scale_x", 1.0);
    declare_parameter<double>("cluster_scale_y", 1.0);
    declare_parameter<double>("cluster_scale_z", 1.0);
    declare_parameter<double>("xy_voxel_leaf_size", 0.0);
    declare_parameter<bool>("enable_residual_ground_filter", true);
    declare_parameter<double>("residual_ground_filter_range_min_m", 3.0);
    declare_parameter<double>("residual_ground_z_base_m", -0.62);
    declare_parameter<double>("residual_ground_z_slope_per_m", 0.010);

    const auto in_topic = get_parameter("input_topic").as_string();
    const auto out_topic = get_parameter("output_topic").as_string();
    eps_ = get_parameter("eps").as_double();
    min_points_ = get_parameter("min_points").as_int();
    max_neighbors_ = get_parameter("max_neighbors").as_int();
    max_points_ = get_parameter("max_points").as_int();
    const int queue_size = get_parameter("queue_size").as_int();

    roi_min_x_ = get_parameter("roi_min_x").as_double();
    roi_max_x_ = get_parameter("roi_max_x").as_double();
    roi_min_y_ = get_parameter("roi_min_y").as_double();
    roi_max_y_ = get_parameter("roi_max_y").as_double();
    roi_min_z_ = get_parameter("roi_min_z").as_double();
    roi_max_z_ = get_parameter("roi_max_z").as_double();

    enable_ego_box_filter_ = get_parameter("enable_ego_box_filter").as_bool();
    ego_box_xy_only_ = get_parameter("ego_box_xy_only").as_bool();
    ego_min_x_ = get_parameter("ego_min_x").as_double();
    ego_max_x_ = get_parameter("ego_max_x").as_double();
    ego_min_y_ = get_parameter("ego_min_y").as_double();
    ego_max_y_ = get_parameter("ego_max_y").as_double();
    ego_min_z_ = get_parameter("ego_min_z").as_double();
    ego_max_z_ = get_parameter("ego_max_z").as_double();
    ego_box_offset_x_ = get_parameter("ego_box_offset_x").as_double();
    ego_box_offset_y_ = get_parameter("ego_box_offset_y").as_double();
    ego_box_offset_z_ = get_parameter("ego_box_offset_z").as_double();

    cluster_xy_only_ = get_parameter("cluster_xy_only").as_bool();
    cluster_scale_x_ = get_parameter("cluster_scale_x").as_double();
    cluster_scale_y_ = get_parameter("cluster_scale_y").as_double();
    cluster_scale_z_ = get_parameter("cluster_scale_z").as_double();
    xy_voxel_leaf_size_ = get_parameter("xy_voxel_leaf_size").as_double();
    enable_residual_ground_filter_ = get_parameter("enable_residual_ground_filter").as_bool();
    residual_ground_filter_range_min_m_ =
      get_parameter("residual_ground_filter_range_min_m").as_double();
    residual_ground_z_base_m_ = get_parameter("residual_ground_z_base_m").as_double();
    residual_ground_z_slope_per_m_ = get_parameter("residual_ground_z_slope_per_m").as_double();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic,
      rclcpp::QoS(queue_size),
      std::bind(&DbscanClusteringNode::cloud_callback, this, _1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out_topic, rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "dbscan_clustering started in:%s out:%s eps:%.3f min_points:%d ROI x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f] xy_only:%s scale[x:%.2f y:%.2f z:%.2f] xy_voxel:%.2f",
      in_topic.c_str(), out_topic.c_str(), eps_, min_points_,
      roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_,
      cluster_xy_only_ ? "on" : "off",
      cluster_scale_x_, cluster_scale_y_, cluster_scale_z_, xy_voxel_leaf_size_);
    if (enable_residual_ground_filter_) {
      RCLCPP_INFO(
        get_logger(),
        "residual ground filter enabled range>=%.1fm z<=%.3f + %.4f*(range-%.1f)",
        residual_ground_filter_range_min_m_,
        residual_ground_z_base_m_,
        residual_ground_z_slope_per_m_,
        residual_ground_filter_range_min_m_);
    }
    if (enable_ego_box_filter_) {
      RCLCPP_INFO(
        get_logger(),
        "ego box mask enabled xy_only:%s box x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] offset[%.2f,%.2f,%.2f]",
        ego_box_xy_only_ ? "on" : "off",
        ego_min_x_, ego_max_x_, ego_min_y_, ego_max_y_, ego_min_z_, ego_max_z_,
        ego_box_offset_x_, ego_box_offset_y_, ego_box_offset_z_);
    }
  }

private:
  struct CandidatePoint
  {
    float x;
    float y;
    float z;
  };

  struct FieldMeta
  {
    int offset{-1};
    uint8_t datatype{0};
  };

  static uint64_t pack_voxel_key(int32_t ix, int32_t iy)
  {
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
      static_cast<uint32_t>(iy);
  }

  std::vector<CandidatePoint> apply_xy_voxel_filter(
    const std::vector<CandidatePoint> & points) const
  {
    if (xy_voxel_leaf_size_ <= 0.0 || points.empty()) {
      return points;
    }

    const float leaf = static_cast<float>(xy_voxel_leaf_size_);
    const float inv_leaf = 1.0F / leaf;

    std::vector<CandidatePoint> filtered;
    filtered.reserve(points.size());

    std::vector<float> representative_dist2;
    representative_dist2.reserve(points.size());

    std::unordered_map<uint64_t, size_t> voxel_to_index;
    voxel_to_index.reserve(points.size());

    for (const auto & point : points) {
      const int32_t ix = static_cast<int32_t>(std::floor(point.x * inv_leaf));
      const int32_t iy = static_cast<int32_t>(std::floor(point.y * inv_leaf));
      const uint64_t key = pack_voxel_key(ix, iy);

      const float center_x = (static_cast<float>(ix) + 0.5F) * leaf;
      const float center_y = (static_cast<float>(iy) + 0.5F) * leaf;
      const float dx = point.x - center_x;
      const float dy = point.y - center_y;
      const float dist2 = dx * dx + dy * dy;

      const auto [it, inserted] = voxel_to_index.emplace(key, filtered.size());
      if (inserted) {
        filtered.push_back(point);
        representative_dist2.push_back(dist2);
        continue;
      }

      if (dist2 < representative_dist2[it->second]) {
        filtered[it->second] = point;
        representative_dist2[it->second] = dist2;
      }
    }

    return filtered;
  }

  static bool read_float_field(
    const uint8_t * point_ptr,
    const FieldMeta & meta,
    float & out)
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

  bool should_reject_residual_ground(const CandidatePoint & point) const
  {
    if (!enable_residual_ground_filter_) {
      return false;
    }

    const double range_xy = std::hypot(static_cast<double>(point.x), static_cast<double>(point.y));
    if (range_xy < residual_ground_filter_range_min_m_) {
      return false;
    }

    const double z_threshold =
      residual_ground_z_base_m_ +
      residual_ground_z_slope_per_m_ * (range_xy - residual_ground_filter_range_min_m_);
    return static_cast<double>(point.z) <= z_threshold;
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    using Clock = std::chrono::steady_clock;
    const auto t_begin = Clock::now();

    int n_points = static_cast<int>(msg->width * msg->height);
    if (n_points <= 0) {
      return;
    }
    if (n_points > max_points_) {
      n_points = max_points_;
    }

    FieldMeta meta_x;
    FieldMeta meta_y;
    FieldMeta meta_z;

    for (const auto & f : msg->fields) {
      if (f.name == "x") {
        meta_x.offset = static_cast<int>(f.offset);
        meta_x.datatype = f.datatype;
      } else if (f.name == "y") {
        meta_y.offset = static_cast<int>(f.offset);
        meta_y.datatype = f.datatype;
      } else if (f.name == "z") {
        meta_z.offset = static_cast<int>(f.offset);
        meta_z.datatype = f.datatype;
      }
    }

    if (meta_x.offset < 0 || meta_y.offset < 0 || meta_z.offset < 0) {
      RCLCPP_ERROR(get_logger(), "PointCloud2 missing x/y/z fields");
      return;
    }

    const size_t point_step = static_cast<size_t>(msg->point_step);
    if (point_step == 0 || msg->data.size() < static_cast<size_t>(n_points) * point_step) {
      RCLCPP_WARN(get_logger(), "Invalid PointCloud2 layout");
      return;
    }

    std::vector<CandidatePoint> roi_points;
    roi_points.reserve(static_cast<size_t>(n_points));
    size_t ego_masked = 0;
    size_t residual_ground_rejected = 0;

    for (int i = 0; i < n_points; ++i) {
      const uint8_t * point_ptr = &msg->data[static_cast<size_t>(i) * point_step];

      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!read_float_field(point_ptr, meta_x, x) ||
        !read_float_field(point_ptr, meta_y, y) ||
        !read_float_field(point_ptr, meta_z, z))
      {
        continue;
      }

      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      if (x < roi_min_x_ || x > roi_max_x_ ||
        y < roi_min_y_ || y > roi_max_y_ ||
        z < roi_min_z_ || z > roi_max_z_)
      {
        continue;
      }

      if (enable_ego_box_filter_) {
        // Shift by configurable offset so the ego mask can be centered on another reference
        // (e.g., base_link) while points are expressed in the input cloud frame.
        const float x_rel = x - static_cast<float>(ego_box_offset_x_);
        const float y_rel = y - static_cast<float>(ego_box_offset_y_);
        const float z_rel = z - static_cast<float>(ego_box_offset_z_);

        const bool in_xy = (x_rel >= ego_min_x_ && x_rel <= ego_max_x_ &&
          y_rel >= ego_min_y_ && y_rel <= ego_max_y_);
        const bool in_z = (z_rel >= ego_min_z_ && z_rel <= ego_max_z_);
        const bool in_ego_box = ego_box_xy_only_ ? in_xy : (in_xy && in_z);
        if (in_ego_box) {
          ++ego_masked;
          continue;
        }
      }

      const CandidatePoint candidate{x, y, z};
      if (should_reject_residual_ground(candidate)) {
        ++residual_ground_rejected;
        continue;
      }

      roi_points.push_back(candidate);
    }
    const auto t_roi_done = Clock::now();

    if (roi_points.empty()) {
      return;
    }

    const size_t roi_count_before_voxel = roi_points.size();
    std::vector<CandidatePoint> nonground_points = apply_xy_voxel_filter(roi_points);

    const int m = static_cast<int>(nonground_points.size());
    if (m == 0) {
      return;
    }

    const int UNVISITED = -2;
    std::vector<int32_t> labels(static_cast<size_t>(m), UNVISITED);
    int cluster_id = 0;

    std::vector<float> xyz(static_cast<size_t>(m) * 3U);
    for (int i = 0; i < m; ++i) {
      xyz[static_cast<size_t>(3 * i + 0)] =
        nonground_points[static_cast<size_t>(i)].x * static_cast<float>(cluster_scale_x_);
      xyz[static_cast<size_t>(3 * i + 1)] =
        nonground_points[static_cast<size_t>(i)].y * static_cast<float>(cluster_scale_y_);
      xyz[static_cast<size_t>(3 * i + 2)] = cluster_xy_only_ ? 0.0F :
        nonground_points[static_cast<size_t>(i)].z * static_cast<float>(cluster_scale_z_);
    }
    std::vector<int> neighbors(static_cast<size_t>(m) * static_cast<size_t>(max_neighbors_), -1);
    std::vector<int> neighbor_counts(static_cast<size_t>(m), 0);
    const auto t_host_prep_done = Clock::now();
    DbscanGpuTiming gpu_timing;
    dbscan_gpu_query_neighbors_timed(
      xyz.data(), m, static_cast<float>(eps_), max_neighbors_,
      neighbors.data(), neighbor_counts.data(), &gpu_timing);
    const auto t_gpu_done = Clock::now();

    int saturated = 0;
    for (int i = 0; i < m; ++i) {
      if (neighbor_counts[static_cast<size_t>(i)] >= max_neighbors_) {
        ++saturated;
      }
    }
    if (saturated > 0) {
      const double sat_ratio = static_cast<double>(saturated) / static_cast<double>(m);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "neighbor cap saturated: %d/%d (%.1f%%). Increase max_neighbors or lower eps.",
        saturated, m, sat_ratio * 100.0);
    }

    for (int i = 0; i < m; ++i) {
      if (labels[static_cast<size_t>(i)] != UNVISITED) {
        continue;
      }

      if (neighbor_counts[static_cast<size_t>(i)] < min_points_) {
        labels[static_cast<size_t>(i)] = -1;
        continue;
      }

      std::queue<int> q;
      labels[static_cast<size_t>(i)] = cluster_id;
      q.push(i);

      while (!q.empty()) {
        const int cur = q.front();
        q.pop();

        const int cur_count = neighbor_counts[static_cast<size_t>(cur)];
        if (cur_count < min_points_) {
          continue;
        }

        const size_t row_begin = static_cast<size_t>(cur) * static_cast<size_t>(max_neighbors_);
        for (int k = 0; k < cur_count; ++k) {
          const int nb = neighbors[row_begin + static_cast<size_t>(k)];
          if (nb < 0 || nb >= m) {
            continue;
          }
          int32_t & lbl = labels[static_cast<size_t>(nb)];
          if (lbl == UNVISITED) {
            lbl = cluster_id;
            q.push(nb);
          } else if (lbl == -1) {
            lbl = cluster_id;
          }
        }
      }

      ++cluster_id;
    }
    const auto t_cluster_done = Clock::now();

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(m);
    out.is_bigendian = msg->is_bigendian;
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

    sensor_msgs::msg::PointField fcluster;
    fcluster.name = "cluster_id";
    fcluster.offset = 12;
    fcluster.datatype = sensor_msgs::msg::PointField::INT32;
    fcluster.count = 1;

    sensor_msgs::msg::PointField frgb;
    frgb.name = "rgb";
    frgb.offset = 16;
    frgb.datatype = sensor_msgs::msg::PointField::FLOAT32;
    frgb.count = 1;

    out.fields = {fx, fy, fz, fcluster, frgb};
    out.point_step = 20;
    out.row_step = out.point_step * out.width;
    out.data.resize(static_cast<size_t>(out.row_step));

    auto pack_rgb_to_float = [](uint8_t r, uint8_t g, uint8_t b) {
      uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
        (static_cast<uint32_t>(g) << 8) |
        static_cast<uint32_t>(b);
      float f = 0.0F;
      std::memcpy(&f, &rgb, sizeof(float));
      return f;
    };

    auto id_to_color = [](int id) -> std::tuple<uint8_t, uint8_t, uint8_t> {
      if (id < 0) {
        return {128, 128, 128};
      }
      const int hue = (id * 37) % 360;
      const float hf = static_cast<float>(hue) / 60.0F;
      const int sector = static_cast<int>(std::floor(hf)) % 6;
      const float frac = hf - std::floor(hf);
      const float v = 1.0F;
      const float s = 1.0F;
      const float p = v * (1.0F - s);
      const float q = v * (1.0F - s * frac);
      const float t = v * (1.0F - s * (1.0F - frac));
      float r = 0.0F;
      float g = 0.0F;
      float b = 0.0F;
      switch (sector) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
      }
      return {
        static_cast<uint8_t>(std::round(r * 255.0F)),
        static_cast<uint8_t>(std::round(g * 255.0F)),
        static_cast<uint8_t>(std::round(b * 255.0F))};
    };

    sensor_msgs::PointCloud2Iterator<float> out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<int32_t> out_cluster(out, "cluster_id");
    sensor_msgs::PointCloud2Iterator<float> out_rgb(out, "rgb");

    for (int i = 0; i < m; ++i) {
      const auto & p = nonground_points[static_cast<size_t>(i)];
      const int32_t lbl = labels[static_cast<size_t>(i)];
      auto [r, g, b] = id_to_color(lbl);

      *out_x = p.x; ++out_x;
      *out_y = p.y; ++out_y;
      *out_z = p.z; ++out_z;
      *out_cluster = lbl; ++out_cluster;
      *out_rgb = pack_rgb_to_float(r, g, b); ++out_rgb;
    }
    const auto t_pack_done = Clock::now();

    pub_->publish(out);
    const auto t_publish_done = Clock::now();

    const double total_ms =
      std::chrono::duration<double, std::milli>(t_publish_done - t_begin).count();
    const double roi_ms =
      std::chrono::duration<double, std::milli>(t_roi_done - t_begin).count();
    const double cpu_prep_ms =
      std::chrono::duration<double, std::milli>(t_host_prep_done - t_roi_done).count();
    const double gpu_wall_ms =
      std::chrono::duration<double, std::milli>(t_gpu_done - t_host_prep_done).count();
    const double cpu_cluster_ms =
      std::chrono::duration<double, std::milli>(t_cluster_done - t_gpu_done).count();
    const double pack_ms =
      std::chrono::duration<double, std::milli>(t_pack_done - t_cluster_done).count();
    const double publish_ms =
      std::chrono::duration<double, std::milli>(t_publish_done - t_pack_done).count();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "points in:%d roi:%zu ego_masked:%zu nonground:%d clusters:%d",
      static_cast<int>(msg->width * msg->height),
      roi_count_before_voxel, ego_masked, m, cluster_id);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "residual_ground_rejected:%zu voxel_kept:%d",
      residual_ground_rejected, m);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "timing ms total:%.2f roi:%.2f cpu_prep:%.2f gpu_wall:%.2f cpu_cluster:%.2f pack:%.2f publish:%.2f "
      "gpu[alloc:%.2f h2d:%.2f kernel:%.2f d2h:%.2f free:%.2f]",
      total_ms,
      roi_ms,
      cpu_prep_ms,
      gpu_wall_ms,
      cpu_cluster_ms,
      pack_ms,
      publish_ms,
      gpu_timing.alloc_ms, gpu_timing.h2d_ms, gpu_timing.kernel_ms,
      gpu_timing.d2h_ms, gpu_timing.free_ms);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  double eps_{0.35};
  int min_points_{12};
  int max_neighbors_{256};
  int max_points_{200000};

  double roi_min_x_{0.5};
  double roi_max_x_{20.0};
  double roi_min_y_{-5.0};
  double roi_max_y_{5.0};
  double roi_min_z_{-2.0};
  double roi_max_z_{2.0};

  bool enable_ego_box_filter_{false};
  bool ego_box_xy_only_{true};
  double ego_min_x_{-2.0};
  double ego_max_x_{2.0};
  double ego_min_y_{-1.0};
  double ego_max_y_{1.0};
  double ego_min_z_{-3.0};
  double ego_max_z_{1.0};
  double ego_box_offset_x_{0.0};
  double ego_box_offset_y_{0.0};
  double ego_box_offset_z_{0.0};

  bool cluster_xy_only_{true};
  double cluster_scale_x_{1.0};
  double cluster_scale_y_{1.0};
  double cluster_scale_z_{1.0};
  double xy_voxel_leaf_size_{0.0};
  bool enable_residual_ground_filter_{true};
  double residual_ground_filter_range_min_m_{3.0};
  double residual_ground_z_base_m_{-0.62};
  double residual_ground_z_slope_per_m_{0.010};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DbscanClusteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
