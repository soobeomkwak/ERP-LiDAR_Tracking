#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace obstacle_filtering
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct WaypointNode
{
  int index{0};

  double left_lat{0.0};
  double left_lon{0.0};
  double left_alt{0.0};

  double right_lat{0.0};
  double right_lon{0.0};
  double right_alt{0.0};

  Point3D left_utm;
  Point3D right_utm;
  Point3D center_utm;

  Point3D left_enu;
  Point3D right_enu;
  Point3D center_enu;
};

struct WaypointWindow
{
  int nearest_index{-1};
  std::size_t nearest_offset{0U};
  std::vector<std::vector<std::size_t>> segments;

  bool empty() const { return segments.empty(); }
};

class WaypointGraph
{
public:
  bool load_from_utm_csv(const std::string & csv_path, std::string & error_message);

  bool set_enu_origin_from_utm(double e0, double n0, double u0);
  bool has_origin() const { return has_origin_; }

  bool make_vehicle_window(
    double vehicle_x,
    double vehicle_y,
    int rear_count,
    int front_count,
    WaypointWindow & out) const;

  bool make_lidar_roi_window(
    double vehicle_x,
    double vehicle_y,
    double vehicle_yaw,
    double base_to_lidar_x,
    double base_to_lidar_y,
    double roi_min_x,
    double roi_max_x,
    double roi_min_y,
    double roi_max_y,
    WaypointWindow & out) const;

  bool point_inside_boundaries(double px, double py, const WaypointWindow & window) const;
  bool point_inside_expanded_corridor(
    double px,
    double py,
    const WaypointWindow & window,
    double lateral_margin_m) const;

  double min_distance_sq_to_left(double px, double py, const WaypointWindow & window) const;
  double min_distance_sq_to_right(double px, double py, const WaypointWindow & window) const;

  std::vector<Point2D> left_points_dense(const WaypointWindow & window, double step_m) const;
  std::vector<Point2D> right_points_dense(const WaypointWindow & window, double step_m) const;

  bool empty() const { return nodes_.empty(); }
  std::size_t size() const { return nodes_.size(); }
  const std::vector<WaypointNode> & nodes() const { return nodes_; }

private:
  bool has_origin_{false};
  Point3D origin_utm_;
  std::vector<WaypointNode> nodes_;
};

}  // namespace obstacle_filtering
