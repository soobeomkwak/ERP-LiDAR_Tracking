#include "obstacle_filtering/waypoint_graph.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace obstacle_filtering
{
namespace
{

std::string trim(const std::string & s)
{
  std::size_t b = 0U;
  while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])) != 0) {
    ++b;
  }
  std::size_t e = s.size();
  while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1U])) != 0) {
    --e;
  }
  return s.substr(b, e - b);
}

std::string to_upper(std::string s)
{
  for (char & c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
}

std::vector<std::string> split_csv_simple(const std::string & line)
{
  std::vector<std::string> out;
  std::string cur;
  cur.reserve(line.size());
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
      continue;
    }
    cur.push_back(c);
  }
  out.push_back(cur);
  return out;
}

bool parse_int(const std::string & s, int & out)
{
  char * end = nullptr;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (end == s.c_str() || *end != '\0') {
    return false;
  }
  out = static_cast<int>(v);
  return true;
}

bool parse_double(const std::string & s, double & out)
{
  char * end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str() || *end != '\0') {
    return false;
  }
  out = v;
  return std::isfinite(out);
}

bool find_column(
  const std::unordered_map<std::string, std::size_t> & cols,
  std::initializer_list<const char *> names,
  std::size_t & out)
{
  for (const auto & n : names) {
    const auto it = cols.find(n);
    if (it != cols.end()) {
      out = it->second;
      return true;
    }
  }
  return false;
}

std::vector<Point2D> densify_polyline(const std::vector<Point2D> & line, double step_m)
{
  if (line.size() < 2U || step_m <= 1e-3) {
    return line;
  }

  std::vector<Point2D> dense;
  dense.reserve(line.size() * 4U);
  dense.push_back(line.front());

  for (std::size_t i = 0; i + 1U < line.size(); ++i) {
    const double ax = line[i].x;
    const double ay = line[i].y;
    const double bx = line[i + 1U].x;
    const double by = line[i + 1U].y;
    const double dx = bx - ax;
    const double dy = by - ay;
    const double seg_len = std::hypot(dx, dy);
    if (seg_len <= 1e-9) {
      continue;
    }

    const int n_div = std::max(1, static_cast<int>(std::ceil(seg_len / step_m)));
    for (int k = 1; k <= n_div; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n_div);
      dense.push_back(Point2D{ax + t * dx, ay + t * dy});
    }
  }

  return dense;
}

double distance_sq_to_points(double px, double py, const std::vector<Point2D> & pts)
{
  if (pts.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : pts) {
    const double dx = px - p.x;
    const double dy = py - p.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best) {
      best = d2;
    }
  }
  return best;
}

}  // namespace

bool WaypointGraph::load_from_utm_csv(const std::string & csv_path, std::string & error_message)
{
  error_message.clear();

  std::ifstream ifs(csv_path);
  if (!ifs.is_open()) {
    error_message = "Failed to open CSV: " + csv_path;
    return false;
  }

  std::string header_line;
  if (!std::getline(ifs, header_line)) {
    error_message = "CSV is empty: " + csv_path;
    return false;
  }

  const auto headers_raw = split_csv_simple(header_line);
  std::unordered_map<std::string, std::size_t> col_idx;
  col_idx.reserve(headers_raw.size());
  for (std::size_t i = 0; i < headers_raw.size(); ++i) {
    col_idx.emplace(to_upper(trim(headers_raw[i])), i);
  }

  std::size_t idx_col = 0U;
  std::size_t l_utm_x_col = 0U;
  std::size_t l_utm_y_col = 0U;
  std::size_t l_alt_col = 0U;
  std::size_t r_utm_x_col = 0U;
  std::size_t r_utm_y_col = 0U;
  std::size_t r_alt_col = 0U;

  const bool has_index = find_column(col_idx, {"INDEX", "IDX", "ID"}, idx_col);
  const bool has_l_utm_x = find_column(col_idx, {"L1_UTM_X", "LEFT_UTM_X", "L_UTM_X"}, l_utm_x_col);
  const bool has_l_utm_y = find_column(col_idx, {"L1_UTM_Y", "LEFT_UTM_Y", "L_UTM_Y"}, l_utm_y_col);
  const bool has_l_alt = find_column(col_idx, {"L1_ALT", "LEFT_ALT", "L_ALT"}, l_alt_col);
  const bool has_r_utm_x = find_column(col_idx, {"R1_UTM_X", "RIGHT_UTM_X", "R_UTM_X"}, r_utm_x_col);
  const bool has_r_utm_y = find_column(col_idx, {"R1_UTM_Y", "RIGHT_UTM_Y", "R_UTM_Y"}, r_utm_y_col);
  const bool has_r_alt = find_column(col_idx, {"R1_ALT", "RIGHT_ALT", "R_ALT"}, r_alt_col);

  if (!(has_l_utm_x && has_l_utm_y && has_r_utm_x && has_r_utm_y)) {
    error_message = "CSV missing required UTM columns for left/right boundaries";
    return false;
  }

  std::vector<WaypointNode> parsed_nodes;
  parsed_nodes.reserve(8192);

  std::string line;
  int auto_idx = 0;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto cols = split_csv_simple(line);
    if (cols.size() < headers_raw.size()) {
      continue;
    }

    int index = auto_idx++;
    double le = 0.0;
    double ln = 0.0;
    double l_alt = 0.0;
    double re = 0.0;
    double rn = 0.0;
    double r_alt = 0.0;

    if (has_index) {
      (void)parse_int(trim(cols[idx_col]), index);
    }

    if (!parse_double(trim(cols[l_utm_x_col]), le) || !parse_double(trim(cols[l_utm_y_col]), ln) ||
      !parse_double(trim(cols[r_utm_x_col]), re) || !parse_double(trim(cols[r_utm_y_col]), rn))
    {
      continue;
    }

    if (has_l_alt) {
      (void)parse_double(trim(cols[l_alt_col]), l_alt);
    }
    if (has_r_alt) {
      (void)parse_double(trim(cols[r_alt_col]), r_alt);
    }

    WaypointNode n;
    n.index = index;
    n.left_alt = l_alt;
    n.right_alt = r_alt;

    n.left_utm = Point3D{le, ln, l_alt};
    n.right_utm = Point3D{re, rn, r_alt};
    n.center_utm = Point3D{0.5 * (le + re), 0.5 * (ln + rn), 0.5 * (l_alt + r_alt)};

    parsed_nodes.push_back(n);
  }

  if (parsed_nodes.empty()) {
    error_message = "No valid waypoint rows parsed from CSV: " + csv_path;
    return false;
  }

  std::sort(parsed_nodes.begin(), parsed_nodes.end(), [](const WaypointNode & a, const WaypointNode & b) {
    return a.index < b.index;
  });

  std::vector<WaypointNode> dedup;
  dedup.reserve(parsed_nodes.size());
  for (const auto & n : parsed_nodes) {
    if (!dedup.empty() && dedup.back().index == n.index) {
      dedup.back() = n;
    } else {
      dedup.push_back(n);
    }
  }

  nodes_ = std::move(dedup);
  has_origin_ = false;
  origin_utm_ = Point3D{};
  return true;
}

bool WaypointGraph::set_enu_origin_from_utm(double e0, double n0, double u0)
{
  if (!std::isfinite(e0) || !std::isfinite(n0) || !std::isfinite(u0) || nodes_.empty()) {
    return false;
  }

  origin_utm_ = Point3D{e0, n0, u0};
  for (auto & n : nodes_) {
    n.left_enu = Point3D{
      n.left_utm.x - origin_utm_.x,
      n.left_utm.y - origin_utm_.y,
      n.left_utm.z - origin_utm_.z};
    n.right_enu = Point3D{
      n.right_utm.x - origin_utm_.x,
      n.right_utm.y - origin_utm_.y,
      n.right_utm.z - origin_utm_.z};
    n.center_enu = Point3D{
      n.center_utm.x - origin_utm_.x,
      n.center_utm.y - origin_utm_.y,
      n.center_utm.z - origin_utm_.z};
  }

  has_origin_ = true;
  return true;
}

namespace
{

Point2D rotate_to_body_frame(double yaw, double dx, double dy)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Point2D{c * dx + s * dy, -s * dx + c * dy};
}

bool point_in_box(double x, double y, double min_x, double max_x, double min_y, double max_y)
{
  return x >= min_x && x <= max_x && y >= min_y && y <= max_y;
}

std::vector<std::vector<std::size_t>> build_circular_segments_from_mask(const std::vector<bool> & mask)
{
  std::vector<std::vector<std::size_t>> segments;
  const std::size_t n = mask.size();
  if (n == 0U) {
    return segments;
  }

  const bool all_selected = std::all_of(mask.begin(), mask.end(), [](bool v) { return v; });
  if (all_selected) {
    std::vector<std::size_t> segment;
    segment.reserve(n + (n > 1U ? 1U : 0U));
    for (std::size_t i = 0; i < n; ++i) {
      segment.push_back(i);
    }
    if (segment.size() > 1U) {
      segment.push_back(segment.front());
    }
    segments.push_back(std::move(segment));
    return segments;
  }

  std::size_t anchor_false = 0U;
  while (anchor_false < n && mask[anchor_false]) {
    ++anchor_false;
  }
  if (anchor_false >= n) {
    return segments;
  }

  std::vector<std::size_t> current;
  for (std::size_t step = 1; step <= n; ++step) {
    const std::size_t idx = (anchor_false + step) % n;
    if (mask[idx]) {
      current.push_back(idx);
    } else if (!current.empty()) {
      segments.push_back(std::move(current));
      current.clear();
    }
  }

  return segments;
}

bool find_nearest_window_offset(
  const std::vector<WaypointNode> & nodes,
  const WaypointWindow & window,
  double px,
  double py,
  std::size_t & best_offset,
  const std::vector<std::size_t> *& best_segment,
  std::size_t & best_local_idx)
{
  best_offset = 0U;
  best_segment = nullptr;
  best_local_idx = 0U;

  if (nodes.empty() || window.empty()) {
    return false;
  }

  double best_d2 = std::numeric_limits<double>::infinity();
  for (const auto & segment : window.segments) {
    for (std::size_t i = 0; i < segment.size(); ++i) {
      const std::size_t offset = segment[i];
      if (offset >= nodes.size()) {
        continue;
      }
      const double dx = nodes[offset].center_enu.x - px;
      const double dy = nodes[offset].center_enu.y - py;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_offset = offset;
        best_segment = &segment;
        best_local_idx = i;
      }
    }
  }

  return best_segment != nullptr;
}

std::vector<Point2D> dense_side_points(
  const std::vector<WaypointNode> & nodes,
  const WaypointWindow & window,
  double step_m,
  bool use_left)
{
  std::vector<Point2D> dense;
  if (nodes.empty() || window.empty()) {
    return dense;
  }

  for (const auto & segment : window.segments) {
    if (segment.empty()) {
      continue;
    }

    std::vector<Point2D> line;
    line.reserve(segment.size());
    for (const std::size_t offset : segment) {
      if (offset >= nodes.size()) {
        continue;
      }
      const auto & p = use_left ? nodes[offset].left_enu : nodes[offset].right_enu;
      line.push_back(Point2D{p.x, p.y});
    }

    const auto seg_dense = densify_polyline(line, step_m);
    dense.insert(dense.end(), seg_dense.begin(), seg_dense.end());
  }

  return dense;
}

}  // namespace

bool WaypointGraph::make_vehicle_window(
  double vehicle_x,
  double vehicle_y,
  int rear_count,
  int front_count,
  WaypointWindow & out) const
{
  if (!has_origin_ || nodes_.empty()) {
    return false;
  }

  const int rear = std::max(0, rear_count);
  const int front = std::max(0, front_count);

  std::size_t best_i = 0U;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const double dx = nodes_[i].center_enu.x - vehicle_x;
    const double dy = nodes_[i].center_enu.y - vehicle_y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }

  const std::size_t n = nodes_.size();
  const std::size_t count = std::min<std::size_t>(
    n, static_cast<std::size_t>(rear) + static_cast<std::size_t>(front) + 1U);

  std::vector<std::size_t> offsets;
  offsets.reserve(count + ((count == n && count > 1U) ? 1U : 0U));

  const auto wrap_index = [n](long long idx) {
    long long wrapped = idx % static_cast<long long>(n);
    if (wrapped < 0) {
      wrapped += static_cast<long long>(n);
    }
    return static_cast<std::size_t>(wrapped);
  };

  const long long start = static_cast<long long>(best_i) - static_cast<long long>(rear);
  for (std::size_t k = 0; k < count; ++k) {
    offsets.push_back(wrap_index(start + static_cast<long long>(k)));
  }
  if (count == n && offsets.size() > 1U) {
    offsets.push_back(offsets.front());
  }

  out = WaypointWindow{};
  out.nearest_index = nodes_[best_i].index;
  out.nearest_offset = best_i;
  out.segments.push_back(std::move(offsets));
  return true;
}

bool WaypointGraph::make_lidar_roi_window(
  double vehicle_x,
  double vehicle_y,
  double vehicle_yaw,
  double base_to_lidar_x,
  double base_to_lidar_y,
  double roi_min_x,
  double roi_max_x,
  double roi_min_y,
  double roi_max_y,
  WaypointWindow & out) const
{
  if (!has_origin_ || nodes_.empty()) {
    return false;
  }

  std::size_t best_i = 0U;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const double dx = nodes_[i].center_enu.x - vehicle_x;
    const double dy = nodes_[i].center_enu.y - vehicle_y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_i = i;
    }
  }

  std::vector<bool> in_roi(nodes_.size(), false);
  bool any_in_roi = false;
  for (std::size_t i = 0; i < nodes_.size(); ++i) {
    const double dx = nodes_[i].center_enu.x - vehicle_x;
    const double dy = nodes_[i].center_enu.y - vehicle_y;
    const auto p_base = rotate_to_body_frame(vehicle_yaw, dx, dy);
    const double lidar_x = p_base.x - base_to_lidar_x;
    const double lidar_y = p_base.y - base_to_lidar_y;
    const bool inside = point_in_box(lidar_x, lidar_y, roi_min_x, roi_max_x, roi_min_y, roi_max_y);
    in_roi[i] = inside;
    any_in_roi = any_in_roi || inside;
  }
  if (!any_in_roi) {
    return false;
  }

  out = WaypointWindow{};
  out.nearest_index = nodes_[best_i].index;
  out.nearest_offset = best_i;
  out.segments = build_circular_segments_from_mask(in_roi);
  return !out.empty();
}

bool WaypointGraph::point_inside_boundaries(double px, double py, const WaypointWindow & window) const
{
  if (!has_origin_ || nodes_.empty() || window.empty()) {
    return false;
  }

  std::size_t nearest_i = 0U;
  const std::vector<std::size_t> * nearest_segment = nullptr;
  std::size_t nearest_local_idx = 0U;
  if (!find_nearest_window_offset(nodes_, window, px, py, nearest_i, nearest_segment, nearest_local_idx)) {
    return false;
  }
  (void)nearest_segment;
  (void)nearest_local_idx;

  const auto & l = nodes_[nearest_i].left_enu;
  const auto & r = nodes_[nearest_i].right_enu;

  const double wx = r.x - l.x;
  const double wy = r.y - l.y;
  const double w2 = wx * wx + wy * wy;
  if (w2 <= 1e-9) {
    return false;
  }

  const double vx = px - l.x;
  const double vy = py - l.y;
  const double t = (vx * wx + vy * wy) / w2;
  return (t >= 0.0 && t <= 1.0);
}

bool WaypointGraph::point_inside_expanded_corridor(
  double px,
  double py,
  const WaypointWindow & window,
  double lateral_margin_m) const
{
  if (!has_origin_ || nodes_.empty() || window.empty()) {
    return false;
  }

  const double margin = std::max(0.0, lateral_margin_m);

  std::size_t nearest_i = 0U;
  const std::vector<std::size_t> * nearest_segment = nullptr;
  std::size_t nearest_local_idx = 0U;
  if (!find_nearest_window_offset(nodes_, window, px, py, nearest_i, nearest_segment, nearest_local_idx)) {
    return false;
  }
  if (nearest_segment == nullptr || nearest_segment->empty()) {
    return false;
  }

  const bool closed = nearest_segment->size() > 1U && nearest_segment->front() == nearest_segment->back();
  const std::size_t prev_i =
    (nearest_local_idx > 0U) ? (*nearest_segment)[nearest_local_idx - 1U] :
    (closed && nearest_segment->size() > 2U ? (*nearest_segment)[nearest_segment->size() - 2U] : nearest_i);
  const std::size_t next_i =
    (nearest_local_idx + 1U < nearest_segment->size()) ? (*nearest_segment)[nearest_local_idx + 1U] :
    (closed && nearest_segment->size() > 2U ? (*nearest_segment)[1U] : nearest_i);

  const auto & c_prev = nodes_[prev_i].center_enu;
  const auto & c_next = nodes_[next_i].center_enu;

  double tx = c_next.x - c_prev.x;
  double ty = c_next.y - c_prev.y;
  double t_norm = std::hypot(tx, ty);
  if (t_norm <= 1e-9) {
    const auto & l = nodes_[nearest_i].left_enu;
    const auto & r = nodes_[nearest_i].right_enu;
    tx = r.y - l.y;
    ty = -(r.x - l.x);
    t_norm = std::hypot(tx, ty);
    if (t_norm <= 1e-9) {
      return false;
    }
  }
  tx /= t_norm;
  ty /= t_norm;

  const double nx = -ty;
  const double ny = tx;

  const auto & c = nodes_[nearest_i].center_enu;
  const auto & l = nodes_[nearest_i].left_enu;
  const auto & r = nodes_[nearest_i].right_enu;

  const double lr_width = std::hypot(r.x - l.x, r.y - l.y);
  const double half_width = 0.5 * lr_width + margin;

  const double vx = px - c.x;
  const double vy = py - c.y;
  const double lateral_dist = std::abs(vx * nx + vy * ny);
  return lateral_dist <= half_width;
}

double WaypointGraph::min_distance_sq_to_left(double px, double py, const WaypointWindow & window) const
{
  return distance_sq_to_points(px, py, left_points_dense(window, 0.3));
}

double WaypointGraph::min_distance_sq_to_right(double px, double py, const WaypointWindow & window) const
{
  return distance_sq_to_points(px, py, right_points_dense(window, 0.3));
}

std::vector<Point2D> WaypointGraph::left_points_dense(const WaypointWindow & window, double step_m) const
{
  return dense_side_points(nodes_, window, step_m, true);
}

std::vector<Point2D> WaypointGraph::right_points_dense(const WaypointWindow & window, double step_m) const
{
  return dense_side_points(nodes_, window, step_m, false);
}

}  // namespace obstacle_filtering
