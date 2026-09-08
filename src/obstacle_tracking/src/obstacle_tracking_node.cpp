// Map-frame CV-KF tracker.
// Obstacles are tracked in the map frame so ego rotation never corrupts
// the process model — boxes stay stable through turns and lane changes.
//
// State: [x_map, y_map, vx_map, vy_map]
// Measurement: [x_map, y_map] (detection transformed to map via TF)
// Heading: tracked separately with EMA (base_link frame, 180° disambiguated)
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "lshape_fitting/msg/detection2_d_array.hpp"
#include "obstacle_tracking/msg/track_debug_array.hpp"
#include "obstacle_tracking/msg/track2_d.hpp"
#include "obstacle_tracking/msg/track2_d_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

// ── Math helpers ────────────────────────────────────────────────────────────
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr int kS = 4;   // state dim: x y vx vy (map frame)
constexpr int kM = 2;   // meas  dim: x y (map frame)

using SV = std::array<double, kS>;
using SM = std::array<double, kS * kS>;
using MM = std::array<double, kM * kM>;

double wrap(double y)
{
  while (y > kPi) y -= 2.0 * kPi;
  while (y < -kPi) y += 2.0 * kPi;
  return y;
}
double ambig180(double y, double ref)
{
  const double a = wrap(y), b = wrap(y + kPi);
  return std::abs(wrap(a - ref)) <= std::abs(wrap(b - ref)) ? a : b;
}
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

geometry_msgs::msg::Quaternion yaw_to_quat(double y)
{
  tf2::Quaternion q; q.setRPY(0.0, 0.0, y); q.normalize(); return tf2::toMsg(q);
}
double quat_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion qq; tf2::fromMsg(q, qq);
  double r = 0.0, p = 0.0, y = 0.0;
  tf2::Matrix3x3(qq).getRPY(r, p, y); return wrap(y);
}

// Index helpers for flat arrays
double & s(SM & m, int r, int c) { return m[r * kS + c]; }
double   s(const SM & m, int r, int c) { return m[r * kS + c]; }
double & m2(MM & m, int r, int c) { return m[r * kM + c]; }
double   m2(const MM & m, int r, int c) { return m[r * kM + c]; }

bool inv2x2(const MM & in, MM & out)
{
  const double det = m2(in,0,0)*m2(in,1,1) - m2(in,0,1)*m2(in,1,0);
  if (!std::isfinite(det) || std::abs(det) < 1e-12) return false;
  const double id = 1.0 / det;
  m2(out,0,0) =  m2(in,1,1)*id;  m2(out,0,1) = -m2(in,0,1)*id;
  m2(out,1,0) = -m2(in,1,0)*id;  m2(out,1,1) =  m2(in,0,0)*id;
  return true;
}

// Jonker-Volgenant / Hungarian (squared-cost O(n³))
std::vector<int> hungarian(const std::vector<std::vector<double>> & cost)
{
  const std::size_t rows = cost.size();
  const std::size_t cols = rows > 0 ? cost.front().size() : 0;
  const std::size_t sz   = std::max(rows, cols);
  if (sz == 0) return {};
  const double kBig = 1e9;
  std::vector<std::vector<double>> sq(sz, std::vector<double>(sz, kBig));
  for (std::size_t r = 0; r < rows; ++r)
    for (std::size_t c = 0; c < cols; ++c)
      sq[r][c] = cost[r][c];
  std::vector<double> u(sz+1,0), v(sz+1,0);
  std::vector<int> p(sz+1,0), way(sz+1,0);
  for (std::size_t i = 1; i <= sz; ++i) {
    p[0] = static_cast<int>(i); int j0 = 0;
    std::vector<double> minv(sz+1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(sz+1, false);
    do {
      used[static_cast<std::size_t>(j0)] = true;
      const int i0 = p[static_cast<std::size_t>(j0)];
      double delta = std::numeric_limits<double>::infinity(); int j1 = 0;
      for (std::size_t j = 1; j <= sz; ++j) {
        if (used[j]) continue;
        const double cur = sq[static_cast<std::size_t>(i0-1)][j-1]
          - u[static_cast<std::size_t>(i0)] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = static_cast<int>(j); }
      }
      for (std::size_t j = 0; j <= sz; ++j)
        if (used[j]) { u[static_cast<std::size_t>(p[j])] += delta; v[j] -= delta; }
        else minv[j] -= delta;
      j0 = j1;
    } while (p[static_cast<std::size_t>(j0)] != 0);
    do {
      const int j1 = way[static_cast<std::size_t>(j0)];
      p[static_cast<std::size_t>(j0)] = p[static_cast<std::size_t>(j1)];
      j0 = j1;
    } while (j0 != 0);
  }
  std::vector<int> asgn(rows, -1);
  for (std::size_t j = 1; j <= sz; ++j) {
    const int ri = p[j] - 1;
    if (ri >= 0 && static_cast<std::size_t>(ri) < rows && j-1 < cols)
      asgn[static_cast<std::size_t>(ri)] = static_cast<int>(j-1);
  }
  return asgn;
}

std::string fmt(double v, int p = 2)
{
  std::ostringstream o; o.precision(p); o << std::fixed << v; return o.str();
}

// Minimal flat-JSON field extractor (no external library needed)
double json_double(const std::string & j, const std::string & key, double fallback = 0.0)
{
  const std::string tok = "\"" + key + "\": ";
  auto pos = j.find(tok);
  if (pos == std::string::npos) return fallback;
  pos += tok.size();
  try { return std::stod(j.substr(pos)); } catch (...) { return fallback; }
}
std::string json_string(const std::string & j, const std::string & key)
{
  const std::string tok = "\"" + key + "\": \"";
  auto pos = j.find(tok);
  if (pos == std::string::npos) return "?";
  pos += tok.size();
  auto end = j.find('"', pos);
  return (end == std::string::npos) ? "?" : j.substr(pos, end - pos);
}
}  // namespace

// ── Node ─────────────────────────────────────────────────────────────────────
class ObstacleTrackingNode : public rclcpp::Node
{
public:
  explicit ObstacleTrackingNode(const rclcpp::NodeOptions & opt = rclcpp::NodeOptions())
  : Node("obstacle_tracking_node", opt)
  {
    // ── declare parameters ──────────────────────────────────────────────────
    declare_parameter<std::string>("input_topic", "/target/detections");
    declare_parameter<std::string>("output_topic", "");
    declare_parameter<std::string>("marker_topic", "/tracked/markers");
    declare_parameter<std::string>("debug_topic", "/tracked/debug");
    declare_parameter<std::string>("perception_obstacles_topic", "/perception/obstacles");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<int>("queue_size", 10);

    // Kalman noise
    declare_parameter<double>("process_noise_pos", 0.20);       // 뼈대 차량 centroid 불안정 → 큰 값으로 유효 gate 확장
    declare_parameter<double>("process_noise_vel_low", 0.5);   // static/slow → 속도 안정
    declare_parameter<double>("process_noise_vel_high", 2.0);  // moving → 기동 추적
    declare_parameter<double>("measurement_noise_x", 0.50);    // 뼈대 차량 위치 노이즈 큼
    declare_parameter<double>("measurement_noise_y", 0.50);

    // Association
    declare_parameter<double>("gate_mahal_sq", 9.0);

    // Size gate
    declare_parameter<double>("track_max_length_m", 3.2);
    declare_parameter<double>("track_max_width_m", 2.2);
    declare_parameter<double>("track_max_area_m2", 7.0);

    // Lifecycle
    declare_parameter<int>("confirm_hits", 2);
    declare_parameter<int>("max_missed_frames", 10);
    declare_parameter<bool>("publish_unconfirmed", false);
    declare_parameter<double>("min_confidence_new", 0.35);
    declare_parameter<double>("min_confidence_confirm", 0.30);
    declare_parameter<double>("confidence_decay", 0.97);
    declare_parameter<double>("confidence_alpha", 0.30);

    // Velocity
    declare_parameter<double>("init_speed_mps", 0.0);   // new track initial speed (ego direction)
    declare_parameter<double>("init_speed_min_ego_speed_mps", 1.0);
    declare_parameter<double>("velocity_decay_on_miss", 0.80);
    declare_parameter<double>("velocity_decay_on_miss_moving", 0.99);
    declare_parameter<double>("max_speed_mps", 10.0);
    declare_parameter<double>("max_measurement_jump_m", 2.0);   // 뼈대 차량 centroid 점프 허용
    declare_parameter<double>("static_clamp_mps", 0.30);      // ZUPT speed threshold
    declare_parameter<double>("static_pos_clamp_m", 0.12);   // ZUPT innovation threshold
    declare_parameter<int>("static_lock_hits", 6);            // frames before position lock
    declare_parameter<double>("static_unlock_m", 0.40);       // innovation to break lock

    // Heading
    declare_parameter<double>("heading_alpha", 0.30);
    declare_parameter<double>("heading_freeze_speed_mps", 0.5);

    // Size smoothing
    declare_parameter<double>("size_alpha", 0.20);
    declare_parameter<double>("size_shrink_max", 0.85);
    declare_parameter<double>("size_grow_max", 1.20);

    // Reacquire
    declare_parameter<double>("reacquire_sec", 2.0);
    declare_parameter<double>("reacquire_dist_m", 2.5);
    // Fragment suppression: distance-dependent no-spawn zone around confirmed tracks
    // Near ego (<near_range_m): min_new_track_dist_m, Far (>far_range_m): min_new_track_dist_far_m
    // Linear blend in between.
    declare_parameter<double>("min_new_track_dist_m",     1.2);  // close range suppression radius
    declare_parameter<double>("min_new_track_dist_far_m", 2.5);  // far range suppression radius
    declare_parameter<double>("frag_near_range_m",  6.0);        // distance where near→far blend starts
    declare_parameter<double>("frag_far_range_m",  12.0);        // distance where far suppression is fully applied

    declare_parameter<double>("dt_min_sec", 0.02);
    declare_parameter<double>("dt_max_sec", 0.30);
    declare_parameter<double>("debug_period_sec", 1.0);

    // ── read parameters ─────────────────────────────────────────────────────
    map_frame_     = get_parameter("map_frame").as_string();
    base_frame_    = get_parameter("base_frame").as_string();
    q_pos_         = get_parameter("process_noise_pos").as_double();
    q_vel_low_     = get_parameter("process_noise_vel_low").as_double();
    q_vel_high_    = get_parameter("process_noise_vel_high").as_double();
    r_x_           = get_parameter("measurement_noise_x").as_double();
    r_y_           = get_parameter("measurement_noise_y").as_double();
    gate_sq_       = get_parameter("gate_mahal_sq").as_double();
    max_len_       = get_parameter("track_max_length_m").as_double();
    max_wid_       = get_parameter("track_max_width_m").as_double();
    max_area_      = get_parameter("track_max_area_m2").as_double();
    confirm_hits_  = get_parameter("confirm_hits").as_int();
    max_missed_    = get_parameter("max_missed_frames").as_int();
    pub_unconf_    = get_parameter("publish_unconfirmed").as_bool();
    min_conf_new_  = get_parameter("min_confidence_new").as_double();
    min_conf_ok_   = get_parameter("min_confidence_confirm").as_double();
    conf_decay_    = get_parameter("confidence_decay").as_double();
    conf_alpha_    = get_parameter("confidence_alpha").as_double();
    vel_decay_         = get_parameter("velocity_decay_on_miss").as_double();
    vel_decay_moving_  = get_parameter("velocity_decay_on_miss_moving").as_double();
    max_speed_     = get_parameter("max_speed_mps").as_double();
    max_jump_      = get_parameter("max_measurement_jump_m").as_double();
    static_clamp_      = get_parameter("static_clamp_mps").as_double();
    static_pos_clamp_  = get_parameter("static_pos_clamp_m").as_double();
    static_lock_hits_  = get_parameter("static_lock_hits").as_int();
    static_unlock_m_   = get_parameter("static_unlock_m").as_double();
    hdg_alpha_     = get_parameter("heading_alpha").as_double();
    hdg_freeze_    = get_parameter("heading_freeze_speed_mps").as_double();
    sz_alpha_      = get_parameter("size_alpha").as_double();
    sz_shrink_     = get_parameter("size_shrink_max").as_double();
    sz_grow_       = get_parameter("size_grow_max").as_double();
    reacq_sec_        = get_parameter("reacquire_sec").as_double();
    reacq_dist_       = get_parameter("reacquire_dist_m").as_double();
    min_new_dist_     = get_parameter("min_new_track_dist_m").as_double();
    min_new_dist_far_ = get_parameter("min_new_track_dist_far_m").as_double();
    frag_near_range_  = get_parameter("frag_near_range_m").as_double();
    frag_far_range_   = get_parameter("frag_far_range_m").as_double();
    init_speed_    = get_parameter("init_speed_mps").as_double();
    init_speed_min_ego_speed_ = get_parameter("init_speed_min_ego_speed_mps").as_double();
    dt_min_        = get_parameter("dt_min_sec").as_double();
    dt_max_        = get_parameter("dt_max_sec").as_double();
    dbg_period_    = get_parameter("debug_period_sec").as_double();

    // ── TF ──────────────────────────────────────────────────────────────────
    tf_buf_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_buf_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface()));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // ── subscriptions / publishers ───────────────────────────────────────────
    const int qs = get_parameter("queue_size").as_int();
    sub_ = create_subscription<lshape_fitting::msg::Detection2DArray>(
      get_parameter("input_topic").as_string(), rclcpp::QoS(qs),
      std::bind(&ObstacleTrackingNode::on_detection, this, _1));

    const auto out_t  = get_parameter("output_topic").as_string();
    const auto perc_t = get_parameter("perception_obstacles_topic").as_string();
    const auto mkr_t  = get_parameter("marker_topic").as_string();
    if (!out_t.empty())
      track_pub_  = create_publisher<obstacle_tracking::msg::Track2DArray>(out_t, 10);
    if (!perc_t.empty() && perc_t != out_t)
      perc_pub_   = create_publisher<obstacle_tracking::msg::Track2DArray>(perc_t, 10);
    if (!mkr_t.empty())
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(mkr_t, 10);

    vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        ego_vx_        = msg->twist.twist.linear.x;
        ego_map_speed_ = std::hypot(
          msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      });

    plan_debug_sub_ = create_subscription<std_msgs::msg::String>(
      "/planning/behavior_debug", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        plan_json_ = msg->data;
      });

    last_dbg_ = now();
    RCLCPP_INFO(get_logger(),
      "obstacle_tracking [MAP-FRAME CV-KF]  gate=%.1f  q_pos=%.3f  r=%.2f  static_clamp=%.2f",
      gate_sq_, q_pos_, r_x_, static_clamp_);
  }

private:
  // ── Data types ─────────────────────────────────────────────────────────────
  struct Det {
    double x{0}, y{0};        // map frame
    double yaw_base{0};       // heading in base_link (from L-shape)
    double length{0}, width{0}, confidence{0};
  };

  struct Track {
    int32_t id{0};
    SV st{};     // [x_map, y_map, vx_map, vy_map]
    SM P{};
    double hdg{0};    // EMA heading in base_link frame
    bool has_hdg{false};
    double length{0}, width{0}, confidence{0};
    int hits{0}, missed{0}, age{0};
    bool confirmed{false};
    // Position lock for confirmed static obstacles
    int static_frames{0};        // consecutive frames with ZUPT (v==0)
    bool pos_locked{false};
    double locked_x{0}, locked_y{0};
  };

  struct Dead { Track t; rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME}; };

  // ── KF: predict ───────────────────────────────────────────────────────────
  // F = [[I dt*I],[0 I]] (constant-velocity model)
  void kf_predict(Track & tk, double dt) const
  {
    // Locked tracks: keep position fixed; use measurement-noise-sized P so
    // the Mahalanobis gate stays wide enough (~3m) despite L-shape variation.
    if (tk.pos_locked) {
      tk.st[0] = tk.locked_x; tk.st[1] = tk.locked_y;
      tk.st[2] = 0.0;         tk.st[3] = 0.0;
      for (auto & v : tk.P) v = 0.0;
      s(tk.P,0,0) = r_x_; s(tk.P,1,1) = r_y_;   // gate ≈ sqrt(gate_sq*(r+r)) wide
      s(tk.P,2,2) = 0.01; s(tk.P,3,3) = 0.01;
      return;
    }
    // State propagation
    tk.st[0] += tk.st[2] * dt;
    tk.st[1] += tk.st[3] * dt;
    clamp_speed(tk);

    // P = F*P*F' + Q
    // FP[r][c] = P[r][c] + dt*P[r+2][c]  (for r<2)
    SM FP{};
    for (int r = 0; r < kS; ++r)
      for (int c = 0; c < kS; ++c)
        s(FP,r,c) = s(tk.P,r,c) + (r < 2 ? dt * s(tk.P,r+2,c) : 0.0);
    // Pnew[r][c] = FP[r][c] + dt*FP[r][c+2]  (for c<2)
    SM Pn{};
    for (int r = 0; r < kS; ++r)
      for (int c = 0; c < kS; ++c)
        s(Pn,r,c) = s(FP,r,c) + (c < 2 ? dt * s(FP,r,c+2) : 0.0);
    const double dt2 = dt * dt;
    const double spd = std::hypot(tk.st[2], tk.st[3]);
    const double q_vel = (spd > static_clamp_) ? q_vel_high_ : q_vel_low_;
    s(Pn,0,0) += q_pos_*dt2;  s(Pn,1,1) += q_pos_*dt2;
    s(Pn,2,2) += q_vel*dt2;   s(Pn,3,3) += q_vel*dt2;
    tk.P = Pn;
  }

  // ── KF: Mahalanobis² (position only) ──────────────────────────────────────
  double mahal_sq(const Track & tk, const Det & d) const
  {
    const double dx = d.x - tk.st[0], dy = d.y - tk.st[1];
    const double s00 = s(tk.P,0,0)+r_x_, s01 = s(tk.P,0,1);
    const double s10 = s(tk.P,1,0),      s11 = s(tk.P,1,1)+r_y_;
    const double det = s00*s11 - s01*s10;
    if (!std::isfinite(det) || det <= 1e-12) return 1e9;
    const double id = 1.0 / det;
    return dx*(s11*id*dx - s10*id*dy) + dy*(-s01*id*dx + s00*id*dy);
  }

  // ── KF: update (Joseph form) ───────────────────────────────────────────────
  bool kf_update(Track & tk, const Det & d)
  {
    // Locked tracks: check if the object has started moving before skipping KF.
    if (tk.pos_locked) {
      const double raw_inno = std::hypot(d.x - tk.locked_x, d.y - tk.locked_y);
      if (raw_inno > static_unlock_m_) {
        // Object moved — release lock and fall through to full KF update
        tk.pos_locked    = false;
        tk.static_frames = 0;
        // Reset P so KF can re-converge quickly
        for (auto & v : tk.P) v = 0.0;
        s(tk.P,0,0)=0.5; s(tk.P,1,1)=0.5; s(tk.P,2,2)=4.0; s(tk.P,3,3)=4.0;
      } else {
        // Still static: skip KF, update secondary attributes only
        const double aligned = ambig180(d.yaw_base, tk.hdg);
        tk.hdg = wrap(tk.hdg + hdg_alpha_ * wrap(aligned - tk.hdg));
        const double el = clamp(d.length, tk.length*sz_shrink_, tk.length*sz_grow_);
        const double ew = clamp(d.width,  tk.width *sz_shrink_, tk.width *sz_grow_);
        tk.length = (1.0-sz_alpha_)*tk.length + sz_alpha_*el;
        tk.width  = (1.0-sz_alpha_)*tk.width  + sz_alpha_*ew;
        tk.confidence = (1.0-conf_alpha_)*tk.confidence + conf_alpha_*d.confidence;
        tk.hits += 1; tk.missed = 0; tk.static_frames += 1;
        tk.st[0] = tk.locked_x; tk.st[1] = tk.locked_y;
        tk.st[2] = 0.0;         tk.st[3] = 0.0;
        return true;
      }
    }

    // Clamp jump
    double mx = d.x, my = d.y;
    const double jd = std::hypot(mx - tk.st[0], my - tk.st[1]);
    if (max_jump_ > 0.0 && jd > max_jump_) {
      const double sc = max_jump_ / jd;
      mx = tk.st[0] + (mx - tk.st[0]) * sc;
      my = tk.st[1] + (my - tk.st[1]) * sc;
    }
    const double ix = mx - tk.st[0], iy = my - tk.st[1];

    // S = P[0:2,0:2] + R
    MM Sm{}, Si{};
    m2(Sm,0,0) = s(tk.P,0,0)+r_x_;  m2(Sm,0,1) = s(tk.P,0,1);
    m2(Sm,1,0) = s(tk.P,1,0);        m2(Sm,1,1) = s(tk.P,1,1)+r_y_;
    if (!inv2x2(Sm, Si)) return false;

    // K = P * H' * S^{-1}   (H selects rows 0,1)
    double K[kS][kM]{};
    for (int r = 0; r < kS; ++r) {
      K[r][0] = s(tk.P,r,0)*m2(Si,0,0) + s(tk.P,r,1)*m2(Si,1,0);
      K[r][1] = s(tk.P,r,0)*m2(Si,0,1) + s(tk.P,r,1)*m2(Si,1,1);
    }

    // State update
    for (int r = 0; r < kS; ++r)
      tk.st[static_cast<std::size_t>(r)] += K[r][0]*ix + K[r][1]*iy;
    clamp_speed(tk);

    // ZUPT: only clamp velocity when BOTH speed is low AND innovation is small.
    // Checking innovation prevents locking onto a walking person whose KF
    // velocity estimate hasn't converged yet (speed looks low but position moves).
    const double inno_dist = std::hypot(ix, iy);
    const bool zupt_applied = (static_clamp_ > 0.0 &&
        std::hypot(tk.st[2], tk.st[3]) < static_clamp_ &&
        inno_dist < static_pos_clamp_);
    if (zupt_applied) {
      tk.st[2] = 0.0; tk.st[3] = 0.0;
    }

    // Covariance: Joseph form  P = (I-KH)*P*(I-KH)' + K*R*K'
    SM A{};
    for (int r = 0; r < kS; ++r)
      for (int c = 0; c < kS; ++c) {
        double v = (r == c) ? 1.0 : 0.0;
        if (c == 0) v -= K[r][0];
        if (c == 1) v -= K[r][1];
        s(A,r,c) = v;
      }
    SM AP{};
    for (int r = 0; r < kS; ++r)
      for (int c = 0; c < kS; ++c) {
        double v = 0.0;
        for (int k = 0; k < kS; ++k) v += s(A,r,k) * s(tk.P,k,c);
        s(AP,r,c) = v;
      }
    SM J{};
    for (int r = 0; r < kS; ++r)
      for (int c = 0; c < kS; ++c) {
        double v = 0.0;
        for (int k = 0; k < kS; ++k) v += s(AP,r,k) * s(A,c,k);
        s(J,r,c) = v + K[r][0]*r_x_*K[c][0] + K[r][1]*r_y_*K[c][1];
      }
    tk.P = J;

    // Heading EMA (180° disambiguated, base_link frame)
    const double ref = tk.has_hdg ? tk.hdg : d.yaw_base;
    const double aligned = ambig180(d.yaw_base, ref);
    if (!tk.has_hdg) { tk.hdg = aligned; tk.has_hdg = true; }
    else { tk.hdg = wrap(tk.hdg + hdg_alpha_ * wrap(aligned - tk.hdg)); }

    // Size (clamped EMA)
    const double el = clamp(d.length, tk.length*sz_shrink_, tk.length*sz_grow_);
    const double ew = clamp(d.width,  tk.width *sz_shrink_, tk.width *sz_grow_);
    tk.length = (1.0-sz_alpha_)*tk.length + sz_alpha_*el;
    tk.width  = (1.0-sz_alpha_)*tk.width  + sz_alpha_*ew;

    tk.confidence =
      (1.0 - conf_alpha_) * tk.confidence + conf_alpha_ * d.confidence;
    tk.hits   += 1;
    tk.missed  = 0;
    tk.confirmed = tk.confirmed ||
      (tk.hits >= confirm_hits_ && tk.confidence >= min_conf_ok_);

    // Position lock: confirmed + static for enough frames → freeze position.
    // Release immediately if innovation exceeds unlock threshold (object moving).
    if (tk.pos_locked && inno_dist > static_unlock_m_) {
      tk.pos_locked   = false;
      tk.static_frames = 0;
    }
    if (zupt_applied) {
      tk.static_frames += 1;
      if (!tk.pos_locked && tk.confirmed && tk.static_frames >= static_lock_hits_) {
        tk.locked_x   = tk.st[0];
        tk.locked_y   = tk.st[1];
        tk.pos_locked = true;
      }
    } else {
      tk.static_frames = 0;
    }
    if (tk.pos_locked) {
      tk.st[0] = tk.locked_x; tk.st[1] = tk.locked_y;
    }
    return true;
  }

  // ── Helpers ─────────────────────────────────────────────────────────────────
  void clamp_speed(Track & tk) const
  {
    const double spd = std::hypot(tk.st[2], tk.st[3]);
    if (spd > max_speed_) { tk.st[2] *= max_speed_/spd; tk.st[3] *= max_speed_/spd; }
  }

  bool size_ok(const Det & d) const
  {
    const double L = std::max(d.length, d.width);
    const double W = std::min(d.length, d.width);
    return !(max_len_  > 0.0 && L   > max_len_) &&
           !(max_wid_  > 0.0 && W   > max_wid_) &&
           !(max_area_ > 0.0 && L*W > max_area_);
  }

  Track make_track(const Det & d, double cb, double sb) const
  {
    Track tk;
    // Seed velocity in ego heading direction when ego is moving.
    // P_vel=100 (σ_v=10 m/s): high initial uncertainty → fast convergence in both
    // directions (static objects reach ZUPT in ~2 frames, fast objects converge in ~3).
    const double spd = (ego_map_speed_ > init_speed_min_ego_speed_) ? init_speed_ : 0.0;
    tk.st = {d.x, d.y, spd * cb, spd * sb};
    tk.P = {}; s(tk.P,0,0)=0.5; s(tk.P,1,1)=0.5; s(tk.P,2,2)=100.0; s(tk.P,3,3)=100.0;
    tk.hdg = d.yaw_base; tk.has_hdg = true;
    tk.length = d.length; tk.width = d.width; tk.confidence = d.confidence;
    tk.hits = 1; tk.age = 1;
    return tk;
  }

  std::optional<std::size_t> find_reacquire(
    const Det & d, const rclcpp::Time & stamp) const
  {
    std::optional<std::size_t> best;
    double bc = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < dead_.size(); ++i) {
      if ((stamp - dead_[i].stamp).seconds() > reacq_sec_) continue;
      const double dist = std::hypot(d.x - dead_[i].t.st[0], d.y - dead_[i].t.st[1]);
      if (dist > reacq_dist_ || dist >= bc) continue;
      bc = dist; best = i;
    }
    return best;
  }

  bool get_tf(
    const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & tf) const
  {
    try {
      tf = tf_buf_->lookupTransform(map_frame_, base_frame_,
        rclcpp::Time(stamp), tf2::durationFromSec(0.1));
      return true;
    } catch (...) {}
    try {
      tf = tf_buf_->lookupTransform(map_frame_, base_frame_,
        tf2::TimePointZero, tf2::durationFromSec(0.1));
      return true;
    } catch (...) {}
    return false;
  }

  // ── Main callback ────────────────────────────────────────────────────────────
  void on_detection(const lshape_fitting::msg::Detection2DArray::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf{};
    if (!get_tf(msg->header.stamp, tf)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "No TF %s→%s", base_frame_.c_str(), map_frame_.c_str());
      return;
    }
    const double base_yaw = quat_to_yaw(tf.transform.rotation);
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const double cb = std::cos(base_yaw), sb = std::sin(base_yaw);

    // ── transform detections to map frame ─────────────────────────────────
    std::vector<Det> dets;
    dets.reserve(msg->detections.size());
    for (const auto & det : msg->detections) {
      const double bx = det.pose.position.x, by = det.pose.position.y;
      Det d;
      d.x = tx + cb*bx - sb*by;
      d.y = ty + sb*bx + cb*by;
      d.yaw_base = quat_to_yaw(det.pose.orientation);
      d.length   = det.length;
      d.width    = det.width;
      d.confidence = det.confidence;
      if (size_ok(d)) dets.push_back(d);
    }

    // ── dt ────────────────────────────────────────────────────────────────
    const rclcpp::Time stamp(msg->header.stamp);
    double dt = 0.1;
    if (has_prev_stamp_)
      dt = clamp((stamp - prev_stamp_).seconds(), dt_min_, dt_max_);
    prev_stamp_ = stamp; has_prev_stamp_ = true;

    // ── predict ───────────────────────────────────────────────────────────
    for (auto & tk : tracks_) {
      kf_predict(tk, dt);
      tk.age += 1;
      tk.confidence *= conf_decay_;
    }

    // ── cost matrix + Hungarian ───────────────────────────────────────────
    const double kBig = 1e9;
    std::vector<std::vector<double>> cost(
      tracks_.size(), std::vector<double>(dets.size(), kBig));
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti)
      for (std::size_t di = 0; di < dets.size(); ++di) {
        const double mh = mahal_sq(tracks_[ti], dets[di]);
        if (mh <= gate_sq_) cost[ti][di] = mh;
      }

    std::vector<bool> t_hit(tracks_.size(), false), d_hit(dets.size(), false);
    if (!tracks_.empty() && !dets.empty()) {
      const auto asgn = hungarian(cost);
      for (std::size_t ti = 0; ti < asgn.size(); ++ti) {
        const int di_i = asgn[ti];
        if (di_i < 0) continue;
        const auto di = static_cast<std::size_t>(di_i);
        if (di >= dets.size() || cost[ti][di] >= kBig) continue;
        if (kf_update(tracks_[ti], dets[di])) {
          t_hit[ti] = true; d_hit[di] = true;
        }
      }
    }

    // ── missed tracks ──────────────────────────────────────────────────────
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      if (t_hit[ti]) continue;
      tracks_[ti].missed += 1;
      if (tracks_[ti].pos_locked) {
        // Keep locked position frozen through dropout frames
        tracks_[ti].st[0] = tracks_[ti].locked_x;
        tracks_[ti].st[1] = tracks_[ti].locked_y;
        tracks_[ti].st[2] = 0.0; tracks_[ti].st[3] = 0.0;
      } else {
        const double spd_miss = std::hypot(tracks_[ti].st[2], tracks_[ti].st[3]);
        const double decay = (spd_miss > static_clamp_) ? vel_decay_moving_ : vel_decay_;
        tracks_[ti].st[2] *= decay;
        tracks_[ti].st[3] *= decay;
        if (static_clamp_ > 0.0 &&
            std::hypot(tracks_[ti].st[2], tracks_[ti].st[3]) < static_clamp_)
        {
          tracks_[ti].st[2] = 0.0; tracks_[ti].st[3] = 0.0;
        }
      }
      tracks_[ti].confirmed =
        tracks_[ti].confirmed && tracks_[ti].confidence >= min_conf_ok_;
    }

    // ── new tracks for unmatched detections ────────────────────────────────
    for (std::size_t di = 0; di < dets.size(); ++di) {
      if (d_hit[di] || dets[di].confidence < min_conf_new_) continue;
      // Suppress fragment detections near a confirmed track.
      // Suppression radius grows with ego distance: far vehicles are sparser → larger fragments.
      if (min_new_dist_ > 0.0) {
        const double det_ego_dist = std::hypot(dets[di].x - tx, dets[di].y - ty);
        const double t = std::max(0.0, std::min(1.0,
          (det_ego_dist - frag_near_range_) / std::max(frag_far_range_ - frag_near_range_, 1e-3)));
        const double suppress_r = min_new_dist_ + t * (min_new_dist_far_ - min_new_dist_);
        bool near = false;
        for (const auto & tk : tracks_) {
          if (!tk.confirmed) continue;
          if (std::hypot(dets[di].x - tk.st[0], dets[di].y - tk.st[1]) < suppress_r) {
            near = true; break;
          }
        }
        if (near) continue;
      }
      Track nt;
      if (auto idx = find_reacquire(dets[di], stamp)) {
        nt = dead_[*idx].t;
        nt.st[0] = dets[di].x; nt.st[1] = dets[di].y;
        nt.st[2] = 0.0; nt.st[3] = 0.0;   // reset velocity on reacquire
        nt.length = dets[di].length; nt.width = dets[di].width;
        nt.confidence = dets[di].confidence; nt.missed = 0;
        dead_.erase(dead_.begin() + static_cast<std::ptrdiff_t>(*idx));
      } else {
        nt = make_track(dets[di], cb, sb);
        nt.id = next_id_++;
      }
      tracks_.push_back(nt);
    }

    // ── prune dead tracks ──────────────────────────────────────────────────
    {
      std::vector<Track> kept;
      for (const auto & tk : tracks_) {
        if (tk.missed > max_missed_) dead_.push_back({tk, stamp});
        else kept.push_back(tk);
      }
      tracks_ = std::move(kept);
      dead_.erase(
        std::remove_if(dead_.begin(), dead_.end(), [&](const Dead & d) {
          return (stamp - d.stamp).seconds() > reacq_sec_;
        }), dead_.end());
    }

    publish(tf, base_yaw, cb, sb);
    if ((now() - last_dbg_).seconds() >= dbg_period_) emit_debug();
  }

  // ── Publish ────────────────────────────────────────────────────────────────
  void publish(
    const geometry_msgs::msg::TransformStamped & tf,
    double base_yaw, double cb, double sb)
  {
    const double tx = tf.transform.translation.x, ty = tf.transform.translation.y;

    std_msgs::msg::Header hdr;
    hdr.stamp     = tf.header.stamp;
    hdr.frame_id  = map_frame_;

    obstacle_tracking::msg::Track2DArray out;
    out.header = hdr;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker del;
    del.header = hdr;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(del);

    for (const auto & tk : tracks_) {
      if (!pub_unconf_ && !tk.confirmed) continue;

      const double map_spd = std::hypot(tk.st[2], tk.st[3]);

      // map pose
      const double map_hdg = wrap(base_yaw + tk.hdg);
      geometry_msgs::msg::Pose map_pose;
      map_pose.position.x = tk.st[0];
      map_pose.position.y = tk.st[1];
      map_pose.orientation = yaw_to_quat(map_hdg);

      // base_link pose (inverse TF)
      const double dx = tk.st[0] - tx, dy = tk.st[1] - ty;
      geometry_msgs::msg::Pose base_pose;
      base_pose.position.x =  cb*dx + sb*dy;
      base_pose.position.y = -sb*dx + cb*dy;
      const double out_hdg =
        (tk.has_hdg && map_spd < hdg_freeze_) ? tk.hdg : (tk.has_hdg ? tk.hdg : 0.0);
      base_pose.orientation = yaw_to_quat(out_hdg);

      // velocities
      geometry_msgs::msg::Twist map_twist;
      map_twist.linear.x = tk.st[2]; map_twist.linear.y = tk.st[3];
      geometry_msgs::msg::Twist twist;
      twist.linear.x =  cb*tk.st[2] + sb*tk.st[3];
      twist.linear.y = -sb*tk.st[2] + cb*tk.st[3];

      obstacle_tracking::msg::Track2D tm;
      tm.header    = hdr;
      tm.track_id  = tk.id;
      tm.pose      = base_pose;
      tm.map_pose  = map_pose;
      tm.twist     = twist;
      tm.map_twist = map_twist;
      tm.length    = static_cast<float>(tk.length);
      tm.width     = static_cast<float>(tk.width);
      tm.confidence = static_cast<float>(
        clamp(0.2 + 0.5*tk.confidence + 0.05*tk.hits - 0.05*tk.missed, 0.0, 1.0));
      tm.age    = tk.age;
      tm.hits   = tk.hits;
      tm.missed = tk.missed;
      out.tracks.push_back(tm);

      // ── Color logic ────────────────────────────────────────────────────
      // unstable: missed > 0 (running on prediction only) → orange
      // locked:   pos_locked                              → red text
      // small:    length < 1.2 m (person/cone)            → yellow
      // vehicle:  length >= 1.2 m                         → green
      const bool unstable = (tk.missed >= 3);  // 3프레임(~0.4초) 이상 연속 miss 시 불안정
      const bool small_obj = (tk.length * tk.width < 1.0);  // < 1m²: person/cone

      float br, bg, bb;   // box colour
      if (unstable) {
        br = 1.0f; bg = 0.5f; bb = 0.0f;   // orange
      } else if (small_obj) {
        br = 1.0f; bg = 1.0f; bb = 0.0f;   // yellow
      } else {
        br = 0.1f; bg = 0.9f; bb = 0.1f;   // green
      }

      float tr, tg, tb;   // text colour
      if (tk.pos_locked) {
        tr = 1.0f; tg = 0.0f; tb = 0.0f;   // red
      } else if (unstable) {
        tr = 1.0f; tg = 0.5f; tb = 0.0f;   // orange
      } else {
        tr = 1.0f; tg = 1.0f; tb = 1.0f;   // white
      }

      // ── Marker: cube in map frame ──────────────────────────────────────
      visualization_msgs::msg::Marker cube;
      cube.header = hdr;
      cube.ns     = "tracked_cube";
      cube.id     = tk.id;
      cube.type   = visualization_msgs::msg::Marker::CUBE;
      cube.action = visualization_msgs::msg::Marker::ADD;
      cube.pose   = map_pose;
      cube.scale.x = std::max(0.1, tk.length);
      cube.scale.y = std::max(0.1, tk.width);
      cube.scale.z = 1.0;
      cube.color.r = br; cube.color.g = bg; cube.color.b = bb;
      cube.color.a = 0.5f;
      markers.markers.push_back(cube);

      visualization_msgs::msg::Marker txt;
      txt.header = hdr;
      txt.ns     = "tracked_text";
      txt.id     = tk.id;
      txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      txt.action = visualization_msgs::msg::Marker::ADD;
      txt.pose.position.x = tk.st[0];
      txt.pose.position.y = tk.st[1];
      txt.pose.position.z = 1.4;
      txt.pose.orientation.w = 1.0;
      txt.scale.z = 0.28;
      txt.color.r = tr; txt.color.g = tg; txt.color.b = tb; txt.color.a = 1.0f;
      txt.text =
        "id=" + std::to_string(tk.id) +
        (tk.pos_locked ? " [LOCKED]" : (unstable ? " [MISS]" : "")) +
        "\nmap_pose (" + fmt(tk.st[0],1) + ", " + fmt(tk.st[1],1) + ")" +
        "\nmap_twist (" + fmt(tk.st[2],2) + ", " + fmt(tk.st[3],2) + ")" +
        "\nhypot (" + fmt(map_spd,2) + " m/s)" +
        "\nxTTC " + [&]() -> std::string {
          const double dist_x = base_pose.position.x;
          const double rel_vx = ego_vx_ - twist.linear.x;   // 접근 속도 (양수 = 가까워짐)
          if (dist_x > 0.0 && rel_vx > 0.1)
            return fmt(dist_x / rel_vx, 1) + " s";
          return "---";
        }();
      markers.markers.push_back(txt);
    }

    // ── Ego speed marker ──────────────────────────────────────────────────
    {
      visualization_msgs::msg::Marker ego_txt;
      ego_txt.header = hdr;
      ego_txt.ns     = "ego_speed";
      ego_txt.id     = 0;
      ego_txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      ego_txt.action = visualization_msgs::msg::Marker::ADD;
      ego_txt.pose.position.x = tx;
      ego_txt.pose.position.y = ty;
      ego_txt.pose.position.z = 1.8;
      ego_txt.pose.orientation.w = 1.0;
      ego_txt.scale.z = 0.28;
      ego_txt.color.r = 0.0f; ego_txt.color.g = 1.0f;
      ego_txt.color.b = 1.0f; ego_txt.color.a = 1.0f;
      // planning TTC 파싱
      const std::string mode    = plan_json_.empty() ? "---" : json_string(plan_json_, "mode");
      const double plan_ttc_eff = plan_json_.empty() ? 9999.0 : json_double(plan_json_, "ttc_effective", 9999.0);
      const double plan_gap     = plan_json_.empty() ? 9999.0 : json_double(plan_json_, "effective_gap", 9999.0);
      const auto ttc_str = [](double v) -> std::string {
        return (v >= 999.0) ? "inf" : fmt(v, 1) + "s";
      };

      ego_txt.text =
        "EGO  " + fmt(ego_map_speed_, 2) + " m/s\n"
        "[" + mode + "]\n"
        "TTC " + ttc_str(plan_ttc_eff) + "\n"
        "gap " + fmt(plan_gap, 1) + "m";
      markers.markers.push_back(ego_txt);
    }

    if (track_pub_)  track_pub_->publish(out);
    if (perc_pub_)   perc_pub_->publish(out);
    if (marker_pub_) marker_pub_->publish(markers);
  }

  void emit_debug()
  {
    const int nc = static_cast<int>(
      std::count_if(tracks_.begin(), tracks_.end(),
        [](const Track & tk){ return tk.confirmed; }));
    RCLCPP_INFO(get_logger(),
      "Tracker: active=%zu confirmed=%d  dead=%zu",
      tracks_.size(), nc, dead_.size());
    // Per-track info
    for (const auto & tk : tracks_) {
      if (!tk.confirmed) continue;
      RCLCPP_INFO(get_logger(),
        "  id=%-3d map=(%.2f,%.2f) v=(%.3f,%.3f) spd=%.3f  h=%d m=%d",
        tk.id, tk.st[0], tk.st[1], tk.st[2], tk.st[3],
        std::hypot(tk.st[2], tk.st[3]), tk.hits, tk.missed);
    }
    last_dbg_ = now();
  }

  // ── Members ──────────────────────────────────────────────────────────────
  rclcpp::Subscription<lshape_fitting::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_debug_sub_;
  rclcpp::Publisher<obstacle_tracking::msg::Track2DArray>::SharedPtr track_pub_, perc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  double ego_map_speed_{0.0};
  double ego_vx_{0.0};
  std::string plan_json_;
  std::unique_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_, base_frame_;
  double q_pos_, q_vel_low_, q_vel_high_, r_x_, r_y_, gate_sq_;
  double max_len_, max_wid_, max_area_;
  int confirm_hits_, max_missed_;
  bool pub_unconf_;
  double min_conf_new_, min_conf_ok_, conf_decay_, conf_alpha_;
  double vel_decay_, vel_decay_moving_, max_speed_, max_jump_, static_clamp_, static_pos_clamp_, static_unlock_m_;
  int static_lock_hits_;
  double hdg_alpha_, hdg_freeze_;
  double sz_alpha_, sz_shrink_, sz_grow_;
  double reacq_sec_, reacq_dist_;
  double min_new_dist_, min_new_dist_far_, frag_near_range_, frag_far_range_;
  double init_speed_, init_speed_min_ego_speed_;
  double dt_min_, dt_max_, dbg_period_;

  int32_t next_id_{0};
  bool has_prev_stamp_{false};
  rclcpp::Time prev_stamp_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_dbg_{0, 0, RCL_SYSTEM_TIME};
  std::vector<Track> tracks_;
  std::vector<Dead>  dead_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleTrackingNode>());
  rclcpp::shutdown();
  return 0;
}
