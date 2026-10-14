#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "cohan_msgs/msg/agent_type.hpp"
#include "cohan_msgs/msg/tracked_segment_type.hpp"
#include "cohan_msgs/msg/tracked_agents.hpp"
#include "crowd_aware_interfaces/msg/dynamic_dwal_cluster.hpp"
#include "crowd_aware_interfaces/msg/dynamic_dwal_path.hpp"
#include "crowd_aware_interfaces/msg/dynamic_dwal_state.hpp"
#include "crowd_aware_simulation/dynamic_dwal_core.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cas = crowd_aware_simulation;
using std::placeholders::_1;

namespace
{

enum class FootprintState {FREE, COLLISION, UNKNOWN};

struct FootprintCheck
{
  FootprintState state{FootprintState::FREE};
  std::string reason{"free"};
  double point_x{std::numeric_limits<double>::quiet_NaN()};
  double point_y{std::numeric_limits<double>::quiet_NaN()};
  int costmap_cell_x{-1};
  int costmap_cell_y{-1};
  int costmap_cell_value{-1};
  int laser_ray_index{-1};
  double laser_ray_angle{std::numeric_limits<double>::quiet_NaN()};
  double laser_ray_range{std::numeric_limits<double>::quiet_NaN()};
  double laser_required_range{std::numeric_limits<double>::quiet_NaN()};
  std::string detail;
};

double quaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<std::pair<double, double>> transformPolygon(
  const std::vector<std::pair<double, double>> & footprint,
  double x, double y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<std::pair<double, double>> result;
  result.reserve(footprint.size());
  for (const auto & point : footprint) {
    result.emplace_back(
      x + c * point.first - s * point.second,
      y + s * point.first + c * point.second);
  }
  return result;
}

bool intervalsOverlap(double a_min, double a_max, double b_min, double b_max)
{
  return a_max >= b_min && b_max >= a_min;
}

bool polygonIntersectsRectangle(
  const std::vector<std::pair<double, double>> & polygon,
  double xmin, double ymin, double xmax, double ymax)
{
  auto separated = [&](double ax, double ay) {
      double pmin = std::numeric_limits<double>::infinity();
      double pmax = -std::numeric_limits<double>::infinity();
      for (const auto & p : polygon) {
        const double projection = ax * p.first + ay * p.second;
        pmin = std::min(pmin, projection);
        pmax = std::max(pmax, projection);
      }
      const double r0 = ax * xmin + ay * ymin;
      const double r1 = ax * xmin + ay * ymax;
      const double r2 = ax * xmax + ay * ymin;
      const double r3 = ax * xmax + ay * ymax;
      const double rmin = std::min({r0, r1, r2, r3});
      const double rmax = std::max({r0, r1, r2, r3});
      return !intervalsOverlap(pmin, pmax, rmin, rmax);
    };

  if (separated(1.0, 0.0) || separated(0.0, 1.0)) {
    return false;
  }
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double edge_x = b.first - a.first;
    const double edge_y = b.second - a.second;
    if (separated(-edge_y, edge_x)) {
      return false;
    }
  }
  return true;
}

bool pointInConvexPolygon(
  const std::vector<std::pair<double, double>> & polygon,
  double x, double y)
{
  double reference_cross = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double cross =
      (b.first - a.first) * (y - a.second) -
      (b.second - a.second) * (x - a.first);
    if (std::abs(cross) <= 1.0e-10) {
      continue;
    }
    if (reference_cross == 0.0) {
      reference_cross = cross;
    } else if ((cross > 0.0) != (reference_cross > 0.0)) {
      return false;
    }
  }
  return true;
}

bool segmentIntersectsRectangle(
  double ax, double ay, double bx, double by,
  double xmin, double ymin, double xmax, double ymax)
{
  // Liang-Barsky clipping. End-point contact at the laser or query is not a
  // self shadow; the open segment must actually pass through the body box.
  const double dx = bx - ax;
  const double dy = by - ay;
  double enter = 0.0;
  double leave = 1.0;
  const double p[4] = {-dx, dx, -dy, dy};
  const double q[4] = {ax - xmin, xmax - ax, ay - ymin, ymax - ay};
  for (int i = 0; i < 4; ++i) {
    if (std::abs(p[i]) <= 1.0e-12) {
      if (q[i] < 0.0) {
        return false;
      }
      continue;
    }
    const double ratio = q[i] / p[i];
    if (p[i] < 0.0) {
      enter = std::max(enter, ratio);
    } else {
      leave = std::min(leave, ratio);
    }
    if (enter > leave) {
      return false;
    }
  }
  return leave > 1.0e-6 && enter < 1.0 - 1.0e-6;
}


std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA value;
  value.r = r;
  value.g = g;
  value.b = b;
  value.a = a;
  return value;
}

std_msgs::msg::ColorRGBA clusterColor(std::int32_t id)
{
  switch ((id % 6 + 6) % 6) {
    case 0: return color(0.10F, 0.85F, 0.20F);
    case 1: return color(0.10F, 0.55F, 1.00F);
    case 2: return color(0.95F, 0.75F, 0.05F);
    case 3: return color(0.75F, 0.20F, 0.95F);
    case 4: return color(0.00F, 0.85F, 0.85F);
    default: return color(1.00F, 0.35F, 0.65F);
  }
}

}  // namespace

class DynamicDwalNode : public rclcpp::Node
{
public:
  DynamicDwalNode()
  : Node("dynamic_dwal")
  {
    mode_ = declare_parameter<std::string>("mode", "off");
    if (mode_ != "off" && mode_ != "on") {
      throw std::runtime_error("dynamic_dwal.mode must be 'off' or 'on'");
    }

    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "sim_base");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    costmap_topic_ = declare_parameter<std::string>(
      "costmap_topic", "/local_costmap/costmap");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser_frame");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/reference_cmd");
    tracks_topic_ = declare_parameter<std::string>(
      "tracks_topic", "/tracked_agents");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/dynamic_dwal/diagnostics");
    markers_topic_ = declare_parameter<std::string>(
      "markers_topic", "/dynamic_dwal/markers");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/cmd_vel_selected");

    max_linear_ = declare_parameter<double>("max_linear", 0.30);
    max_angular_ = declare_parameter<double>("max_angular", 0.80);
    acceleration_ = declare_parameter<double>("acceleration", 0.50);
    deceleration_ = declare_parameter<double>("deceleration", 0.50);
    angular_acceleration_ = declare_parameter<double>(
      "angular_acceleration", 1.0);
    angular_deceleration_ = declare_parameter<double>(
      "angular_deceleration", 1.0);
    dynamic_window_dt_ = declare_parameter<double>("dynamic_window_dt", 0.10);
    max_curvature_ = declare_parameter<double>("max_curvature", 4.0);
    curvature_step_ = declare_parameter<double>("curvature_step", 0.10);
    zero_linear_epsilon_ = declare_parameter<double>(
      "zero_linear_epsilon", 0.02);

    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.61);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.095);
    max_wheel_angular_speed_ = declare_parameter<double>(
      "max_wheel_angular_speed", 20.0);

    max_search_length_ = declare_parameter<double>("max_search_length", 4.0);
    path_output_resolution_ = declare_parameter<double>(
      "path_output_resolution", 0.05);
    sweep_resolution_ = declare_parameter<double>("sweep_resolution", 0.02);
    ineligible_sweep_resolution_ = declare_parameter<double>(
      "ineligible_sweep_resolution", 0.05);
    footprint_padding_ = declare_parameter<double>("footprint_padding", 0.02);
    lethal_cost_ = declare_parameter<int>("lethal_cost", 100);
    reaction_time_ = declare_parameter<double>("reaction_time", 0.40);
    longitudinal_margin_ = declare_parameter<double>(
      "longitudinal_margin", 0.05);
    known_space_resolution_ = declare_parameter<double>(
      "known_space_resolution", 0.025);
    laser_x_ = declare_parameter<double>("laser_x", 0.85);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    candidate_marker_width_ = declare_parameter<double>(
      "candidate_marker_width", 0.005);
    selected_marker_width_ = declare_parameter<double>(
      "selected_marker_width", 0.010);
    transition_marker_scale_ = declare_parameter<double>(
      "transition_marker_scale", 0.035);
    marker_labels_ = declare_parameter<bool>("marker_labels", false);
    marker_label_scale_ = declare_parameter<double>(
      "marker_label_scale", 0.055);
    stale_marker_lifetime_ = declare_parameter<double>(
      "stale_marker_lifetime", 0.75);

    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);
    costmap_timeout_ = declare_parameter<double>("costmap_timeout", 0.50);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.30);
    reference_timeout_ = declare_parameter<double>("reference_timeout", 0.35);
    tracks_timeout_ = declare_parameter<double>("tracks_timeout", 0.50);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);

    prediction_horizon_ = declare_parameter<double>(
      "prediction_horizon", 2.0);
    prediction_time_step_ = declare_parameter<double>(
      "prediction_time_step", 0.05);
    final_speed_resolution_ = declare_parameter<double>(
      "final_speed_resolution", 0.025);
    human_radius_ = declare_parameter<double>("human_radius", 0.40);
    human_base_uncertainty_ = declare_parameter<double>(
      "human_base_uncertainty", 0.10);
    human_uncertainty_growth_ = declare_parameter<double>(
      "human_uncertainty_growth", 0.10);

    const auto footprint_values = declare_parameter<std::vector<double>>(
      "footprint", std::vector<double>{
        0.865, 0.405, 0.865, -0.405, -0.103, -0.405, -0.103, 0.405});
    if (footprint_values.size() < 6 || footprint_values.size() % 2 != 0) {
      throw std::runtime_error("dynamic_dwal.footprint must contain x,y pairs");
    }
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < footprint_values.size(); i += 2) {
      xmin = std::min(xmin, footprint_values[i]);
      xmax = std::max(xmax, footprint_values[i]);
      ymin = std::min(ymin, footprint_values[i + 1]);
      ymax = std::max(ymax, footprint_values[i + 1]);
    }
    shadow_xmin_ = xmin;
    shadow_xmax_ = xmax;
    shadow_ymin_ = ymin;
    shadow_ymax_ = ymax;
    // Padding is applied exactly once here. Inflation-layer graded costs are
    // deliberately not treated as hard collision below.
    xmin -= footprint_padding_;
    xmax += footprint_padding_;
    ymin -= footprint_padding_;
    ymax += footprint_padding_;
    footprint_ = {
      {xmax, ymax}, {xmax, ymin}, {xmin, ymin}, {xmin, ymax}};
    footprint_max_radius_ = 0.0;
    for (const auto & point : footprint_) {
      footprint_max_radius_ = std::max(
        footprint_max_radius_, std::hypot(point.first, point.second));
    }

    if (max_search_length_ <= 0.0 || path_output_resolution_ <= 0.0 ||
      sweep_resolution_ <= 0.0 || publish_rate_ <= 0.0 ||
      ineligible_sweep_resolution_ <= 0.0 ||
      dynamic_window_dt_ <= 0.0 || prediction_time_step_ <= 0.0 ||
      final_speed_resolution_ <= 0.0 || candidate_marker_width_ <= 0.0 ||
      selected_marker_width_ <= 0.0 || transition_marker_scale_ <= 0.0 ||
      marker_label_scale_ <= 0.0 || stale_marker_lifetime_ <= 0.0)
    {
      throw std::runtime_error("dynamic_dwal resolution/rate parameters must be positive");
    }

    state_publisher_ =
      create_publisher<crowd_aware_interfaces::msg::DynamicDwalState>(
      diagnostics_topic_, 10);
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);
    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);

    subscription_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscription_group_;
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_odom_ = std::move(msg);
        latest_odom_received_ = now();
      }, subscription_options);
    reference_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      reference_topic_, rclcpp::QoS(20),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_reference_ = std::move(msg);
        latest_reference_received_ = now();
      }, subscription_options);
    costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_costmap_ = std::move(msg);
        latest_costmap_received_ = now();
      }, subscription_options);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_scan_ = std::move(msg);
        latest_scan_received_ = now();
      }, subscription_options);

    // OFF intentionally has no structured-track subscription.
    if (mode_ == "on") {
      tracks_subscription_ = create_subscription<cohan_msgs::msg::TrackedAgents>(
        tracks_topic_, rclcpp::QoS(20),
        [this](cohan_msgs::msg::TrackedAgents::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(input_mutex_);
          latest_tracks_ = std::move(msg);
          latest_tracks_received_ = now();
        }, subscription_options);
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&DynamicDwalNode::tick, this));
    RCLCPP_INFO(
      get_logger(),
      "dynamic_dwal %s: per-path generator and shared teleop controller",
      mode_.c_str());
  }

private:
  double receiveAge(const rclcpp::Time & received, bool present) const
  {
    if (!present) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (now() - received).seconds());
  }

  double stampedAge(
    const builtin_interfaces::msg::Time & stamp,
    const rclcpp::Time & received, bool present) const
  {
    const double rx_age = receiveAge(received, present);
    if (!present || (stamp.sec == 0 && stamp.nanosec == 0)) {
      return rx_age;
    }
    const double source_age = (now() - rclcpp::Time(stamp)).seconds();
    return std::max(rx_age, std::max(0.0, source_age));
  }

  cas::Pose2 globalPose(const cas::Pose2 & local) const
  {
    const auto & position = odom_->pose.pose.position;
    const double initial_yaw = quaternionYaw(odom_->pose.pose.orientation);
    const double c = std::cos(initial_yaw);
    const double s = std::sin(initial_yaw);
    return {
      position.x + c * local.x - s * local.y,
      position.y + s * local.x + c * local.y,
      initial_yaw + local.theta};
  }

  std::vector<std::pair<double, double>> footprintAt(
    const cas::Pose2 & local) const
  {
    const cas::Pose2 global = globalPose(local);
    return transformPolygon(
      footprint_, global.x, global.y, global.theta);
  }

  FootprintCheck footprintState(
    const cas::Pose2 & local, double sweep_epsilon) const
  {
    FootprintCheck result;
    const auto global_polygon = footprintAt(local);
    const auto & origin = costmap_->info.origin;
    const double origin_yaw = quaternionYaw(origin.orientation);
    const double c = std::cos(origin_yaw);
    const double s = std::sin(origin_yaw);
    auto to_map_coordinates =
      [&](const std::vector<std::pair<double, double>> & source) {
        std::vector<std::pair<double, double>> result;
        result.reserve(source.size());
        for (const auto & point : source) {
          const double dx = point.first - origin.position.x;
          const double dy = point.second - origin.position.y;
          result.emplace_back(c * dx + s * dy, -s * dx + c * dy);
        }
        return result;
      };
    const auto polygon = to_map_coordinates(global_polygon);
    // Certify only the footprint occupied by the robot at the current pose.
    // Newly swept unknown cells still terminate the candidate path.
    const auto current_polygon =
      to_map_coordinates(footprintAt(cas::Pose2{}));

    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto & point : polygon) {
      xmin = std::min(xmin, point.first);
      xmax = std::max(xmax, point.first);
      ymin = std::min(ymin, point.second);
      ymax = std::max(ymax, point.second);
    }
    xmin -= sweep_epsilon;
    xmax += sweep_epsilon;
    ymin -= sweep_epsilon;
    ymax += sweep_epsilon;

    const double resolution = costmap_->info.resolution;
    const int ix_min = static_cast<int>(std::floor(xmin / resolution));
    const int ix_max = static_cast<int>(std::floor(xmax / resolution));
    const int iy_min = static_cast<int>(std::floor(ymin / resolution));
    const int iy_max = static_cast<int>(std::floor(ymax / resolution));
    auto record_cell = [&](FootprintCheck & check, int ix, int iy, int value) {
        const double map_x = (static_cast<double>(ix) + 0.5) * resolution;
        const double map_y = (static_cast<double>(iy) + 0.5) * resolution;
        check.point_x =
          origin.position.x + c * map_x - s * map_y;
        check.point_y =
          origin.position.y + s * map_x + c * map_y;
        check.costmap_cell_x = ix;
        check.costmap_cell_y = iy;
        check.costmap_cell_value = value;
      };
    for (int iy = iy_min; iy <= iy_max; ++iy) {
      for (int ix = ix_min; ix <= ix_max; ++ix) {
        const double cell_xmin = ix * resolution - sweep_epsilon;
        const double cell_ymin = iy * resolution - sweep_epsilon;
        const double cell_xmax = (ix + 1) * resolution + sweep_epsilon;
        const double cell_ymax = (iy + 1) * resolution + sweep_epsilon;
        if (!polygonIntersectsRectangle(
            polygon, cell_xmin, cell_ymin, cell_xmax, cell_ymax))
        {
          continue;
        }
        if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(costmap_->info.width) ||
          iy >= static_cast<int>(costmap_->info.height))
        {
          if (result.state == FootprintState::FREE) {
            result.state = FootprintState::UNKNOWN;
            result.reason = "costmap_boundary";
            result.detail = "footprint intersects a cell outside the costmap bounds";
            record_cell(result, ix, iy, -1);
          }
          continue;
        }
        const std::size_t index =
          static_cast<std::size_t>(iy) * costmap_->info.width +
          static_cast<std::size_t>(ix);
        const int value = static_cast<int>(costmap_->data[index]);
        if (value < 0) {
          if (!polygonIntersectsRectangle(
              current_polygon, ix * resolution, iy * resolution,
              (ix + 1) * resolution, (iy + 1) * resolution))
          {
            if (result.state == FootprintState::FREE) {
              result.state = FootprintState::UNKNOWN;
              result.reason = "unknown_costmap_cell";
              result.detail = "new swept footprint intersects an unknown costmap cell";
              record_cell(result, ix, iy, value);
            }
          }
        } else if (value >= lethal_cost_) {
          result.state = FootprintState::COLLISION;
          result.reason = "lethal_footprint_collision";
          result.detail = "padded footprint intersects a lethal costmap cell";
          record_cell(result, ix, iy, value);
          return result;
        }
      }
    }
    return result;
  }
  FootprintCheck footprintKnownByScan(
    const cas::Pose2 & local, double sweep_epsilon,
    const std::vector<std::vector<std::pair<double, double>>> &
    certified_footprints) const
  {
    FootprintCheck result;
    if (!scan_ || scan_->ranges.empty() ||
      scan_->angle_increment <= 0.0F)
    {
      result.state = FootprintState::UNKNOWN;
      result.reason = "invalid_laser_ray";
      result.detail = "laser scan is empty or has a non-positive angle increment";
      return result;
    }
    const auto polygon =
      transformPolygon(footprint_, local.x, local.y, local.theta);
    auto set_failure_point = [&](FootprintCheck & check, double x, double y) {
        const cas::Pose2 global = globalPose({x, y, 0.0});
        check.point_x = global.x;
        check.point_y = global.y;
      };
    auto point_is_known = [&](double x, double y) {
        FootprintCheck check;
        // Previously accepted footprint area remains certified even when the
        // front-mounted laser is self-occluded. New swept area still needs rays.
        // Consecutive sweep poses overlap by construction. Reusing the most
        // recent certified footprint is sufficient and conservative: older
        // coverage may cause an extra laser check, never an unsafe acceptance.
        // This avoids quadratic work as a path grows.
        if (!certified_footprints.empty() &&
          pointInConvexPolygon(certified_footprints.back(), x, y))
        {
          return check;
        }

        const double base_dx = x - laser_x_;
        const double base_dy = y - laser_y_;
        const double laser_c = std::cos(laser_yaw_);
        const double laser_s = std::sin(laser_yaw_);
        const double dx = laser_c * base_dx + laser_s * base_dy;
        const double dy = -laser_s * base_dx + laser_c * base_dy;
        const double distance = std::hypot(dx, dy);
        double angle = std::atan2(dy, dx);
        while (angle < scan_->angle_min) {
          angle += 2.0 * cas::kPi;
        }
        while (angle > scan_->angle_max) {
          angle -= 2.0 * cas::kPi;
        }
        if (angle < scan_->angle_min || angle > scan_->angle_max) {
          check.state = FootprintState::UNKNOWN;
          check.reason = "laser_outside_fov";
          check.laser_ray_angle = angle;
          check.laser_required_range = distance + sweep_epsilon;
          check.detail = "footprint boundary point is outside the laser field of view";
          set_failure_point(check, x, y);
          return check;
        }
        const int center = static_cast<int>(std::llround(
          (angle - scan_->angle_min) / scan_->angle_increment));
        double clear_range = std::numeric_limits<double>::infinity();
        bool valid_ray = false;
        bool delegated_self_shadow = false;
        const bool full_circle_scan =
          static_cast<double>(scan_->ranges.size()) * scan_->angle_increment >=
          2.0 * cas::kPi - 1.5 * scan_->angle_increment;
        for (int offset = -1; offset <= 1; ++offset) {
          int index = center + offset;
          if (full_circle_scan) {
            const int count = static_cast<int>(scan_->ranges.size());
            index = (index % count + count) % count;
          } else if (index < 0 ||
            index >= static_cast<int>(scan_->ranges.size()))
          {
            check.state = FootprintState::UNKNOWN;
            check.reason = "laser_outside_fov";
            check.laser_ray_index = index;
            check.laser_ray_angle = angle;
            check.laser_required_range = distance + sweep_epsilon;
            check.detail = "laser neighborhood extends outside the scan array";
            set_failure_point(check, x, y);
            return check;
          }
          const double measured = scan_->ranges[index];
          double ray_range = 0.0;
          if (std::isinf(measured) && measured > 0.0) {
            ray_range = scan_->range_max;
          } else if (std::isfinite(measured) &&
            measured >= scan_->range_min &&
            measured <= scan_->range_max)
          {
            ray_range = measured;
          } else {
            // Check the geometry of this exact invalid ray. The former code
            // reused the query point's angle for all three neighboring rays;
            // a tangent query was therefore rejected when only its neighbor
            // was a legitimate self-filtered body return.
            const double ray_angle =
              scan_->angle_min + static_cast<double>(index) *
              scan_->angle_increment;
            const double base_ray_angle = laser_yaw_ + ray_angle;
            // The box filter invalidates the complete ray when its eventual
            // return is on the robot. The queried free point can be before
            // that return, so prove intersection over the sensor range, not
            // merely over the shorter laser-to-query segment.
            const double ray_length = scan_->range_max;
            const double ray_x =
              laser_x_ + ray_length * std::cos(base_ray_angle);
            const double ray_y =
              laser_y_ + ray_length * std::sin(base_ray_angle);
            const bool ray_self_occluded = segmentIntersectsRectangle(
              laser_x_, laser_y_, ray_x, ray_y,
              shadow_xmin_, shadow_ymin_, shadow_xmax_, shadow_ymax_);
            if (ray_self_occluded) {
              delegated_self_shadow = true;
              continue;
            }
            check.state = FootprintState::UNKNOWN;
            check.reason = "invalid_laser_ray";
            check.laser_ray_index = index;
            check.laser_ray_angle = ray_angle;
            check.laser_ray_range = measured;
            check.laser_required_range = distance + sweep_epsilon;
            check.detail = "invalid laser ray is not geometrically explained by self-occlusion";
            set_failure_point(check, x, y);
            return check;
          }
          clear_range = std::min(clear_range, ray_range);
          valid_ray = true;
        }
        if (valid_ray && distance + sweep_epsilon >= clear_range) {
          check.state = FootprintState::UNKNOWN;
          check.reason = "insufficient_laser_clearance";
          check.laser_ray_index = center;
          check.laser_ray_angle = angle;
          check.laser_ray_range = clear_range;
          check.laser_required_range = distance + sweep_epsilon;
          check.detail = "laser clearance ends before the swept footprint boundary";
          set_failure_point(check, x, y);
          return check;
        }
        if (!valid_ray && !delegated_self_shadow) {
          check.state = FootprintState::UNKNOWN;
          check.reason = "invalid_laser_ray";
          check.laser_ray_index = center;
          check.laser_ray_angle = angle;
          check.laser_required_range = distance + sweep_epsilon;
          check.detail = "no valid or geometrically self-occluded ray in neighborhood";
          set_failure_point(check, x, y);
        }
        return check;
      };

    for (std::size_t edge = 0; edge < polygon.size(); ++edge) {
      const auto & a = polygon[edge];
      const auto & b = polygon[(edge + 1) % polygon.size()];
      const double length = std::hypot(
        b.first - a.first, b.second - a.second);
      const std::size_t samples = std::max<std::size_t>(
        1, static_cast<std::size_t>(
          std::ceil(length / known_space_resolution_)));
      for (std::size_t sample = 0; sample <= samples; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const FootprintCheck check = point_is_known(
          a.first + ratio * (b.first - a.first),
          a.second + ratio * (b.second - a.second));
        if (check.state != FootprintState::FREE) {
          return check;
        }
      }
    }
    return result;
  }

  crowd_aware_interfaces::msg::DynamicDwalPath tracePath(
    std::size_t index, double curvature, double required_stop,
    const cas::CurvatureWindow & curvature_window) const
  {
    crowd_aware_interfaces::msg::DynamicDwalPath path;
    path.index = static_cast<std::uint32_t>(index);
    path.curvature = curvature;
    path.required_stop_length = required_stop;
    path.test_angular_speed = curvature * test_speed_.test;
    path.cluster_id = -1;
    path.known_length = max_search_length_;
    path.collision_length = std::numeric_limits<double>::infinity();
    path.first_failure_length = std::numeric_limits<double>::infinity();
    path.costmap_cell_x = -1;
    path.costmap_cell_y = -1;
    path.costmap_cell_value = -1;
    path.laser_ray_index = -1;
    path.laser_ray_angle = std::numeric_limits<double>::quiet_NaN();
    path.laser_ray_range = std::numeric_limits<double>::quiet_NaN();
    path.laser_required_range = std::numeric_limits<double>::quiet_NaN();

    const bool angular_window =
      path.test_angular_speed >= curvature_window.angular_min - 1.0e-9 &&
      path.test_angular_speed <= curvature_window.angular_max + 1.0e-9;
    const bool wheels = cas::wheelSpeedFeasible(
      test_speed_.test, path.test_angular_speed,
      wheel_separation_, wheel_radius_, max_wheel_angular_speed_);
    const bool angular_braking =
      std::abs(curvature) * deceleration_ <= angular_deceleration_ + 1.0e-9;
    const bool transition_after_stop =
      required_stop < cas::arcLineTransition(curvature);
    path.kinematically_feasible =
      angular_window && wheels && angular_braking && transition_after_stop;

    double arc_length = 0.0;
    double last_free_length = 0.0;
    bool have_certified_pose = false;
    const double transition = cas::arcLineTransition(curvature);
    std::vector<std::vector<std::pair<double, double>>> certified_footprints;
    certified_footprints.push_back(footprint_);

    while (true) {
      const double local_k = cas::localCurvature(curvature, arc_length);
      // Non-selectable members are still checked and visualized, but can use
      // a coarser conservative sweep: the larger motion bound expands both
      // costmap cells and required laser clearance. Command-eligible members
      // always retain the configured safety sweep resolution.
      const double trace_resolution = path.kinematically_feasible ?
        sweep_resolution_ :
        std::max(sweep_resolution_, ineligible_sweep_resolution_);
      const double step = std::min(
        path_output_resolution_,
        trace_resolution /
        std::max(1.0, 1.0 + footprint_max_radius_ * std::abs(local_k)));
      const double motion_bound = !have_certified_pose ? 0.0 :
        (arc_length - last_free_length) *
        (1.0 + footprint_max_radius_ * std::abs(curvature));
      const cas::Pose2 pose = cas::arcLinePose(curvature, arc_length);
      FootprintCheck check = footprintState(pose, motion_bound);
      if (check.state == FootprintState::FREE) {
        check = footprintKnownByScan(
          pose, motion_bound, certified_footprints);
      }
      if (check.state != FootprintState::FREE) {
        // Only the last fully checked pose is certified. The failing pose is
        // diagnostic evidence and is never emitted as a safe path point.
        path.free_length = have_certified_pose ? last_free_length : 0.0;
        path.first_failure_length = arc_length;
        path.first_failure_pose.x = pose.x;
        path.first_failure_pose.y = pose.y;
        path.first_failure_pose.theta = pose.theta;
        path.first_failure_point.x = check.point_x;
        path.first_failure_point.y = check.point_y;
        path.costmap_cell_x = check.costmap_cell_x;
        path.costmap_cell_y = check.costmap_cell_y;
        path.costmap_cell_value = check.costmap_cell_value;
        path.laser_ray_index = check.laser_ray_index;
        path.laser_ray_angle = check.laser_ray_angle;
        path.laser_ray_range = check.laser_ray_range;
        path.laser_required_range = check.laser_required_range;
        path.failure_detail = check.detail;
        path.termination_reason = check.reason;
        if (check.state == FootprintState::UNKNOWN) {
          path.known_length = path.free_length;
        } else {
          path.collision_length = arc_length;
        }
        break;
      }

      last_free_length = arc_length;
      have_certified_pose = true;
      certified_footprints.push_back(
        transformPolygon(footprint_, pose.x, pose.y, pose.theta));

      if (arc_length >= max_search_length_ - 1.0e-10) {
        path.free_length = max_search_length_;
        path.termination_reason = "maximum_search_length";
        path.failure_detail = "configured search cap reached";
        break;
      }
      double next_arc_length = std::min(max_search_length_, arc_length + step);
      // Certify the exact arc-line junction instead of stepping across it.
      if (transition > arc_length + 1.0e-10 &&
        transition < next_arc_length - 1.0e-10)
      {
        next_arc_length = transition;
      }
      arc_length = next_arc_length;
    }

    // Emit regular samples, the exact transition when certified, and the last
    // certified point. The failing pose above is deliberately excluded.
    std::vector<double> output_lengths{0.0};
    for (double s = path_output_resolution_;
      s < path.free_length - 1.0e-10; s += path_output_resolution_)
    {
      output_lengths.push_back(s);
    }
    if (std::isfinite(transition) &&
      transition <= path.free_length + 1.0e-10)
    {
      output_lengths.push_back(transition);
    }
    output_lengths.push_back(path.free_length);
    std::sort(output_lengths.begin(), output_lengths.end());
    output_lengths.erase(
      std::unique(
        output_lengths.begin(), output_lengths.end(),
        [](double a, double b) {return std::abs(a - b) <= 1.0e-10;}),
      output_lengths.end());
    for (const double s : output_lengths) {
      const cas::Pose2 pose = cas::arcLinePose(curvature, s);
      geometry_msgs::msg::Pose2D output_pose;
      output_pose.x = pose.x;
      output_pose.y = pose.y;
      output_pose.theta = pose.theta;
      path.poses.push_back(output_pose);
      path.arc_lengths.push_back(s);
    }

    path.statically_admissible =
      path.kinematically_feasible &&
      cas::hasStaticBrakingClearance(required_stop, path.free_length);
    path.prediction_admissible = mode_ == "off";
    path.admissible = path.statically_admissible && path.prediction_admissible;
    if (!angular_window) {
      path.rejection_reason = "outside_angular_dynamic_window";
    } else if (!wheels) {
      path.rejection_reason = "wheel_speed_limit";
    } else if (!angular_braking) {
      path.rejection_reason = "angular_braking_limit";
    } else if (!transition_after_stop) {
      path.rejection_reason = "upstream_arc_line_curvature_jump_within_stop";
    } else if (!cas::hasStaticBrakingClearance(required_stop, path.free_length)) {
      path.rejection_reason = "insufficient_static_braking_length";
    } else {
      path.rejection_reason = "none";
    }
    return path;
  }

  const cohan_msgs::msg::TrackedSegment * torsoSegment(
    const cohan_msgs::msg::TrackedAgent & agent) const
  {
    for (const auto & segment : agent.segments) {
      if (segment.type == cohan_msgs::msg::TrackedSegmentType::TORSO) {
        return &segment;
      }
    }
    return nullptr;
  }

  bool predictionAdmissible(
    const crowd_aware_interfaces::msg::DynamicDwalPath & path,
    double track_age, std::string & rejection) const
  {
    const double stopping_time =
      cas::stopTime(test_speed_.test, reaction_time_, deceleration_);
    if (stopping_time > prediction_horizon_ + 1.0e-9) {
      rejection = "fallback_stop_exceeds_prediction_horizon";
      return false;
    }

    for (double tau = 0.0; tau <= stopping_time + 1.0e-9;
      tau += prediction_time_step_)
    {
      const double distance = cas::distanceAtTime(
        test_speed_.test, reaction_time_, deceleration_,
        std::min(tau, stopping_time));
      const auto robot_polygon =
        footprintAt(cas::arcLinePose(path.curvature, distance));
      for (const auto & agent : tracks_->agents) {
        if (agent.type != cohan_msgs::msg::AgentType::HUMAN) {
          continue;
        }
        const auto * segment_ptr = torsoSegment(agent);
        if (!segment_ptr) {
          rejection = "human_track_without_torso_segment";
          return false;
        }
        const auto & segment = *segment_ptr;
        const double vx = segment.twist.twist.linear.x;
        const double vy = segment.twist.twist.linear.y;
        const double speed = std::hypot(vx, vy);
        const double prediction_time = track_age + tau;
        const double human_x =
          segment.pose.pose.position.x + vx * prediction_time;
        const double human_y =
          segment.pose.pose.position.y + vy * prediction_time;
        // Half a temporal interval of relative motion conservatively covers
        // the gap between simultaneous prediction samples.
        const double temporal_margin =
          0.5 * (test_speed_.test + speed) * prediction_time_step_;
        const double radius =
          human_radius_ + human_base_uncertainty_ +
          human_uncertainty_growth_ * prediction_time + temporal_margin;
        if (cas::distancePointPolygon(
            human_x, human_y, robot_polygon) <= radius)
        {
          rejection = "predicted_human_footprint_intersection";
          return false;
        }
      }
    }
    rejection = "none";
    return true;
  }

  struct MotionCheck
  {
    bool admissible{false};
    std::string reason;
  };

  double angularSpeedAtTime(double angular, double time) const
  {
    if (time <= reaction_time_) {
      return angular;
    }
    const double magnitude = std::max(
      0.0, std::abs(angular) -
      angular_deceleration_ * (time - reaction_time_));
    return std::copysign(magnitude, angular);
  }

  std::vector<cas::Pose2> brakingTrajectory(
    double linear, double angular) const
  {
    const double linear_stop_time = linear > 1.0e-12 ?
      reaction_time_ + linear / deceleration_ : 0.0;
    const double angular_stop_time = std::abs(angular) > 1.0e-12 ?
      reaction_time_ + std::abs(angular) / angular_deceleration_ : 0.0;
    const double stopping_time = std::max(
      linear_stop_time, angular_stop_time);
    std::vector<cas::Pose2> result;
    cas::Pose2 pose;
    result.push_back(pose);
    double tau = 0.0;
    while (tau < stopping_time - 1.0e-9) {
      const double dt = std::min(
        prediction_time_step_, stopping_time - tau);
      const double midpoint = tau + 0.5 * dt;
      const double v_mid = cas::speedAtTime(
        linear, reaction_time_, deceleration_, midpoint);
      const double w_mid = angularSpeedAtTime(angular, midpoint);
      const double heading_mid = pose.theta + 0.5 * w_mid * dt;
      pose.x += v_mid * std::cos(heading_mid) * dt;
      pose.y += v_mid * std::sin(heading_mid) * dt;
      pose.theta += w_mid * dt;
      tau += dt;
      result.push_back(pose);
    }
    return result;
  }

  void setBrakingPoses(
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    double linear, double angular) const
  {
    state.braking_poses.clear();
    for (const cas::Pose2 & pose : brakingTrajectory(linear, angular)) {
      geometry_msgs::msg::Pose2D output;
      output.x = pose.x;
      output.y = pose.y;
      output.theta = pose.theta;
      state.braking_poses.push_back(output);
    }
  }

  MotionCheck checkFinalMotion(
    double linear, double angular, double track_age) const
  {
    const double linear_stop_time = linear > 1.0e-12 ?
      reaction_time_ + linear / deceleration_ : 0.0;
    const double angular_stop_time = std::abs(angular) > 1.0e-12 ?
      reaction_time_ + std::abs(angular) / angular_deceleration_ : 0.0;
    const double stopping_time =
      std::max(linear_stop_time, angular_stop_time);
    if (mode_ == "on" &&
      stopping_time > prediction_horizon_ + 1.0e-9)
    {
      return {false, "final_stop_exceeds_prediction_horizon"};
    }

    cas::Pose2 pose;
    std::vector<std::vector<std::pair<double, double>>> certified_footprints;
    certified_footprints.push_back(footprint_);
    double tau = 0.0;
    while (true) {
      const double v = cas::speedAtTime(
        linear, reaction_time_, deceleration_, tau);
      const double w = angularSpeedAtTime(angular, tau);
      const double dt = std::min(
        prediction_time_step_, std::max(0.0, stopping_time - tau));
      const double motion_bound =
        dt * (std::abs(v) + footprint_max_radius_ * std::abs(w));
      FootprintCheck footprint_check = footprintState(pose, motion_bound);
      if (footprint_check.state == FootprintState::FREE) {
        footprint_check = footprintKnownByScan(
          pose, motion_bound, certified_footprints);
      }
      if (footprint_check.state == FootprintState::COLLISION) {
        return {false, "final_" + footprint_check.reason};
      }
      if (footprint_check.state == FootprintState::UNKNOWN) {
        return {false, "final_" + footprint_check.reason};
      }
      certified_footprints.push_back(
        transformPolygon(footprint_, pose.x, pose.y, pose.theta));

      if (mode_ == "on") {
        const auto robot_polygon = footprintAt(pose);
        for (const auto & agent : tracks_->agents) {
          if (agent.type != cohan_msgs::msg::AgentType::HUMAN) {
            continue;
          }
          const auto * segment = torsoSegment(agent);
          if (!segment) {
            return {false, "human_track_without_torso_segment"};
          }
          const double vx = segment->twist.twist.linear.x;
          const double vy = segment->twist.twist.linear.y;
          const double human_speed = std::hypot(vx, vy);
          const double prediction_time = track_age + tau;
          const double human_x =
            segment->pose.pose.position.x + vx * prediction_time;
          const double human_y =
            segment->pose.pose.position.y + vy * prediction_time;
          const double temporal_margin =
            0.5 * (std::abs(v) + human_speed) * prediction_time_step_;
          const double radius =
            human_radius_ + human_base_uncertainty_ +
            human_uncertainty_growth_ * prediction_time + temporal_margin;
          if (cas::distancePointPolygon(
              human_x, human_y, robot_polygon) <= radius)
          {
            return {false, "final_predicted_human_intersection"};
          }
        }
      }

      if (tau >= stopping_time - 1.0e-9) {
        break;
      }
      const double midpoint = tau + 0.5 * dt;
      const double v_mid = cas::speedAtTime(
        linear, reaction_time_, deceleration_, midpoint);
      const double w_mid = angularSpeedAtTime(angular, midpoint);
      const double heading_mid = pose.theta + 0.5 * w_mid * dt;
      pose.x += v_mid * std::cos(heading_mid) * dt;
      pose.y += v_mid * std::sin(heading_mid) * dt;
      pose.theta += w_mid * dt;
      tau += dt;
    }
    return {true, "final_motion_and_braking_admissible"};
  }

  void assignClusters(crowd_aware_interfaces::msg::DynamicDwalState & state)
  {
    std::vector<bool> admissible;
    admissible.reserve(state.paths.size());
    for (const auto & path : state.paths) {
      admissible.push_back(path.admissible);
    }
    const auto groups = cas::clusterContiguous(admissible);
    for (std::size_t id = 0; id < groups.size(); ++id) {
      crowd_aware_interfaces::msg::DynamicDwalCluster cluster;
      cluster.id = static_cast<std::int32_t>(id);
      cluster.min_curvature =
        state.paths[groups[id].front()].curvature;
      cluster.max_curvature =
        state.paths[groups[id].back()].curvature;
      for (const std::size_t path_index : groups[id]) {
        state.paths[path_index].cluster_id = cluster.id;
        cluster.path_indices.push_back(
          static_cast<std::uint32_t>(path_index));
      }
      state.clusters.push_back(std::move(cluster));
    }
  }

  void appendBrakingMarkers(
    const crowd_aware_interfaces::msg::DynamicDwalState & state,
    visualization_msgs::msg::MarkerArray & array, int & marker_id) const
  {
    if (state.braking_poses.empty()) {
      return;
    }
    visualization_msgs::msg::Marker sweep;
    sweep.header = state.header;
    sweep.ns = state.in_place_rotation ?
      "dynamic_dwal_rotation_swept_footprint" :
      "dynamic_dwal_braking_swept_footprint";
    sweep.id = marker_id++;
    sweep.type = visualization_msgs::msg::Marker::LINE_LIST;
    sweep.action = visualization_msgs::msg::Marker::ADD;
    sweep.pose.orientation.w = 1.0;
    sweep.scale.x = candidate_marker_width_;
    sweep.color = state.in_place_rotation ?
      color(1.0F, 0.35F, 0.95F, 0.65F) :
      color(0.35F, 0.85F, 1.0F, 0.55F);
    for (const auto & pose : state.braking_poses) {
      const auto polygon = footprintAt({pose.x, pose.y, pose.theta});
      for (std::size_t edge = 0; edge < polygon.size(); ++edge) {
        const auto & a = polygon[edge];
        const auto & b = polygon[(edge + 1) % polygon.size()];
        geometry_msgs::msg::Point pa;
        pa.x = a.first;
        pa.y = a.second;
        pa.z = 0.035;
        geometry_msgs::msg::Point pb = pa;
        pb.x = b.first;
        pb.y = b.second;
        sweep.points.push_back(pa);
        sweep.points.push_back(pb);
      }
    }
    array.markers.push_back(std::move(sweep));

    const auto & stop = state.braking_poses.back();
    const auto stop_polygon = footprintAt({stop.x, stop.y, stop.theta});
    visualization_msgs::msg::Marker final_outline;
    final_outline.header = state.header;
    final_outline.ns = "dynamic_dwal_predicted_stop_footprint";
    final_outline.id = marker_id++;
    final_outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
    final_outline.action = visualization_msgs::msg::Marker::ADD;
    final_outline.pose.orientation.w = 1.0;
    final_outline.scale.x = selected_marker_width_;
    final_outline.color = color(1.0F, 1.0F, 1.0F, 0.9F);
    for (std::size_t i = 0; i <= stop_polygon.size(); ++i) {
      geometry_msgs::msg::Point point;
      point.x = stop_polygon[i % stop_polygon.size()].first;
      point.y = stop_polygon[i % stop_polygon.size()].second;
      point.z = 0.045;
      final_outline.points.push_back(point);
    }
    array.markers.push_back(std::move(final_outline));
  }

  void publishMarkers(
    const crowd_aware_interfaces::msg::DynamicDwalState & state)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    int marker_id = 0;
    if (state.paths.empty()) {
      const double cache_age = last_family_stamp_.nanoseconds() == 0 ?
        std::numeric_limits<double>::infinity() :
        std::max(0.0, (now() - last_family_stamp_).seconds());
      if (cache_age <= stale_marker_lifetime_) {
        for (const auto & cached : last_family_markers_.markers) {
          if (cached.action == visualization_msgs::msg::Marker::DELETEALL) {
            continue;
          }
          auto stale = cached;
          stale.header.stamp = state.header.stamp;
          stale.ns = "stale_" + cached.ns;
          stale.id = marker_id++;
          stale.color.a = std::min(stale.color.a, 0.25F);
          stale.lifetime = rclcpp::Duration::from_seconds(
            std::max(0.01, stale_marker_lifetime_ - cache_age));
          if (stale.type == visualization_msgs::msg::Marker::TEXT_VIEW_FACING) {
            stale.text = "OLD " + stale.text;
          }
          array.markers.push_back(std::move(stale));
        }
      }
      appendBrakingMarkers(state, array, marker_id);
      marker_publisher_->publish(array);
      return;
    }

    for (const auto & path : state.paths) {
      visualization_msgs::msg::Marker line;
      line.header = state.header;
      line.ns = "dynamic_dwal_paths";
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = candidate_marker_width_;
      if (path.admissible) {
        line.color = clusterColor(path.cluster_id);
      } else if (
        path.rejection_reason == "predicted_human_footprint_intersection")
      {
        line.color = color(1.0F, 0.5F, 0.0F);
      } else if (path.kinematically_feasible) {
        line.color = color(0.9F, 0.1F, 0.1F);
      } else {
        line.color = color(0.45F, 0.45F, 0.45F, 0.45F);
      }
      for (const auto & pose : path.poses) {
        const cas::Pose2 global = globalPose({pose.x, pose.y, pose.theta});
        geometry_msgs::msg::Point point;
        point.x = global.x;
        point.y = global.y;
        point.z = 0.06;
        line.points.push_back(point);
      }
      array.markers.push_back(std::move(line));

      if (path.poses.size() == 1U) {
        visualization_msgs::msg::Marker point_marker;
        point_marker.header = state.header;
        point_marker.ns = "dynamic_dwal_zero_length";
        point_marker.id = marker_id++;
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        const auto & only = path.poses.front();
        const cas::Pose2 global =
          globalPose({only.x, only.y, only.theta});
        point_marker.pose.position.x = global.x;
        point_marker.pose.position.y = global.y;
        point_marker.pose.position.z = 0.06;
        point_marker.pose.orientation.w = 1.0;
        point_marker.scale.x = transition_marker_scale_;
        point_marker.scale.y = transition_marker_scale_;
        point_marker.scale.z = transition_marker_scale_;
        point_marker.color = path.admissible ?
          clusterColor(path.cluster_id) : color(0.9F, 0.1F, 0.1F);
        array.markers.push_back(std::move(point_marker));
      }
      const double transition = cas::arcLineTransition(path.curvature);
      if (std::isfinite(transition) &&
        transition <= path.free_length + 1.0e-10)
      {
        const cas::Pose2 local = cas::arcLinePose(path.curvature, transition);
        const cas::Pose2 global = globalPose(local);
        visualization_msgs::msg::Marker junction;
        junction.header = state.header;
        junction.ns = "dynamic_dwal_arc_line_transition";
        junction.id = marker_id++;
        junction.type = visualization_msgs::msg::Marker::SPHERE;
        junction.action = visualization_msgs::msg::Marker::ADD;
        junction.pose.position.x = global.x;
        junction.pose.position.y = global.y;
        junction.pose.position.z = 0.075;
        junction.pose.orientation.w = 1.0;
        junction.scale.x = transition_marker_scale_;
        junction.scale.y = transition_marker_scale_;
        junction.scale.z = transition_marker_scale_;
        junction.color = path.admissible ?
          clusterColor(path.cluster_id) : color(0.75F, 0.75F, 0.75F, 0.75F);
        array.markers.push_back(std::move(junction));
      }
      if (std::isfinite(path.first_failure_length) &&
        std::isfinite(path.first_failure_point.x) &&
        std::isfinite(path.first_failure_point.y))
      {
        visualization_msgs::msg::Marker failure;
        failure.header = state.header;
        failure.ns = "dynamic_dwal_first_failure";
        failure.id = marker_id++;
        failure.type = visualization_msgs::msg::Marker::SPHERE;
        failure.action = visualization_msgs::msg::Marker::ADD;
        failure.pose.position = path.first_failure_point;
        failure.pose.position.z = 0.055;
        failure.pose.orientation.w = 1.0;
        failure.scale.x = 0.018;
        failure.scale.y = 0.018;
        failure.scale.z = 0.018;
        failure.color = color(1.0F, 0.1F, 0.1F, 0.75F);
        array.markers.push_back(std::move(failure));
      }
      if (marker_labels_ && !path.poses.empty()) {
        visualization_msgs::msg::Marker label;
        label.header = state.header;
        label.ns = "dynamic_dwal_labels";
        label.id = marker_id++;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        const auto & endpoint = path.poses.back();
        const cas::Pose2 global =
          globalPose({endpoint.x, endpoint.y, endpoint.theta});
        label.pose.position.x = global.x;
        label.pose.position.y = global.y;
        label.pose.position.z = 0.16;
        label.pose.orientation.w = 1.0;
        label.scale.z = marker_label_scale_;
        label.color = color(1.0F, 1.0F, 1.0F);
        label.text =
          "i=" + std::to_string(path.index) +
          " k=" + std::to_string(path.curvature).substr(0, 5) +
          " L=" + std::to_string(path.free_length).substr(0, 4) +
          " c=" + std::to_string(path.cluster_id) +
          " " + path.rejection_reason;
        array.markers.push_back(std::move(label));
      }
    }
    if (state.selected_path_index >= 0 &&
      static_cast<std::size_t>(state.selected_path_index) < state.paths.size())
    {
      const auto & selected =
        state.paths[static_cast<std::size_t>(state.selected_path_index)];
      visualization_msgs::msg::Marker highlight;
      highlight.header = state.header;
      highlight.ns = "dynamic_dwal_selected";
      highlight.id = marker_id++;
      highlight.type = visualization_msgs::msg::Marker::LINE_STRIP;
      highlight.action = visualization_msgs::msg::Marker::ADD;
      highlight.pose.orientation.w = 1.0;
      highlight.scale.x = selected_marker_width_;
      highlight.color = color(0.0F, 1.0F, 1.0F);
      for (const auto & pose : selected.poses) {
        const cas::Pose2 global = globalPose({pose.x, pose.y, pose.theta});
        geometry_msgs::msg::Point point;
        point.x = global.x;
        point.y = global.y;
        point.z = 0.11;
        highlight.points.push_back(point);
      }
      array.markers.push_back(std::move(highlight));
    }
    appendBrakingMarkers(state, array, marker_id);
    marker_publisher_->publish(array);
    last_family_markers_ = array;
    last_family_stamp_ = now();
  }

  void setCommand(
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    double linear, double angular, const std::string & reason,
    bool final_motion_admissible, bool in_place_rotation = false)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    command_publisher_->publish(command);
    state.command_published = true;
    state.selected_linear = linear;
    state.selected_angular = angular;
    state.final_motion_admissible = final_motion_admissible;
    state.in_place_rotation = in_place_rotation;
    state.controller_reason = reason;
  }

  void publishInvalid(
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    const std::string & reason)
  {
    state.data_valid = false;
    state.state_reason = reason;
    setCommand(state, 0.0, 0.0, reason, false);
    state_publisher_->publish(state);
    publishMarkers(state);
  }

  void tick()
  {
    {
      // Immutable input snapshot for this potentially expensive full-family
      // evaluation. Subscription callbacks keep receiving fresh sensor data
      // concurrently for the next cycle.
      std::lock_guard<std::mutex> lock(input_mutex_);
      odom_ = latest_odom_;
      costmap_ = latest_costmap_;
      scan_ = latest_scan_;
      reference_ = latest_reference_;
      tracks_ = latest_tracks_;
      odom_received_ = latest_odom_received_;
      costmap_received_ = latest_costmap_received_;
      scan_received_ = latest_scan_received_;
      reference_received_ = latest_reference_received_;
      tracks_received_ = latest_tracks_received_;
    }
    crowd_aware_interfaces::msg::DynamicDwalState state;
    state.header.stamp = now();
    state.header.frame_id = global_frame_;
    state.cycle = cycle_++;
    state.mode = mode_;
    state.selected_path_index = -1;
    state.selected_cluster_id = -1;

    state.odom_age = stampedAge(
      odom_ ? odom_->header.stamp : builtin_interfaces::msg::Time(),
      odom_received_, static_cast<bool>(odom_));
    state.costmap_age = stampedAge(
      costmap_ ? costmap_->header.stamp : builtin_interfaces::msg::Time(),
      costmap_received_, static_cast<bool>(costmap_));
    state.scan_age = stampedAge(
      scan_ ? scan_->header.stamp : builtin_interfaces::msg::Time(),
      scan_received_, static_cast<bool>(scan_));
    state.reference_age = receiveAge(
      reference_received_, static_cast<bool>(reference_));
    state.tracks_age = mode_ == "on" ?
      stampedAge(
        tracks_ ? tracks_->header.stamp : builtin_interfaces::msg::Time(),
        tracks_received_, static_cast<bool>(tracks_)) : -1.0;

    state.odom_fresh = state.odom_age <= odom_timeout_;
    state.costmap_fresh = state.costmap_age <= costmap_timeout_;
    state.scan_fresh = state.scan_age <= scan_timeout_;
    state.reference_fresh = state.reference_age <= reference_timeout_;
    state.tracks_fresh =
      mode_ == "on" && state.tracks_age <= tracks_timeout_;

    if (!state.odom_fresh) {
      publishInvalid(state, "missing_or_stale_odometry");
      return;
    }
    if (!state.costmap_fresh) {
      publishInvalid(state, "missing_or_stale_costmap");
      return;
    }
    if (odom_->header.frame_id != global_frame_ ||
      odom_->child_frame_id != base_frame_)
    {
      publishInvalid(state, "odometry_frame_mismatch");
      return;
    }
    if (!state.scan_fresh) {
      publishInvalid(state, "missing_or_stale_scan");
      return;
    }
    if (costmap_->header.frame_id != global_frame_) {
      publishInvalid(state, "costmap_frame_mismatch");
      return;
    }
    if (scan_->header.frame_id != laser_frame_) {
      publishInvalid(state, "scan_frame_mismatch");
      return;
    }

    state.requested_linear =
      reference_ ? reference_->linear.x : 0.0;
    state.requested_angular =
      reference_ ? reference_->angular.z : 0.0;
    state.measured_linear = odom_->twist.twist.linear.x;
    state.measured_angular = odom_->twist.twist.angular.z;
    if (!state.reference_fresh) {
      publishInvalid(state, "missing_or_stale_reference");
      return;
    }
    if (state.requested_linear < 0.0 || state.measured_linear < -1.0e-3) {
      publishInvalid(state, "reverse_translation_outside_scope");
      return;
    }

    test_speed_ = cas::computeTestSpeed(
      state.requested_linear, state.measured_linear, max_linear_,
      acceleration_, deceleration_, dynamic_window_dt_);
    state.test_linear = test_speed_.test;
    state.dynamic_v_min = test_speed_.window_min;
    state.dynamic_v_max = test_speed_.window_max;
    const auto curvature_window = cas::computeCurvatureWindow(
      state.measured_angular, test_speed_.test, max_angular_,
      angular_acceleration_, dynamic_window_dt_, max_curvature_,
      zero_linear_epsilon_);
    state.dynamic_w_min = curvature_window.angular_min;
    state.dynamic_w_max = curvature_window.angular_max;
    state.required_stop_length = cas::stoppingDistance(
      test_speed_.test, reaction_time_, deceleration_,
      longitudinal_margin_);

    if (mode_ == "on") {
      if (!state.tracks_fresh || !tracks_) {
        publishInvalid(state, "missing_or_stale_tracks");
        return;
      }
      if (tracks_->header.frame_id != global_frame_) {
        publishInvalid(state, "tracks_frame_mismatch");
        return;
      }
    }

    const bool zero_reference =
      std::abs(state.requested_linear) <= zero_linear_epsilon_ &&
      std::abs(state.requested_angular) <= zero_linear_epsilon_;
    if (zero_reference) {
      const auto braking = checkFinalMotion(
        std::max(0.0, state.measured_linear), state.measured_angular,
        state.tracks_age);
      setBrakingPoses(
        state, std::max(0.0, state.measured_linear),
        state.measured_angular);
      state.data_valid = true;
      state.state_reason = "teleop_zero";
      setCommand(
        state, 0.0, 0.0, "teleop_zero_controlled_stop",
        braking.admissible);
      state_publisher_->publish(state);
      publishMarkers(state);
      return;
    }

    const bool rotation_request =
      state.requested_linear <= zero_linear_epsilon_ &&
      std::abs(state.requested_angular) > zero_linear_epsilon_;
    if (rotation_request) {
      state.data_valid = true;
      state.state_reason = "in_place_rotation";
      if (state.measured_linear > zero_linear_epsilon_) {
        const auto braking = checkFinalMotion(
          state.measured_linear, state.measured_angular, state.tracks_age);
        setBrakingPoses(
          state, state.measured_linear, state.measured_angular);
        setCommand(
          state, 0.0, 0.0, "rotation_waiting_for_translation_stop",
          braking.admissible, true);
      } else {
        const double target_angular = std::clamp(
          state.requested_angular, curvature_window.angular_min,
          curvature_window.angular_max);
        const bool same_direction =
          target_angular * state.requested_angular > 0.0;
        const bool wheels = cas::wheelSpeedFeasible(
          0.0, target_angular, wheel_separation_, wheel_radius_,
          max_wheel_angular_speed_);
        const double test_angular =
          std::abs(state.measured_angular) > std::abs(target_angular) ?
          state.measured_angular : target_angular;
        setBrakingPoses(state, 0.0, test_angular);
        const auto test_check =
          checkFinalMotion(0.0, test_angular, state.tracks_age);
        const auto final_check =
          checkFinalMotion(0.0, target_angular, state.tracks_age);
        if (same_direction && wheels &&
          test_check.admissible && final_check.admissible)
        {
          setCommand(
            state, 0.0, target_angular,
            "safe_in_place_rotation", true, true);
        } else {
          const std::string reason = !same_direction ?
            "rotation_direction_not_reachable" :
            (!wheels ? "rotation_wheel_speed_limit" : final_check.reason);
          setCommand(state, 0.0, 0.0, reason, false, true);
        }
      }
      state_publisher_->publish(state);
      publishMarkers(state);
      return;
    }

    // Keep the visual/geometric family stable. Dynamic-window membership is a
    // per-path eligibility test in tracePath(), not a sampling boundary.
    const auto curvatures = cas::sampleSymmetricCurvatures(
      max_curvature_, curvature_step_);
    state.sampled_curvature_count =
      static_cast<std::uint32_t>(curvatures.size());
    if (!curvatures.empty()) {
      state.sampled_curvature_min = curvatures.front();
      state.sampled_curvature_max = curvatures.back();
    }
    state.paths.reserve(curvatures.size());
    for (std::size_t i = 0; i < curvatures.size(); ++i) {
      state.paths.push_back(
        tracePath(i, curvatures[i], state.required_stop_length,
        curvature_window));
    }

    if (mode_ == "on") {
      for (auto & path : state.paths) {
        if (!path.statically_admissible) {
          path.prediction_admissible = false;
          path.admissible = false;
          continue;
        }
        std::string prediction_rejection;
        path.prediction_admissible = predictionAdmissible(
          path, state.tracks_age, prediction_rejection);
        path.admissible =
          path.statically_admissible && path.prediction_admissible;
        if (!path.prediction_admissible) {
          path.rejection_reason = prediction_rejection;
        }
      }
    }

    assignClusters(state);
    state.admissible_count = static_cast<std::uint32_t>(std::count_if(
      state.paths.begin(), state.paths.end(),
      [](const auto & path) {return path.admissible;}));
    state.data_valid = true;

    const double command_linear = test_speed_.reachable_target;
    std::vector<std::pair<double, std::size_t>> ranked;
    for (std::size_t i = 0; i < state.paths.size(); ++i) {
      const auto & path = state.paths[i];
      if (!path.admissible) {
        continue;
      }
      const double command_angular = path.curvature * command_linear;
      const bool angular_reachable =
        command_angular >= curvature_window.angular_min - 1.0e-9 &&
        command_angular <= curvature_window.angular_max + 1.0e-9;
      const bool wheels = cas::wheelSpeedFeasible(
        command_linear, command_angular, wheel_separation_, wheel_radius_,
        max_wheel_angular_speed_);
      if (!angular_reachable || !wheels) {
        continue;
      }
      const double intent_cost =
        std::abs(command_angular - state.requested_angular) /
        std::max(max_angular_, 1.0e-6);
      ranked.emplace_back(
        intent_cost + 1.0e-5 * std::abs(path.curvature), i);
    }
    std::sort(ranked.begin(), ranked.end());

    const auto current_braking = checkFinalMotion(
      std::max(0.0, state.measured_linear), state.measured_angular,
      state.tracks_age);
    std::string last_failure = current_braking.reason;
    bool selected = false;
    if (current_braking.admissible) {
      for (const auto & candidate : ranked) {
        const std::size_t index = candidate.second;
        const auto & path = state.paths[index];
        const double low_speed =
          std::min(command_linear, test_speed_.test);
        const double high_speed =
          std::max(command_linear, test_speed_.test);
        const std::size_t checks = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(
            (high_speed - low_speed) / final_speed_resolution_)));
        bool final_safe = true;
        for (std::size_t n = 0; n <= checks; ++n) {
          const double ratio =
            static_cast<double>(n) / static_cast<double>(checks);
          const double checked_speed =
            low_speed + ratio * (high_speed - low_speed);
          const auto final_check = checkFinalMotion(
            checked_speed, path.curvature * checked_speed,
            state.tracks_age);
          if (!final_check.admissible) {
            final_safe = false;
            last_failure = final_check.reason;
            break;
          }
        }
        if (!final_safe) {
          continue;
        }

        state.selected_path_index = static_cast<std::int32_t>(index);
        state.selected_cluster_id = path.cluster_id;
        setBrakingPoses(
          state, command_linear, path.curvature * command_linear);
        setCommand(
          state, command_linear, path.curvature * command_linear,
          "selected_admissible_cluster_member", true);
        selected = true;
        break;
      }
    }

    if (selected) {
      state.state_reason = "ok";
    } else {
      state.state_reason = state.paths.empty() ?
        "no_feasible_curvature_samples" : "no_safe_selected_motion";
      setBrakingPoses(
        state, std::max(0.0, state.measured_linear),
        state.measured_angular);
      setCommand(
        state, 0.0, 0.0,
        ranked.empty() ? "no_admissible_cluster_member" : last_failure,
        false);
    }
    state_publisher_->publish(state);
    publishMarkers(state);
  }

  std::string mode_;
  std::string global_frame_;
  std::string base_frame_;
  std::string odom_topic_;
  std::string costmap_topic_;
  std::string scan_topic_;
  std::string laser_frame_;
  std::string reference_topic_;
  std::string tracks_topic_;
  std::string diagnostics_topic_;
  std::string markers_topic_;
  std::string command_topic_;

  double max_linear_;
  double max_angular_;
  double acceleration_;
  double deceleration_;
  double angular_acceleration_;
  double angular_deceleration_;
  double dynamic_window_dt_;
  double max_curvature_;
  double curvature_step_;
  double zero_linear_epsilon_;
  double wheel_separation_;
  double wheel_radius_;
  double max_wheel_angular_speed_;
  double max_search_length_;
  double path_output_resolution_;
  double sweep_resolution_;
  double ineligible_sweep_resolution_;
  double footprint_padding_;
  double known_space_resolution_;
  double laser_x_;
  double laser_y_;
  double laser_yaw_;
  double candidate_marker_width_;
  double selected_marker_width_;
  double transition_marker_scale_;
  bool marker_labels_;
  double marker_label_scale_;
  double stale_marker_lifetime_;
  int lethal_cost_;
  double reaction_time_;
  double longitudinal_margin_;
  double odom_timeout_;
  double costmap_timeout_;
  double scan_timeout_;
  double reference_timeout_;
  double tracks_timeout_;
  double publish_rate_;
  double prediction_horizon_;
  double prediction_time_step_;
  double final_speed_resolution_;
  double human_radius_;
  double human_base_uncertainty_;
  double human_uncertainty_growth_;
  double footprint_max_radius_;
  double shadow_xmin_;
  double shadow_xmax_;
  double shadow_ymin_;
  double shadow_ymax_;
  std::vector<std::pair<double, double>> footprint_;
  cas::SpeedState test_speed_;
  std::uint64_t cycle_{0};
  visualization_msgs::msg::MarkerArray last_family_markers_;
  rclcpp::Time last_family_stamp_{0, 0, RCL_ROS_TIME};

  nav_msgs::msg::Odometry::SharedPtr odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  geometry_msgs::msg::Twist::SharedPtr reference_;
  cohan_msgs::msg::TrackedAgents::SharedPtr tracks_;
  rclcpp::Time odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time costmap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tracks_received_{0, 0, RCL_ROS_TIME};

  std::mutex input_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  geometry_msgs::msg::Twist::SharedPtr latest_reference_;
  cohan_msgs::msg::TrackedAgents::SharedPtr latest_tracks_;
  rclcpp::Time latest_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_costmap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_reference_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_tracks_received_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<
    crowd_aware_interfaces::msg::DynamicDwalState>::SharedPtr state_publisher_;
  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    command_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    costmap_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
    reference_subscription_;
  rclcpp::Subscription<cohan_msgs::msg::TrackedAgents>::SharedPtr
    tracks_subscription_;
  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DynamicDwalNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("dynamic_dwal"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
