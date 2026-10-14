#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <cohan_msgs/msg/agent_type.hpp>
#include <cohan_msgs/msg/tracked_agents.hpp>
#include <cohan_msgs/msg/tracked_segment_type.hpp>
#include <crowd_aware_interfaces/msg/dynamic_dwal_cluster.hpp>
#include <crowd_aware_interfaces/msg/dynamic_dwal_path.hpp>
#include <crowd_aware_interfaces/msg/dynamic_dwal_state.hpp>
#include <crowd_aware_simulation/dynamic_dwal_core.hpp>
#include <dwal_planner/msg/sampled_cluster.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace cas = crowd_aware_simulation;
using std::placeholders::_1;

namespace
{

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<std::pair<double, double>> transformPolygon(
  const std::vector<std::pair<double, double>> & polygon,
  double x, double y, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<std::pair<double, double>> result;
  result.reserve(polygon.size());
  for (const auto & point : polygon) {
    result.emplace_back(
      x + c * point.first - s * point.second,
      y + s * point.first + c * point.second);
  }
  return result;
}

bool overlap(double amin, double amax, double bmin, double bmax)
{
  return amax >= bmin && bmax >= amin;
}

bool pointInConvexPolygon(
  double x, double y, const std::vector<std::pair<double, double>> & polygon)
{
  double reference = 0.0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double cross =
      (b.first - a.first) * (y - a.second) -
      (b.second - a.second) * (x - a.first);
    if (std::abs(cross) <= 1.0e-10) {
      continue;
    }
    if (reference == 0.0) {
      reference = cross;
    } else if ((cross > 0.0) != (reference > 0.0)) {
      return false;
    }
  }
  return true;
}

bool polygonIntersectsRectangle(
  const std::vector<std::pair<double, double>> & polygon,
  double xmin, double ymin, double xmax, double ymax)
{
  auto separated = [&](double ax, double ay) {
      double pmin = std::numeric_limits<double>::infinity();
      double pmax = -std::numeric_limits<double>::infinity();
      for (const auto & point : polygon) {
        const double p = ax * point.first + ay * point.second;
        pmin = std::min(pmin, p);
        pmax = std::max(pmax, p);
      }
      const double r0 = ax * xmin + ay * ymin;
      const double r1 = ax * xmin + ay * ymax;
      const double r2 = ax * xmax + ay * ymin;
      const double r3 = ax * xmax + ay * ymax;
      return !overlap(
        pmin, pmax, std::min({r0, r1, r2, r3}),
        std::max({r0, r1, r2, r3}));
    };

  if (separated(1.0, 0.0) || separated(0.0, 1.0)) {
    return false;
  }
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    if (separated(-(b.second - a.second), b.first - a.first)) {
      return false;
    }
  }
  return true;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0F)
{
  std_msgs::msg::ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;
  return result;
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

class DynamicDwalClustering : public rclcpp::Node
{
public:
  DynamicDwalClustering()
  : Node("dwal_clustering")
  {
    // The unchanged generator sets this parameter through this node's service.
    (void)declare_parameter<int>("dwal_clustering/Marker_num", -1);

    mode_ = declare_parameter<std::string>("mode", "off");
    if (mode_ != "off" && mode_ != "on") {
      throw std::runtime_error("mode must be 'off' or 'on'");
    }

    global_frame_ = declare_parameter<std::string>("global_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "sim_base");
    sampled_topic_ = declare_parameter<std::string>("sampled_paths_topic", "sampled_paths");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    costmap_topic_ = declare_parameter<std::string>(
      "costmap_topic", "/local_costmap/costmap");
    reference_topic_ = declare_parameter<std::string>(
      "reference_topic", "/reference_cmd");
    tracks_topic_ = declare_parameter<std::string>(
      "tracks_topic", "/tracked_agents");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser_frame");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/dynamic_dwal/diagnostics");
    markers_topic_ = declare_parameter<std::string>(
      "markers_topic", "/dynamic_dwal/markers");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/cmd_vel_selected");

    max_linear_ = declare_parameter<double>("max_linear", 0.30);
    max_angular_ = declare_parameter<double>("max_angular", 0.80);
    acceleration_ = declare_parameter<double>("acceleration", 0.50);
    a_brake_ = declare_parameter<double>("a_brake", 0.50);
    angular_acceleration_ = declare_parameter<double>(
      "angular_acceleration", 1.0);
    angular_deceleration_ = declare_parameter<double>(
      "angular_deceleration", 1.0);
    dynamic_window_dt_ = declare_parameter<double>("dynamic_window_dt", 0.10);
    max_curvature_ = declare_parameter<double>("max_curvature", 4.0);
    zero_linear_epsilon_ = declare_parameter<double>(
      "zero_linear_epsilon", 0.02);
    wheel_separation_ = declare_parameter<double>("wheel_separation", 0.61);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.095);
    max_wheel_angular_speed_ = declare_parameter<double>(
      "max_wheel_angular_speed", 20.0);

    max_search_length_ = declare_parameter<double>("max_search_length", 4.0);
    path_output_resolution_ = declare_parameter<double>(
      "path_output_resolution", 0.05);
    footprint_padding_ = declare_parameter<double>("footprint_padding", 0.02);
    lethal_cost_ = declare_parameter<int>("lethal_cost", 100);
    reaction_time_ = declare_parameter<double>("T_reaction", 0.40);
    longitudinal_margin_ = declare_parameter<double>("S_margin", 0.05);

    cluster_separation_ = declare_parameter<int>(
      "cluster_separation", 5);
    min_cluster_span_ = declare_parameter<double>(
      "min_cluster_span", 0.20);
    cluster_reference_radius_ = declare_parameter<double>(
      "cluster_reference_radius", 1.80);

    prediction_horizon_ = declare_parameter<double>(
      "prediction_horizon", 2.0);
    prediction_time_step_ = declare_parameter<double>(
      "prediction_time_step", 0.05);
    human_radius_ = declare_parameter<double>("human_radius", 0.40);
    human_base_uncertainty_ = declare_parameter<double>(
      "human_base_uncertainty", 0.10);
    human_uncertainty_growth_ = declare_parameter<double>(
      "human_uncertainty_growth", 0.10);
    laser_x_ = declare_parameter<double>("laser_x", 0.885);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    odom_timeout_ = declare_parameter<double>("odom_timeout", 0.50);
    costmap_timeout_ = declare_parameter<double>("costmap_timeout", 0.50);
    reference_timeout_ = declare_parameter<double>("reference_timeout", 0.35);
    tracks_timeout_ = declare_parameter<double>("tracks_timeout", 0.50);
    scan_timeout_ = declare_parameter<double>("scan_timeout", 0.50);
    sampled_timeout_ = declare_parameter<double>("sampled_paths_timeout", 0.50);
    publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
    marker_width_ = declare_parameter<double>("candidate_marker_width", 0.005);
    selected_marker_width_ = declare_parameter<double>(
      "selected_marker_width", 0.010);

    const auto footprint_values = declare_parameter<std::vector<double>>(
      "footprint", std::vector<double>{
        0.865, 0.405, 0.865, -0.405, -0.103, -0.405, -0.103, 0.405});
    if (footprint_values.size() < 6 || footprint_values.size() % 2 != 0) {
      throw std::runtime_error("footprint must contain at least three x,y pairs");
    }
    std::vector<geometry_msgs::msg::Point> padded_footprint;
    padded_footprint.reserve(footprint_values.size() / 2);
    for (std::size_t i = 0; i < footprint_values.size(); i += 2) {
      geometry_msgs::msg::Point point;
      point.x = footprint_values[i];
      point.y = footprint_values[i + 1];
      padded_footprint.push_back(point);
    }
    nav2_costmap_2d::padFootprint(padded_footprint, footprint_padding_);
    for (const auto & point : padded_footprint) {
      footprint_.emplace_back(point.x, point.y);
    }

    if (a_brake_ <= 0.0 || reaction_time_ < 0.0 ||
      longitudinal_margin_ < 0.0 || max_search_length_ <= 0.0 ||
      path_output_resolution_ <= 0.0 || prediction_time_step_ <= 0.0 ||
      publish_rate_ <= 0.0 || cluster_separation_ < 0 ||
      min_cluster_span_ < 0.0 || cluster_reference_radius_ <= 0.0)
    {
      throw std::runtime_error("invalid dynamic DWAL safety parameter");
    }

    state_publisher_ =
      create_publisher<crowd_aware_interfaces::msg::DynamicDwalState>(
      diagnostics_topic_, rclcpp::QoS(10));
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic_, rclcpp::QoS(10));
    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>(
      command_topic_, rclcpp::QoS(10));

    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sampled_subscription_ =
      create_subscription<dwal_planner::msg::SampledCluster>(
      sampled_topic_, rclcpp::QoS(2),
      [this](dwal_planner::msg::SampledCluster::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_sampled_ = std::move(msg);
        latest_sampled_received_ = now();
      }, options);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_odom_ = std::move(msg);
        latest_odom_received_ = now();
      }, options);
    reference_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      reference_topic_, rclcpp::QoS(20),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_reference_ = std::move(msg);
        latest_reference_received_ = now();
      }, options);
    costmap_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_costmap_ = std::move(msg);
        latest_costmap_received_ = now();
      }, options);
    // Tracks are intentionally required in both modes. OFF freezes each torso
    // at p(0); ON is the sole switch that enables p(0)+v*t.
    tracks_subscription_ = create_subscription<cohan_msgs::msg::TrackedAgents>(
      tracks_topic_, rclcpp::QoS(20),
      [this](cohan_msgs::msg::TrackedAgents::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_tracks_ = std::move(msg);
        latest_tracks_received_ = now();
      }, options);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_scan_ = std::move(msg);
        latest_scan_received_ = now();
      }, options);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&DynamicDwalClustering::tick, this));
    RCLCPP_INFO(
      get_logger(),
      "dynamic DWAL clustering/controller mode=%s; cluster gap=%d indices, min span=%.3f m",
      mode_.c_str(), cluster_separation_, min_cluster_span_);
  }

private:
  struct GridCheck
  {
    bool free{true};
    bool unknown{false};
    std::string reason{"free"};
    double x{std::numeric_limits<double>::quiet_NaN()};
    double y{std::numeric_limits<double>::quiet_NaN()};
    int ix{-1};
    int iy{-1};
    int value{-1};
  };

  struct PreviousCluster
  {
    std::int32_t id;
    double mean_curvature;
  };

  double receiveAge(const rclcpp::Time & received, bool present) const
  {
    if (!present || received.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    const double age = (now() - received).seconds();
    return age < 0.0 ? std::numeric_limits<double>::infinity() : age;
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
    return std::max(rx_age, source_age < 0.0 ?
      std::numeric_limits<double>::infinity() : source_age);
  }

  cas::Pose2 globalPose(const cas::Pose2 & local) const
  {
    const double yaw = yawOf(odom_->pose.pose.orientation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {
      odom_->pose.pose.position.x + c * local.x - s * local.y,
      odom_->pose.pose.position.y + s * local.x + c * local.y,
      yaw + local.theta};
  }

  std::vector<std::pair<double, double>> footprintAt(
    const cas::Pose2 & local) const
  {
    const auto global = globalPose(local);
    return transformPolygon(footprint_, global.x, global.y, global.theta);
  }

  const cohan_msgs::msg::TrackedSegment * torso(
    const cohan_msgs::msg::TrackedAgent & agent) const
  {
    for (const auto & segment : agent.segments) {
      if (segment.type == cohan_msgs::msg::TrackedSegmentType::TORSO) {
        return &segment;
      }
    }
    return nullptr;
  }

  bool pointBelongsToTrackedHuman(
    double x, double y, double track_age, double extra) const
  {
    if (!tracks_) {
      return false;
    }
    for (const auto & agent : tracks_->agents) {
      if (agent.type != cohan_msgs::msg::AgentType::HUMAN) {
        continue;
      }
      const auto * segment = torso(agent);
      if (!segment) {
        continue;
      }
      const auto human = cas::predictedPosition(
        segment->pose.pose.position.x,
        segment->pose.pose.position.y,
        segment->twist.twist.linear.x,
        segment->twist.twist.linear.y,
        track_age, mode_ == "on");
      const double hx = human.first;
      const double hy = human.second;
      if (std::hypot(x - hx, y - hy) <=
        human_radius_ + human_base_uncertainty_ + extra)
      {
        return true;
      }
    }
    return false;
  }

  bool scanCertifiesFree(double x, double y) const
  {
    if (!scan_ || !cas::isFresh(scan_age_, scan_timeout_) ||
      scan_->header.frame_id != laser_frame_ || scan_->ranges.empty() ||
      scan_->angle_increment <= 0.0F || scan_->range_max <= 0.0F)
    {
      return false;
    }

    const auto laser = globalPose(cas::Pose2{laser_x_, laser_y_, laser_yaw_});
    const double dx = x - laser.x;
    const double dy = y - laser.y;
    const double local_x = std::cos(laser.theta) * dx + std::sin(laser.theta) * dy;
    const double local_y = -std::sin(laser.theta) * dx + std::cos(laser.theta) * dy;
    const double angle = std::atan2(local_y, local_x);
    const double fractional =
      (angle - static_cast<double>(scan_->angle_min)) /
      static_cast<double>(scan_->angle_increment);
    const long index = std::lround(fractional);
    if (index < 0 || index >= static_cast<long>(scan_->ranges.size())) {
      return false;
    }

    const double measured = static_cast<double>(scan_->ranges[index]);
    if (std::isnan(measured) || measured < static_cast<double>(scan_->range_min)) {
      return false;
    }
    const double usable = std::isinf(measured) && measured > 0.0 ?
      static_cast<double>(scan_->range_max) :
      std::min(measured, static_cast<double>(scan_->range_max));
    if (!std::isfinite(usable)) {
      return false;
    }

    // The whole cell, rather than only its centre, must be before the return.
    // The scan/odom age term is a conservative allowance for using the latest
    // pose instead of interpolating an old sensor transform.
    const double half_cell_diagonal =
      std::sqrt(0.5) * static_cast<double>(costmap_->info.resolution);
    const double motion_allowance =
      std::abs(odom_->twist.twist.linear.x) * std::max(0.0, scan_age_);
    return usable >
      std::hypot(local_x, local_y) + half_cell_diagonal + motion_allowance;
  }

  GridCheck footprintState(
    const cas::Pose2 & local, double track_age) const
  {
    GridCheck result;
    const auto global_polygon = footprintAt(local);
    const auto current_global = footprintAt(cas::Pose2{});
    const auto & origin = costmap_->info.origin;
    const double map_yaw = yawOf(origin.orientation);
    const double c = std::cos(map_yaw);
    const double s = std::sin(map_yaw);
    auto to_map = [&](const std::vector<std::pair<double, double>> & source) {
        std::vector<std::pair<double, double>> output;
        output.reserve(source.size());
        for (const auto & point : source) {
          const double dx = point.first - origin.position.x;
          const double dy = point.second - origin.position.y;
          output.emplace_back(c * dx + s * dy, -s * dx + c * dy);
        }
        return output;
      };
    const auto polygon = to_map(global_polygon);
    const auto current_polygon = to_map(current_global);
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

    const double resolution = costmap_->info.resolution;
    if (resolution <= 0.0) {
      return {false, true, "invalid_costmap_resolution"};
    }
    const int ix0 = static_cast<int>(std::floor(xmin / resolution));
    const int ix1 = static_cast<int>(std::floor(xmax / resolution));
    const int iy0 = static_cast<int>(std::floor(ymin / resolution));
    const int iy1 = static_cast<int>(std::floor(ymax / resolution));
    auto record = [&](GridCheck & check, int ix, int iy, int value) {
        const double mx = (static_cast<double>(ix) + 0.5) * resolution;
        const double my = (static_cast<double>(iy) + 0.5) * resolution;
        check.x = origin.position.x + c * mx - s * my;
        check.y = origin.position.y + s * mx + c * my;
        check.ix = ix;
        check.iy = iy;
        check.value = value;
      };

    for (int iy = iy0; iy <= iy1; ++iy) {
      for (int ix = ix0; ix <= ix1; ++ix) {
        if (!polygonIntersectsRectangle(
            polygon, ix * resolution, iy * resolution,
            (ix + 1) * resolution, (iy + 1) * resolution))
        {
          continue;
        }
        if (ix < 0 || iy < 0 ||
          ix >= static_cast<int>(costmap_->info.width) ||
          iy >= static_cast<int>(costmap_->info.height))
        {
          result = {false, true, "costmap_boundary"};
          record(result, ix, iy, -1);
          return result;
        }
        const std::size_t index =
          static_cast<std::size_t>(iy) * costmap_->info.width +
          static_cast<std::size_t>(ix);
        const int value = static_cast<int>(costmap_->data[index]);
        if (value < 0) {
          const double cell_x = (static_cast<double>(ix) + 0.5) * resolution;
          const double cell_y = (static_cast<double>(iy) + 0.5) * resolution;
          // OccupancyGrid values describe their cell centres. Ignore a cell
          // touched only by the conservative rectangle-overlap rasterization;
          // lethal cells still retain that more conservative test below.
          if (!pointInConvexPolygon(cell_x, cell_y, polygon)) {
            continue;
          }
          const bool currently_occupied =
            pointInConvexPolygon(cell_x, cell_y, current_polygon);
          GridCheck unknown{false, true, "unknown_costmap_cell"};
          record(unknown, ix, iy, value);
          if (!currently_occupied && !scanCertifiesFree(unknown.x, unknown.y)) {
            return unknown;
          }
        } else if (value >= lethal_cost_) {
          GridCheck collision{false, false, "lethal_footprint_collision"};
          record(collision, ix, iy, value);
          // Only cells explained by a fresh identified torso are delegated to
          // the simultaneous human predictor. Unknown lidar returns remain
          // lethal and are never erased or extended through.
          if (pointBelongsToTrackedHuman(
              collision.x, collision.y, track_age,
              std::sqrt(2.0) * resolution))
          {
            continue;
          }
          return collision;
        }
      }
    }
    return result;
  }

  crowd_aware_interfaces::msg::DynamicDwalPath tracePath(
    std::uint32_t original_index, double curvature,
    double required_stop, const cas::CurvatureWindow & window,
    double track_age) const
  {
    crowd_aware_interfaces::msg::DynamicDwalPath path;
    path.index = original_index;
    path.curvature = curvature;
    path.required_stop_length = required_stop;
    path.test_angular_speed = curvature * speed_.test;
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
      window.valid &&
      path.test_angular_speed >= window.angular_min - 1.0e-9 &&
      path.test_angular_speed <= window.angular_max + 1.0e-9;
    const bool wheels = cas::wheelSpeedFeasible(
      speed_.test, path.test_angular_speed, wheel_separation_,
      wheel_radius_, max_wheel_angular_speed_);
    const bool angular_braking =
      std::abs(curvature) * a_brake_ <= angular_deceleration_ + 1.0e-9;
    const bool transition_after_stop =
      required_stop < cas::arcLineTransition(curvature);
    path.kinematically_feasible =
      angular_window && wheels && angular_braking && transition_after_stop;

    for (double arc = 0.0; arc <= max_search_length_ + 1.0e-10;
      arc += path_output_resolution_)
    {
      const double checked_arc = std::min(arc, max_search_length_);
      const auto pose = cas::arcLinePose(curvature, checked_arc);
      const auto check = footprintState(pose, track_age);
      if (!check.free) {
        path.first_failure_length = checked_arc;
        path.first_failure_pose.x = pose.x;
        path.first_failure_pose.y = pose.y;
        path.first_failure_pose.theta = pose.theta;
        path.first_failure_point.x = check.x;
        path.first_failure_point.y = check.y;
        path.costmap_cell_x = check.ix;
        path.costmap_cell_y = check.iy;
        path.costmap_cell_value = check.value;
        path.failure_detail =
          "first non-certified footprint cell in the rolling costmap";
        path.termination_reason = check.reason;
        if (check.unknown) {
          path.known_length = cas::polylineLength(path.poses);
        } else {
          path.collision_length = cas::polylineLength(path.poses);
        }
        break;
      }

      geometry_msgs::msg::Pose2D output;
      output.x = pose.x;
      output.y = pose.y;
      output.theta = pose.theta;
      path.poses.push_back(output);
      path.arc_lengths.push_back(checked_arc);
      if (checked_arc >= max_search_length_ - 1.0e-10) {
        path.termination_reason = "maximum_search_length";
        path.failure_detail = "configured, fully checked search cap reached";
        break;
      }
      if (arc + path_output_resolution_ > max_search_length_ &&
        arc < max_search_length_)
      {
        arc = max_search_length_ - path_output_resolution_;
      }
    }

    // This is S_i from the sampled points, not radial collision_r and not the
    // integration parameter used to generate the curve.
    path.free_length = cas::polylineLength(path.poses);
    path.statically_admissible =
      path.kinematically_feasible &&
      cas::hasStaticBrakingClearance(required_stop, path.free_length);
    path.prediction_admissible = false;
    path.admissible = false;
    if (!angular_window) {
      path.rejection_reason = "outside_angular_dynamic_window";
    } else if (!wheels) {
      path.rejection_reason = "wheel_speed_limit";
    } else if (!angular_braking) {
      path.rejection_reason = "angular_braking_limit";
    } else if (!transition_after_stop) {
      path.rejection_reason = "arc_line_transition_inside_braking_motion";
    } else if (!path.statically_admissible) {
      path.rejection_reason = "insufficient_available_path_length";
    } else {
      path.rejection_reason = "prediction_pending";
    }
    return path;
  }

  bool humanAdmissible(
    const crowd_aware_interfaces::msg::DynamicDwalPath & path,
    double robot_speed, double track_age, std::string & rejection) const
  {
    const double stop_time =
      cas::stopTime(robot_speed, reaction_time_, a_brake_);
    if (stop_time > prediction_horizon_ + 1.0e-9) {
      rejection = "stop_exceeds_prediction_horizon";
      return false;
    }
    for (double tau = 0.0; tau <= stop_time + 1.0e-9;
      tau += prediction_time_step_)
    {
      const double time = std::min(tau, stop_time);
      const double distance = cas::distanceAtTime(
        robot_speed, reaction_time_, a_brake_, time);
      const auto robot = footprintAt(
        cas::arcLinePose(path.curvature, distance));
      for (const auto & agent : tracks_->agents) {
        if (agent.type != cohan_msgs::msg::AgentType::HUMAN) {
          continue;
        }
        const auto * segment = torso(agent);
        if (!segment) {
          rejection = "human_track_without_torso";
          return false;
        }
        const double vx = segment->twist.twist.linear.x;
        const double vy = segment->twist.twist.linear.y;
        const auto human = cas::predictedPosition(
          segment->pose.pose.position.x,
          segment->pose.pose.position.y,
          vx, vy, track_age + time, mode_ == "on");
        const double hx = human.first;
        const double hy = human.second;
        const double uncertainty_time = track_age + time;
        const double interpolation_margin =
          0.5 * (robot_speed + std::hypot(vx, vy)) *
          prediction_time_step_;
        const double extent =
          human_radius_ + human_base_uncertainty_ +
          human_uncertainty_growth_ * uncertainty_time +
          interpolation_margin;
        if (cas::distancePointPolygon(hx, hy, robot) <= extent) {
          rejection = mode_ == "on" ?
            "predicted_moving_human_intersection" :
            "predicted_static_human_intersection";
          return false;
        }
      }
    }
    rejection = "none";
    return true;
  }

  double angularAtTime(double angular, double time) const
  {
    if (time <= reaction_time_) {
      return angular;
    }
    return std::copysign(
      std::max(
        0.0, std::abs(angular) -
        angular_deceleration_ * (time - reaction_time_)),
      angular);
  }

  bool finalMotionAdmissible(
    double linear, double angular, double track_age,
    std::vector<geometry_msgs::msg::Pose2D> * output,
    std::string & rejection) const
  {
    const double linear_stop = linear > 1.0e-12 ?
      reaction_time_ + linear / a_brake_ : 0.0;
    const double angular_stop = std::abs(angular) > 1.0e-12 ?
      reaction_time_ + std::abs(angular) / angular_deceleration_ : 0.0;
    const double stop_time = std::max(linear_stop, angular_stop);
    if (stop_time > prediction_horizon_ + 1.0e-9) {
      rejection = "final_stop_exceeds_prediction_horizon";
      return false;
    }

    cas::Pose2 pose;
    double tau = 0.0;
    while (true) {
      geometry_msgs::msg::Pose2D stored;
      stored.x = pose.x;
      stored.y = pose.y;
      stored.theta = pose.theta;
      if (output) {
        output->push_back(stored);
      }
      const auto grid = footprintState(pose, track_age);
      if (!grid.free) {
        rejection = "final_" + grid.reason;
        return false;
      }
      const auto robot = footprintAt(pose);
      for (const auto & agent : tracks_->agents) {
        if (agent.type != cohan_msgs::msg::AgentType::HUMAN) {
          continue;
        }
        const auto * segment = torso(agent);
        if (!segment) {
          rejection = "human_track_without_torso";
          return false;
        }
        const double vx = segment->twist.twist.linear.x;
        const double vy = segment->twist.twist.linear.y;
        const auto human = cas::predictedPosition(
          segment->pose.pose.position.x,
          segment->pose.pose.position.y,
          vx, vy, track_age + tau, mode_ == "on");
        const double hx = human.first;
        const double hy = human.second;
        const double extent =
          human_radius_ + human_base_uncertainty_ +
          human_uncertainty_growth_ * (track_age + tau) +
          0.5 * (std::abs(cas::speedAtTime(
            linear, reaction_time_, a_brake_, tau)) +
          std::hypot(vx, vy)) * prediction_time_step_;
        if (cas::distancePointPolygon(hx, hy, robot) <= extent) {
          rejection = "final_human_intersection";
          return false;
        }
      }
      if (tau >= stop_time - 1.0e-9) {
        break;
      }
      const double dt = std::min(prediction_time_step_, stop_time - tau);
      const double middle = tau + 0.5 * dt;
      const double v = cas::speedAtTime(
        linear, reaction_time_, a_brake_, middle);
      const double w = angularAtTime(angular, middle);
      const double heading = pose.theta + 0.5 * w * dt;
      pose.x += v * std::cos(heading) * dt;
      pose.y += v * std::sin(heading) * dt;
      pose.theta += w * dt;
      tau += dt;
    }
    rejection = "final_motion_and_braking_admissible";
    return true;
  }

  double pathMaximumRadius(
    const crowd_aware_interfaces::msg::DynamicDwalPath & path) const
  {
    double result = 0.0;
    for (const auto & pose : path.poses) {
      result = std::max(result, std::hypot(pose.x, pose.y));
    }
    return result;
  }

  double pathAngleAtRadius(
    const crowd_aware_interfaces::msg::DynamicDwalPath & path,
    double radius) const
  {
    if (path.poses.empty()) {
      return 0.0;
    }
    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1];
      const auto & b = path.poses[i];
      const double ra = std::hypot(a.x, a.y);
      const double rb = std::hypot(b.x, b.y);
      if (ra <= radius && rb >= radius && rb > ra + 1.0e-12) {
        const double u = (radius - ra) / (rb - ra);
        return std::atan2(
          a.y + u * (b.y - a.y),
          a.x + u * (b.x - a.x));
      }
    }
    const auto & end = path.poses.back();
    return std::atan2(end.y, end.x);
  }

  void closeCluster(
    const std::vector<std::size_t> & members,
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    std::vector<std::vector<std::size_t>> & accepted) const
  {
    if (members.empty()) {
      return;
    }
    // The existing far DWAL level is a geometric observation ring. Only
    // members which actually reach it enter a cluster; paths are never cut at
    // this radius and braking distance is not reused as clustering geometry.
    const double reference_radius = cluster_reference_radius_;
    const double first_angle =
      pathAngleAtRadius(state.paths[members.front()], reference_radius);
    const double last_angle =
      pathAngleAtRadius(state.paths[members.back()], reference_radius);
    const double span = cas::clusterChordSpan(
      reference_radius, first_angle, last_angle);
    if (span + 1.0e-12 >= min_cluster_span_) {
      accepted.push_back(members);
    }
  }

  std::vector<std::int32_t> matchClusterIds(
    const std::vector<double> & means)
  {
    std::vector<std::int32_t> ids(means.size(), -1);
    std::set<std::size_t> used_previous;
    std::set<std::size_t> used_current;
    std::set<std::int32_t> reserved_ids;
    for (const auto & previous : previous_clusters_) {
      reserved_ids.insert(previous.id);
    }

    // Preserve the original global-minimum one-to-one correspondence. Mean
    // curvature replaces mean phi so a changing span radius cannot by itself
    // change an otherwise identical match.
    const std::size_t matches =
      std::min(previous_clusters_.size(), means.size());
    for (std::size_t match = 0; match < matches; ++match) {
      double best = std::numeric_limits<double>::infinity();
      std::size_t best_previous = previous_clusters_.size();
      std::size_t best_current = means.size();
      for (std::size_t previous = 0;
        previous < previous_clusters_.size(); ++previous)
      {
        if (used_previous.count(previous)) {
          continue;
        }
        for (std::size_t current = 0; current < means.size(); ++current) {
          if (used_current.count(current)) {
            continue;
          }
          const double distance = std::abs(
            means[current] - previous_clusters_[previous].mean_curvature);
          if (distance < best) {
            best = distance;
            best_previous = previous;
            best_current = current;
          }
        }
      }
      if (best_previous < previous_clusters_.size() &&
        best_current < means.size())
      {
        ids[best_current] = previous_clusters_[best_previous].id;
        used_previous.insert(best_previous);
        used_current.insert(best_current);
      }
    }

    for (std::size_t current = 0; current < means.size(); ++current) {
      if (ids[current] >= 0) {
        continue;
      }
      std::int32_t id = 0;
      while (reserved_ids.count(id)) {
        ++id;
      }
      ids[current] = id;
      reserved_ids.insert(id);
    }

    previous_clusters_.clear();
    for (std::size_t i = 0; i < means.size(); ++i) {
      previous_clusters_.push_back({ids[i], means[i]});
    }
    return ids;
  }

  void assignClusters(crowd_aware_interfaces::msg::DynamicDwalState & state)
  {
    std::vector<std::vector<std::size_t>> groups;
    std::vector<std::size_t> current;
    for (std::size_t i = 0; i < state.paths.size(); ++i) {
      if (!state.paths[i].admissible ||
        pathMaximumRadius(state.paths[i]) + 1.0e-12 < cluster_reference_radius_)
      {
        continue;
      }
      if (current.empty()) {
        current.push_back(i);
        continue;
      }
      const auto previous_original = state.paths[current.back()].index;
      const auto current_original = state.paths[i].index;
      if (cas::startsNewCluster(
          previous_original, current_original,
          static_cast<std::uint32_t>(cluster_separation_)))
      {
        closeCluster(current, state, groups);
        current.clear();
      }
      // The old code started a new group for Dind > cluster_separation but
      // appended only for Dind <= 2. One parameter now governs both branches.
      current.push_back(i);
    }
    closeCluster(current, state, groups);

    std::vector<double> means;
    means.reserve(groups.size());
    for (const auto & group : groups) {
      means.push_back(0.5 * (
        state.paths[group.front()].curvature +
        state.paths[group.back()].curvature));
    }
    const auto ids = matchClusterIds(means);
    for (std::size_t n = 0; n < groups.size(); ++n) {
      crowd_aware_interfaces::msg::DynamicDwalCluster cluster;
      cluster.id = ids[n];
      cluster.min_curvature =
        state.paths[groups[n].front()].curvature;
      cluster.max_curvature =
        state.paths[groups[n].back()].curvature;
      cluster.reference_radius = cluster_reference_radius_;
      for (const auto index : groups[n]) {
        state.paths[index].cluster_id = cluster.id;
        cluster.path_indices.push_back(state.paths[index].index);
      }
      const double a0 =
        pathAngleAtRadius(state.paths[groups[n].front()],
        cluster.reference_radius);
      const double a1 =
        pathAngleAtRadius(state.paths[groups[n].back()],
        cluster.reference_radius);
      cluster.span = cas::clusterChordSpan(
        cluster.reference_radius, a0, a1);
      state.clusters.push_back(std::move(cluster));
    }
    if (groups.empty()) {
      previous_clusters_.clear();
    }
  }

  void publishCommand(
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    double linear, double angular, const std::string & reason,
    bool admissible)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear;
    command.angular.z = angular;
    command_publisher_->publish(command);
    state.command_published = true;
    state.selected_linear = linear;
    state.selected_angular = angular;
    state.controller_reason = reason;
    state.final_motion_admissible = admissible;
  }

  void publishMarkers(
    const crowd_aware_interfaces::msg::DynamicDwalState & state)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker clear;
    clear.header = state.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;
    for (const auto & path : state.paths) {
      visualization_msgs::msg::Marker marker;
      marker.header = state.header;
      marker.ns = "dynamic_dwal_paths";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker_width_;
      marker.color = path.admissible ?
        clusterColor(path.cluster_id) : color(0.45F, 0.45F, 0.45F, 0.35F);
      for (const auto & local : path.poses) {
        const auto global = globalPose({local.x, local.y, local.theta});
        geometry_msgs::msg::Point point;
        point.x = global.x;
        point.y = global.y;
        point.z = 0.08;
        marker.points.push_back(point);
      }
      array.markers.push_back(std::move(marker));
      if (state.selected_path_index >= 0 &&
        path.index == static_cast<std::uint32_t>(state.selected_path_index))
      {
        auto selected = array.markers.back();
        selected.ns = "dynamic_dwal_selected";
        selected.id = id++;
        selected.scale.x = selected_marker_width_;
        selected.color = color(0.0F, 1.0F, 1.0F);
        array.markers.push_back(std::move(selected));
      }
    }
    marker_publisher_->publish(array);
  }

  void publishInvalid(
    crowd_aware_interfaces::msg::DynamicDwalState & state,
    const std::string & reason)
  {
    state.data_valid = false;
    state.state_reason = reason;
    publishCommand(state, 0.0, 0.0, reason, false);
    state_publisher_->publish(state);
    publishMarkers(state);
  }

  void tick()
  {
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      sampled_ = latest_sampled_;
      odom_ = latest_odom_;
      costmap_ = latest_costmap_;
      reference_ = latest_reference_;
      tracks_ = latest_tracks_;
      scan_ = latest_scan_;
      sampled_received_ = latest_sampled_received_;
      odom_received_ = latest_odom_received_;
      costmap_received_ = latest_costmap_received_;
      reference_received_ = latest_reference_received_;
      tracks_received_ = latest_tracks_received_;
      scan_received_ = latest_scan_received_;
    }

    crowd_aware_interfaces::msg::DynamicDwalState state;
    state.header.stamp = now();
    state.header.frame_id = global_frame_;
    state.cycle = cycle_++;
    state.mode = mode_;
    state.selected_path_index = -1;
    state.selected_cluster_id = -1;
    state.scan_age = stampedAge(
      scan_ ? scan_->header.stamp : builtin_interfaces::msg::Time(),
      scan_received_, static_cast<bool>(scan_));
    state.scan_fresh = cas::isFresh(state.scan_age, scan_timeout_);
    scan_age_ = state.scan_age;

    state.odom_age = stampedAge(
      odom_ ? odom_->header.stamp : builtin_interfaces::msg::Time(),
      odom_received_, static_cast<bool>(odom_));
    state.costmap_age = stampedAge(
      costmap_ ? costmap_->header.stamp : builtin_interfaces::msg::Time(),
      costmap_received_, static_cast<bool>(costmap_));
    state.reference_age = receiveAge(
      reference_received_, static_cast<bool>(reference_));
    state.tracks_age = stampedAge(
      tracks_ ? tracks_->header.stamp : builtin_interfaces::msg::Time(),
      tracks_received_, static_cast<bool>(tracks_));
    const double sampled_age = receiveAge(
      sampled_received_, static_cast<bool>(sampled_));
    state.odom_fresh = cas::isFresh(state.odom_age, odom_timeout_);
    state.costmap_fresh = cas::isFresh(state.costmap_age, costmap_timeout_);
    state.reference_fresh = cas::isFresh(state.reference_age, reference_timeout_);
    state.tracks_fresh = cas::isFresh(state.tracks_age, tracks_timeout_);

    if (!state.odom_fresh) {
      publishInvalid(state, "missing_or_stale_odometry");
      return;
    }
    if (!state.costmap_fresh) {
      publishInvalid(state, "missing_or_stale_costmap");
      return;
    }
    if (!state.reference_fresh) {
      publishInvalid(state, "missing_or_stale_reference");
      return;
    }
    if (!state.tracks_fresh) {
      publishInvalid(state, "missing_or_stale_tracks");
      return;
    }
    if (!state.scan_fresh) {
      publishInvalid(state, "missing_or_stale_scan");
      return;
    }
    if (!cas::isFresh(sampled_age, sampled_timeout_)) {
      publishInvalid(state, "missing_or_stale_sampled_paths");
      return;
    }
    if (odom_->header.frame_id != global_frame_ ||
      odom_->child_frame_id != base_frame_ ||
      costmap_->header.frame_id != global_frame_ ||
      tracks_->header.frame_id != global_frame_ ||
      scan_->header.frame_id != laser_frame_)
    {
      publishInvalid(state, "input_frame_mismatch");
      return;
    }

    state.requested_linear = reference_->linear.x;
    state.requested_angular = reference_->angular.z;
    state.measured_linear = odom_->twist.twist.linear.x;
    state.measured_angular = odom_->twist.twist.angular.z;
    if (state.requested_linear < 0.0 || state.measured_linear < -1.0e-3) {
      publishInvalid(state, "reverse_translation_outside_scope");
      return;
    }

    speed_ = cas::computeTestSpeed(
      state.requested_linear, state.measured_linear, max_linear_,
      acceleration_, a_brake_, dynamic_window_dt_);
    state.test_linear = speed_.test;
    state.dynamic_v_min = speed_.window_min;
    state.dynamic_v_max = speed_.window_max;
    const auto window = cas::computeCurvatureWindow(
      state.measured_angular, speed_.test, max_angular_,
      angular_acceleration_, dynamic_window_dt_, max_curvature_,
      zero_linear_epsilon_);
    state.dynamic_w_min = window.angular_min;
    state.dynamic_w_max = window.angular_max;

    // S_required is deliberately based on current measured speed. A separate
    // controller gate below repeats it for a prospective faster command.
    state.required_stop_length = cas::stoppingDistance(
      std::abs(state.measured_linear), reaction_time_, a_brake_,
      longitudinal_margin_);

    std::vector<std::pair<double, std::uint32_t>> curvatures;
    curvatures.reserve(sampled_->paths.size());
    for (std::uint32_t i = 0; i < sampled_->paths.size(); ++i) {
      curvatures.emplace_back(sampled_->paths[i].curvature, i);
    }
    std::stable_sort(
      curvatures.begin(), curvatures.end(),
      [](const auto & a, const auto & b) {return a.first < b.first;});
    state.sampled_curvature_count =
      static_cast<std::uint32_t>(curvatures.size());
    if (!curvatures.empty()) {
      state.sampled_curvature_min = curvatures.front().first;
      state.sampled_curvature_max = curvatures.back().first;
    }
    state.paths.reserve(curvatures.size());
    for (const auto & sample : curvatures) {
      auto path = tracePath(
        sample.second, sample.first, state.required_stop_length,
        window, state.tracks_age);
      if (path.statically_admissible) {
        std::string rejection;
        path.prediction_admissible = humanAdmissible(
          path, speed_.test, state.tracks_age, rejection);
        path.admissible = path.prediction_admissible;
        path.rejection_reason = rejection;
      }
      state.paths.push_back(std::move(path));
    }

    assignClusters(state);
    state.admissible_count = static_cast<std::uint32_t>(std::count_if(
      state.paths.begin(), state.paths.end(),
      [](const auto & path) {return path.admissible;}));
    state.data_valid = true;

    const bool zero_request =
      std::abs(state.requested_linear) <= zero_linear_epsilon_ &&
      std::abs(state.requested_angular) <= zero_linear_epsilon_;
    if (zero_request) {
      std::string reason;
      state.braking_poses.clear();
      const bool safe = finalMotionAdmissible(
        std::max(0.0, state.measured_linear), state.measured_angular,
        state.tracks_age, &state.braking_poses, reason);
      state.state_reason = "controlled_stop";
      publishCommand(state, 0.0, 0.0, reason, safe);
      state_publisher_->publish(state);
      publishMarkers(state);
      return;
    }

    const bool rotation_request =
      state.requested_linear <= zero_linear_epsilon_ &&
      std::abs(state.requested_angular) > zero_linear_epsilon_;
    if (rotation_request) {
      state.in_place_rotation = true;
      double target = 0.0;
      if (state.measured_linear <= zero_linear_epsilon_) {
        target = std::clamp(
          state.requested_angular, window.angular_min, window.angular_max);
      }
      std::string reason;
      state.braking_poses.clear();
      const bool safe = finalMotionAdmissible(
        std::max(0.0, state.measured_linear), target,
        state.tracks_age, &state.braking_poses, reason);
      state.state_reason = safe ? "in_place_rotation" : "rotation_rejected";
      publishCommand(
        state, 0.0, safe ? target : 0.0, reason, safe);
      state_publisher_->publish(state);
      publishMarkers(state);
      return;
    }

    const double command_linear = speed_.reachable_target;
    const double command_required = cas::stoppingDistance(
      command_linear, reaction_time_, a_brake_, longitudinal_margin_);
    std::vector<std::pair<double, std::size_t>> ranked;
    for (std::size_t i = 0; i < state.paths.size(); ++i) {
      const auto & path = state.paths[i];
      if (!path.admissible || path.cluster_id < 0 ||
        !cas::hasStaticBrakingClearance(
          command_required, path.free_length))
      {
        continue;
      }
      const double angular = path.curvature * command_linear;
      if (angular < window.angular_min - 1.0e-9 ||
        angular > window.angular_max + 1.0e-9 ||
        !cas::wheelSpeedFeasible(
          command_linear, angular, wheel_separation_, wheel_radius_,
          max_wheel_angular_speed_))
      {
        continue;
      }
      ranked.emplace_back(
        std::abs(angular - state.requested_angular) /
        std::max(max_angular_, 1.0e-6), i);
    }
    std::sort(ranked.begin(), ranked.end());

    std::string current_reason;
    const bool current_safe = finalMotionAdmissible(
      std::max(0.0, state.measured_linear), state.measured_angular,
      state.tracks_age, nullptr, current_reason);
    bool selected = false;
    std::string last_reason = current_reason;
    if (current_safe) {
      for (const auto & candidate : ranked) {
        auto & path = state.paths[candidate.second];
        const double angular = path.curvature * command_linear;
        state.braking_poses.clear();
        if (!finalMotionAdmissible(
            command_linear, angular, state.tracks_age,
            &state.braking_poses, last_reason))
        {
          continue;
        }
        state.selected_path_index =
          static_cast<std::int32_t>(path.index);
        state.selected_cluster_id = path.cluster_id;
        publishCommand(
          state, command_linear, angular,
          "selected_real_cluster_member", true);
        selected = true;
        break;
      }
    }
    if (!selected) {
      state.braking_poses.clear();
      std::string ignored;
      (void)finalMotionAdmissible(
        std::max(0.0, state.measured_linear), state.measured_angular,
        state.tracks_age, &state.braking_poses, ignored);
      state.state_reason = state.paths.empty() ?
        "empty_sampled_path_family" : "no_safe_selected_motion";
      publishCommand(
        state, 0.0, 0.0,
        ranked.empty() ? "no_length_and_time_admissible_path" : last_reason,
        false);
    } else {
      state.state_reason = "ok";
    }
    state_publisher_->publish(state);
    publishMarkers(state);
  }

  std::string mode_;
  std::string global_frame_;
  std::string base_frame_;
  std::string sampled_topic_;
  std::string odom_topic_;
  std::string costmap_topic_;
  std::string reference_topic_;
  std::string tracks_topic_;
  std::string scan_topic_;
  std::string laser_frame_;
  std::string diagnostics_topic_;
  std::string markers_topic_;
  std::string command_topic_;

  double max_linear_;
  double max_angular_;
  double acceleration_;
  double a_brake_;
  double angular_acceleration_;
  double angular_deceleration_;
  double dynamic_window_dt_;
  double max_curvature_;
  double zero_linear_epsilon_;
  double wheel_separation_;
  double wheel_radius_;
  double max_wheel_angular_speed_;
  double max_search_length_;
  double path_output_resolution_;
  double footprint_padding_;
  int lethal_cost_;
  double reaction_time_;
  double longitudinal_margin_;
  int cluster_separation_;
  double min_cluster_span_;
  double cluster_reference_radius_;
  double prediction_horizon_;
  double prediction_time_step_;
  double human_radius_;
  double human_base_uncertainty_;
  double human_uncertainty_growth_;
  double laser_x_;
  double laser_y_;
  double laser_yaw_;
  double odom_timeout_;
  double costmap_timeout_;
  double reference_timeout_;
  double tracks_timeout_;
  double scan_timeout_;
  double sampled_timeout_;
  double publish_rate_;
  double marker_width_;
  double selected_marker_width_;
  std::vector<std::pair<double, double>> footprint_;
  cas::SpeedState speed_;
  std::vector<PreviousCluster> previous_clusters_;
  std::uint64_t cycle_{0};

  std::mutex input_mutex_;
  dwal_planner::msg::SampledCluster::SharedPtr latest_sampled_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  geometry_msgs::msg::Twist::SharedPtr latest_reference_;
  cohan_msgs::msg::TrackedAgents::SharedPtr latest_tracks_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_sampled_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_costmap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_reference_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_tracks_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_received_{0, 0, RCL_ROS_TIME};

  dwal_planner::msg::SampledCluster::SharedPtr sampled_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  geometry_msgs::msg::Twist::SharedPtr reference_;
  cohan_msgs::msg::TrackedAgents::SharedPtr tracks_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::Time sampled_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time costmap_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reference_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tracks_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_received_{0, 0, RCL_ROS_TIME};
  double scan_age_{std::numeric_limits<double>::infinity()};

  rclcpp::Publisher<
    crowd_aware_interfaces::msg::DynamicDwalState>::SharedPtr state_publisher_;
  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Subscription<
    dwal_planner::msg::SampledCluster>::SharedPtr sampled_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscription_;
  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr reference_subscription_;
  rclcpp::Subscription<
    cohan_msgs::msg::TrackedAgents>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DynamicDwalClustering>();
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
