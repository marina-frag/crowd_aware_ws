#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace crowd_aware_simulation
{

constexpr double kPi = 3.14159265358979323846;

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct SpeedState
{
  double requested{0.0};
  double measured{0.0};
  double reachable_target{0.0};
  double test{0.0};
  double window_min{0.0};
  double window_max{0.0};
};

struct CurvatureWindow
{
  bool valid{false};
  double minimum{0.0};
  double maximum{0.0};
  double angular_min{0.0};
  double angular_max{0.0};
};

inline double clampFinite(double value, double low, double high)
{
  if (!std::isfinite(value)) {
    return low;
  }
  return std::clamp(value, low, high);
}

inline SpeedState computeTestSpeed(
  double requested, double measured, double max_speed,
  double acceleration, double braking_deceleration, double window_dt)
{
  SpeedState result;
  result.requested = clampFinite(requested, 0.0, max_speed);
  result.measured = clampFinite(measured, 0.0, max_speed);
  result.window_min = std::max(0.0, result.measured - braking_deceleration * window_dt);
  result.window_max = std::min(max_speed, result.measured + acceleration * window_dt);
  result.reachable_target =
    std::clamp(result.requested, result.window_min, result.window_max);
  // A lower request must not hide kinetic energy already present in odometry.
  result.test = std::max(result.measured, result.reachable_target);
  return result;
}

inline double stoppingDistance(
  double speed, double reaction_time, double braking_deceleration,
  double longitudinal_margin)
{
  if (speed < 0.0 || reaction_time < 0.0 || braking_deceleration <= 0.0 ||
    longitudinal_margin < 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  return speed * reaction_time +
         speed * speed / (2.0 * braking_deceleration) +
         longitudinal_margin;
}

inline bool hasStaticBrakingClearance(double required_stop, double free_length)
{
  return std::isfinite(required_stop) && required_stop < free_length;
}

// Exact geometry used by the pinned upstream DWAL implementation:
// constant curvature until |heading change| = pi/2, then the tangent line.
inline Pose2 arcLinePose(double curvature, double arc_length)
{
  Pose2 pose;
  const double s = std::max(0.0, arc_length);
  if (std::abs(curvature) < 1.0e-12) {
    pose.x = s;
    return pose;
  }

  const double arc_end = kPi / (2.0 * std::abs(curvature));
  const double arc_s = std::min(s, arc_end);
  const double heading = curvature * arc_s;
  pose.x = std::sin(heading) / curvature;
  pose.y = (1.0 - std::cos(heading)) / curvature;
  pose.theta = heading;

  if (s > arc_end) {
    const double line_s = s - arc_end;
    pose.x += line_s * std::cos(heading);
    pose.y += line_s * std::sin(heading);
  }
  return pose;
}

inline double localCurvature(double initial_curvature, double arc_length)
{
  if (std::abs(initial_curvature) < 1.0e-12) {
    return 0.0;
  }
  const double arc_end = kPi / (2.0 * std::abs(initial_curvature));
  return arc_length < arc_end ? initial_curvature : 0.0;
}

inline double arcLineTransition(double curvature)
{
  return std::abs(curvature) < 1.0e-12 ?
         std::numeric_limits<double>::infinity() :
         kPi / (2.0 * std::abs(curvature));
}

inline CurvatureWindow computeCurvatureWindow(
  double measured_angular, double test_linear, double max_angular,
  double angular_acceleration, double window_dt, double max_curvature,
  double zero_linear_epsilon)
{
  CurvatureWindow result;
  result.angular_min = std::max(
    -max_angular, measured_angular - angular_acceleration * window_dt);
  result.angular_max = std::min(
    max_angular, measured_angular + angular_acceleration * window_dt);
  if (test_linear <= zero_linear_epsilon ||
    result.angular_min > result.angular_max)
  {
    return result;
  }
  result.minimum = std::max(-max_curvature, result.angular_min / test_linear);
  result.maximum = std::min(max_curvature, result.angular_max / test_linear);
  result.valid = result.minimum <= result.maximum;
  return result;
}

inline bool wheelSpeedFeasible(
  double linear, double angular, double wheel_separation,
  double wheel_radius, double max_wheel_angular_speed)
{
  if (wheel_radius <= 0.0 || max_wheel_angular_speed <= 0.0) {
    return false;
  }
  const double left =
    (linear - 0.5 * angular * wheel_separation) / wheel_radius;
  const double right =
    (linear + 0.5 * angular * wheel_separation) / wheel_radius;
  return std::abs(left) <= max_wheel_angular_speed + 1.0e-12 &&
         std::abs(right) <= max_wheel_angular_speed + 1.0e-12;
}

inline std::vector<double> sampleCurvatures(
  double minimum, double maximum, double step)
{
  std::vector<double> values;
  if (!(minimum <= maximum) || step <= 0.0 ||
    !std::isfinite(minimum) || !std::isfinite(maximum))
  {
    return values;
  }
  const std::size_t count = static_cast<std::size_t>(
    std::floor((maximum - minimum) / step));
  values.reserve(count + 3);
  values.push_back(minimum);
  for (std::size_t n = 1; n <= count; ++n) {
    const double value = minimum + static_cast<double>(n) * step;
    if (value < maximum - 1.0e-10) {
      values.push_back(value);
    }
  }
  if (minimum < 0.0 && maximum > 0.0) {
    values.push_back(0.0);
  }
  if (values.empty() || std::abs(values.back() - maximum) > 1.0e-10) {
    values.push_back(maximum);
  }
  std::sort(values.begin(), values.end());
  values.erase(
    std::unique(values.begin(), values.end(), [](double a, double b) {
      return std::abs(a - b) < 1.0e-10;
    }),
    values.end());
  return values;
}

// Complete configured geometric family, independent of the instantaneous
// dynamic window. Samples are paired exactly around zero and always include
// zero and both configured endpoints.
inline std::vector<double> sampleSymmetricCurvatures(
  double max_curvature, double step)
{
  std::vector<double> values;
  if (!std::isfinite(max_curvature) || max_curvature < 0.0 ||
    !std::isfinite(step) || step <= 0.0)
  {
    return values;
  }
  values.push_back(0.0);
  for (double magnitude = step;
    magnitude < max_curvature - 1.0e-10; magnitude += step)
  {
    values.push_back(-magnitude);
    values.push_back(magnitude);
  }
  if (max_curvature > 1.0e-12) {
    values.push_back(-max_curvature);
    values.push_back(max_curvature);
  }
  std::sort(values.begin(), values.end());
  values.erase(
    std::unique(values.begin(), values.end(), [](double a, double b) {
      return std::abs(a - b) < 1.0e-10;
    }),
    values.end());
  return values;
}

inline std::vector<std::vector<std::size_t>> clusterContiguous(
  const std::vector<bool> & admissible)
{
  std::vector<std::vector<std::size_t>> clusters;
  for (std::size_t i = 0; i < admissible.size(); ++i) {
    if (!admissible[i]) {
      continue;
    }
    if (i == 0 || !admissible[i - 1]) {
      clusters.emplace_back();
    }
    clusters.back().push_back(i);
  }
  return clusters;
}

inline double speedAtTime(
  double test_speed, double reaction_time,
  double braking_deceleration, double time)
{
  if (time <= reaction_time) {
    return test_speed;
  }
  return std::max(
    0.0, test_speed - braking_deceleration * (time - reaction_time));
}

inline double distanceAtTime(
  double test_speed, double reaction_time,
  double braking_deceleration, double time)
{
  if (time <= 0.0) {
    return 0.0;
  }
  if (time <= reaction_time) {
    return test_speed * time;
  }
  const double braking_time = std::min(
    time - reaction_time, test_speed / braking_deceleration);
  return test_speed * reaction_time +
         test_speed * braking_time -
         0.5 * braking_deceleration * braking_time * braking_time;
}

inline double stopTime(
  double test_speed, double reaction_time, double braking_deceleration)
{
  return reaction_time + test_speed / braking_deceleration;
}

inline double squaredDistancePointSegment(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double denominator = dx * dx + dy * dy;
  const double t = denominator > 0.0 ?
    std::clamp(((px - ax) * dx + (py - ay) * dy) / denominator, 0.0, 1.0) :
    0.0;
  const double ex = px - (ax + t * dx);
  const double ey = py - (ay + t * dy);
  return ex * ex + ey * ey;
}

inline bool pointInConvexPolygon(
  double px, double py, const std::vector<std::pair<double, double>> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }
  int sign = 0;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double cross =
      (b.first - a.first) * (py - a.second) -
      (b.second - a.second) * (px - a.first);
    if (std::abs(cross) < 1.0e-12) {
      continue;
    }
    const int current = cross > 0.0 ? 1 : -1;
    if (sign != 0 && current != sign) {
      return false;
    }
    sign = current;
  }
  return true;
}

inline double distancePointPolygon(
  double px, double py, const std::vector<std::pair<double, double>> & polygon)
{
  if (pointInConvexPolygon(px, py, polygon)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    best = std::min(
      best, squaredDistancePointSegment(
        px, py, a.first, a.second, b.first, b.second));
  }
  return std::sqrt(best);
}

}  // namespace crowd_aware_simulation
