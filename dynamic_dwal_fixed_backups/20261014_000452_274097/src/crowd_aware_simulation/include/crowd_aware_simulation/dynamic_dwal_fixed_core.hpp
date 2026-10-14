#pragma once

#include "crowd_aware_simulation/dynamic_dwal_core.hpp"
#include <cstdint>
#include <functional>

namespace crowd_aware_simulation::fixed_dwal
{
using Polygon = std::vector<std::pair<double, double>>;
struct CurvePoint { double s; Pose2 pose; };
struct ClusterMember { std::uint32_t index; double curvature; bool admissible; };
struct Cluster { std::vector<std::size_t> members; double span; };
struct MotionPoint { double time; Pose2 pose; double linear; double angular; };

// The fixed DWAL level is a radius, NOT an arc length.
double lengthAtRadius(double curvature, double radius);
double angleAtRadius(double curvature, double radius);
std::vector<CurvePoint> completeCurve(double curvature, double radius, double step);
Pose2 toLocal(const Pose2 & world, const Pose2 & origin);
Pose2 toWorld(const Pose2 & local, const Pose2 & origin);
Polygon transformPolygon(const Polygon & polygon, const Pose2 & pose);
bool polygonIntersectsRectangle(
  const Polygon & polygon, double xmin, double ymin, double xmax, double ymax);
std::vector<Cluster> formClusters(
  const std::vector<ClusterMember> & sorted_members,
  std::uint32_t separation, double min_span, double radius);
SpeedState evaluateSpeed(
  double requested, double measured, double max_speed,
  double acceleration, double deceleration, double dt);
std::vector<MotionPoint> brakingMotion(
  double linear, double angular, double reaction_time,
  double linear_deceleration, double angular_deceleration, double dt);
std::pair<double, double> humanPosition(
  double x, double y, double vx, double vy, double track_age,
  double future_time, bool moving);
bool humanIntersectsMotion(
  const std::vector<MotionPoint> & motion, const Polygon & footprint,
  double hx, double hy, double vx, double vy, double track_age,
  bool moving, double radius, double uncertainty, double uncertainty_growth);
}  // namespace crowd_aware_simulation::fixed_dwal
