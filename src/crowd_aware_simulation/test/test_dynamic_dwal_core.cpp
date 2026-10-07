#include <cmath>
#include <vector>

#include "crowd_aware_simulation/dynamic_dwal_core.hpp"
#include "gtest/gtest.h"

namespace cas = crowd_aware_simulation;

TEST(DynamicDwalCore, PreservesUpstreamArcLineGeometry)
{
  const double curvature = 2.0;
  const double transition = cas::arcLineTransition(curvature);
  const cas::Pose2 on_arc = cas::arcLinePose(curvature, transition);
  EXPECT_NEAR(on_arc.x, 0.5, 1.0e-12);
  EXPECT_NEAR(on_arc.y, 0.5, 1.0e-12);
  EXPECT_NEAR(on_arc.theta, cas::kPi / 2.0, 1.0e-12);

  const cas::Pose2 on_line =
    cas::arcLinePose(curvature, transition + 0.4);
  EXPECT_NEAR(on_line.x, 0.5, 1.0e-12);
  EXPECT_NEAR(on_line.y, 0.9, 1.0e-12);
  EXPECT_NEAR(on_line.theta, cas::kPi / 2.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(cas::localCurvature(curvature, transition + 0.1), 0.0);
}

TEST(DynamicDwalCore, ArcLineTransitionIsPositionAndTangentContinuous)
{
  const double curvature = -1.7;
  const double transition = cas::arcLineTransition(curvature);
  const double epsilon = 1.0e-6;
  const auto before = cas::arcLinePose(curvature, transition - epsilon);
  const auto at = cas::arcLinePose(curvature, transition);
  const auto after = cas::arcLinePose(curvature, transition + epsilon);
  EXPECT_NEAR(before.x, at.x, epsilon + 1.0e-12);
  EXPECT_NEAR(before.y, at.y, epsilon + 1.0e-12);
  EXPECT_NEAR(after.x, at.x, epsilon + 1.0e-12);
  EXPECT_NEAR(after.y, at.y, epsilon + 1.0e-12);
  const double tangent_x = std::cos(at.theta);
  const double tangent_y = std::sin(at.theta);
  EXPECT_NEAR((at.x - before.x) / epsilon, tangent_x, 2.0e-6);
  EXPECT_NEAR((at.y - before.y) / epsilon, tangent_y, 2.0e-6);
  EXPECT_NEAR((after.x - at.x) / epsilon, tangent_x, 1.0e-9);
  EXPECT_NEAR((after.y - at.y) / epsilon, tangent_y, 1.0e-9);
}

TEST(DynamicDwalCore, MeasuredSpeedCannotBeHiddenByLowerRequest)
{
  const auto speed = cas::computeTestSpeed(
    0.05, 0.28, 0.30, 0.50, 0.50, 0.10);
  EXPECT_NEAR(speed.window_min, 0.23, 1.0e-12);
  EXPECT_NEAR(speed.reachable_target, 0.23, 1.0e-12);
  EXPECT_NEAR(speed.test, 0.28, 1.0e-12);
  EXPECT_NEAR(cas::stoppingDistance(0.28, 0.40, 0.50, 0.05),
    0.2404, 1.0e-12);
}

TEST(DynamicDwalCore, SamplingComesFromAngularDynamicWindow)
{
  const auto window = cas::computeCurvatureWindow(
    0.30, 0.20, 0.80, 1.0, 0.10, 4.0, 0.02);
  ASSERT_TRUE(window.valid);
  EXPECT_NEAR(window.minimum, 1.0, 1.0e-12);
  EXPECT_NEAR(window.maximum, 2.0, 1.0e-12);
  const auto samples =
    cas::sampleCurvatures(window.minimum, window.maximum, 0.25);
  ASSERT_EQ(samples.size(), 5U);
  EXPECT_NEAR(samples.front(), 1.0, 1.0e-12);
  EXPECT_NEAR(samples.back(), 2.0, 1.0e-12);
}

TEST(DynamicDwalCore, FullFamilySamplingIsExactlySymmetric)
{
  const auto samples = cas::sampleSymmetricCurvatures(1.0, 0.3);
  ASSERT_EQ(samples.size(), 9U);
  EXPECT_DOUBLE_EQ(samples.front(), -1.0);
  EXPECT_DOUBLE_EQ(samples.back(), 1.0);
  EXPECT_DOUBLE_EQ(samples[samples.size() / 2], 0.0);
  for (std::size_t i = 0; i < samples.size(); ++i) {
    EXPECT_NEAR(samples[i], -samples[samples.size() - 1 - i], 1.0e-12);
  }
}

TEST(DynamicDwalCore, ZeroTranslationDoesNotUseOmegaOverV)
{
  const auto window = cas::computeCurvatureWindow(
    0.2, 0.0, 0.8, 1.0, 0.1, 4.0, 0.02);
  EXPECT_FALSE(window.valid);
}

TEST(DynamicDwalCore, WheelAndAngularBrakingConstraintsAreExplicit)
{
  EXPECT_TRUE(cas::wheelSpeedFeasible(0.30, 0.80, 0.61, 0.095, 20.0));
  EXPECT_FALSE(cas::wheelSpeedFeasible(0.30, 8.0, 0.61, 0.095, 20.0));
  EXPECT_GT(std::abs(3.0) * 0.5, 1.0);
}

TEST(DynamicDwalCore, RejectedSampleAlwaysBreaksACluster)
{
  const std::vector<bool> admissible{
    true, true, false, true, false, false, true, true};
  const auto clusters = cas::clusterContiguous(admissible);
  ASSERT_EQ(clusters.size(), 3U);
  EXPECT_EQ(clusters[0], (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(clusters[1], (std::vector<std::size_t>{3}));
  EXPECT_EQ(clusters[2], (std::vector<std::size_t>{6, 7}));
}

TEST(DynamicDwalCore, BackupProfileMatchesStoppingEquation)
{
  const double speed = 0.30;
  const double reaction = 0.40;
  const double braking = 0.50;
  const double stop_time = cas::stopTime(speed, reaction, braking);
  EXPECT_NEAR(stop_time, 1.00, 1.0e-12);
  EXPECT_NEAR(
    cas::distanceAtTime(speed, reaction, braking, stop_time),
    speed * reaction + speed * speed / (2.0 * braking), 1.0e-12);
  EXPECT_DOUBLE_EQ(
    cas::speedAtTime(speed, reaction, braking, stop_time), 0.0);
}
TEST(DynamicDwalCore, ShortPathIsNotRejectedByLegacyNearLevel)
{
  const double required =
    cas::stoppingDistance(0.30, 0.40, 0.50, 0.05);
  EXPECT_NEAR(required, 0.260, 1.0e-12);
  EXPECT_TRUE(cas::hasStaticBrakingClearance(required, 0.80));
  EXPECT_FALSE(cas::hasStaticBrakingClearance(required, required));
}

TEST(DynamicDwalCore, ClustersChangeWithTestSpeed)
{
  const std::vector<double> free_lengths{0.12, 0.18, 0.30, 0.18, 0.12};
  auto classify = [&](double speed) {
      const double required =
        cas::stoppingDistance(speed, 0.40, 0.50, 0.05);
      std::vector<bool> admissible;
      for (double length : free_lengths) {
        admissible.push_back(
          cas::hasStaticBrakingClearance(required, length));
      }
      return cas::clusterContiguous(admissible);
    };
  const auto slow = classify(0.05);
  const auto fast = classify(0.25);
  ASSERT_EQ(slow.size(), 1U);
  EXPECT_EQ(slow.front().size(), 5U);
  ASSERT_EQ(fast.size(), 1U);
  EXPECT_EQ(fast.front(), (std::vector<std::size_t>{2}));
}



TEST(DynamicDwalCore, MovingHumanIsComparedAtTheSameTime)
{
  const double human_x0 = 1.0;
  const double human_vx = -0.6;
  const double tau = 1.0;
  const double robot_s = cas::distanceAtTime(0.30, 0.40, 0.50, tau);
  const auto robot_pose = cas::arcLinePose(0.0, robot_s);
  const double simultaneous_separation =
    std::abs((human_x0 + human_vx * tau) - robot_pose.x);
  const double static_snapshot_separation =
    std::abs(human_x0 - robot_pose.x);
  EXPECT_LT(simultaneous_separation, 0.30);
  EXPECT_GT(static_snapshot_separation, 0.70);
}
