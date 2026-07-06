#include <gtest/gtest.h>

#include <vector>

#include "robot_planner/mpc_path_tracker.hpp"

namespace
{

std::vector<robot_planner::Point2D> straightPath()
{
  std::vector<robot_planner::Point2D> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back({0.1 * i, 0.0});
  }
  return path;
}

std::vector<robot_planner::Point2D> leftTurnPath()
{
  std::vector<robot_planner::Point2D> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back({0.1 * i, 0.0});
  }
  for (int i = 1; i <= 12; ++i) {
    path.push_back({1.0, 0.1 * i});
  }
  return path;
}

robot_planner::MpcDwbParams testParams()
{
  robot_planner::MpcDwbParams params;
  params.control_frequency = 10.0;
  params.acc_lim_x = 2.0;
  params.acc_lim_theta = 2.0;
  params.max_vx = 0.4;
  params.max_wz = 0.8;
  params.vx_samples = 5;
  params.wz_samples = 7;
  return params;
}

}  // namespace

TEST(MpcPathTracker, EmptyPathOutputsZero)
{
  robot_planner::MpcPathTracker tracker;
  tracker.configure(testParams());

  const auto result = tracker.computeCommand({0.0, 0.0, 0.0}, {}, {}, 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.state, "EMPTY_PATH");
  EXPECT_DOUBLE_EQ(result.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.cmd.wz, 0.0);
}

TEST(MpcPathTracker, TurnPathKeepsForwardVelocity)
{
  robot_planner::MpcPathTracker tracker;
  auto params = testParams();
  params.path_weight = 8.0;
  params.heading_weight = 1.2;
  params.progress_weight = 0.8;
  tracker.configure(params);

  const auto result = tracker.computeCommand(
    {0.95, 0.0, 0.0}, {0.0, 0.0, 0.0}, leftTurnPath(), 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.state, "TRACK_PATH");
  EXPECT_GT(result.cmd.vx, 0.05);
  EXPECT_GT(std::abs(result.cmd.wz), 0.05);
}

TEST(MpcPathTracker, StraightPathOutputsForwardVelocity)
{
  robot_planner::MpcPathTracker tracker;
  tracker.configure(testParams());

  const auto result = tracker.computeCommand(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, straightPath(), 1.0);

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.state, "TRACK_PATH");
  EXPECT_GT(result.cmd.vx, 0.0);
  EXPECT_NEAR(result.cmd.vy, 0.0, 1e-9);
}

TEST(MpcPathTracker, GoalReachedOutputsZero)
{
  robot_planner::MpcPathTracker tracker;
  auto params = testParams();
  params.xy_goal_tolerance = 0.2;
  tracker.configure(params);

  const auto result = tracker.computeCommand(
    {2.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, straightPath(), 1.0);

  EXPECT_EQ(result.state, "GOAL_REACHED");
  EXPECT_DOUBLE_EQ(result.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.cmd.wz, 0.0);
}

TEST(MpcPathTracker, HardStopOutputsZero)
{
  robot_planner::MpcPathTracker tracker;
  auto params = testParams();
  params.hard_stop_distance = 0.2;
  tracker.configure(params);

  const auto result = tracker.computeCommand(
    {0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, straightPath(), 0.1);

  EXPECT_EQ(result.state, "BLOCKED_STOP");
  EXPECT_DOUBLE_EQ(result.cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(result.cmd.wz, 0.0);
}
