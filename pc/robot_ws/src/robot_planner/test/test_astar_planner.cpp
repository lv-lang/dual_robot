#include <gtest/gtest.h>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "robot_planner/astar_planner.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeMap(int width, int height, double resolution)
{
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(static_cast<std::size_t>(width * height), 0);
  return map;
}

}  // namespace

TEST(AStarPlanner, EmptyMapCanPlan)
{
  robot_planner::AStarParams params;
  params.robot_radius = 0.0;
  params.inflation_radius = 0.0;
  params.densify_spacing = 0.1;

  robot_planner::AStarPlanner planner;
  planner.configure(params);
  planner.setMap(makeMap(30, 30, 0.1));

  const auto result = planner.plan({0.15, 0.15}, {2.5, 2.5});

  EXPECT_TRUE(result.success);
  EXPECT_GT(result.path.size(), 2u);
  EXPECT_EQ(result.state, "PLANNED");
}

TEST(AStarPlanner, ObstacleInflationBlocksOccupiedCells)
{
  robot_planner::AStarParams params;
  params.robot_radius = 0.10;
  params.inflation_radius = 0.15;
  params.occupied_threshold = 65;

  auto map = makeMap(20, 20, 0.1);
  map.data[10 * 20 + 10] = 100;

  robot_planner::AStarPlanner planner;
  planner.configure(params);
  planner.setMap(map);

  EXPECT_FALSE(planner.isFreeWorld(1.05, 1.05));
}

TEST(AStarPlanner, PathDoesNotUseOccupiedWall)
{
  robot_planner::AStarParams params;
  params.robot_radius = 0.0;
  params.inflation_radius = 0.0;
  params.densify_spacing = 0.05;

  auto map = makeMap(30, 20, 0.1);
  for (int y = 0; y < 20; ++y) {
    if (y == 10) {
      continue;
    }
    map.data[static_cast<std::size_t>(y * 30 + 14)] = 100;
  }

  robot_planner::AStarPlanner planner;
  planner.configure(params);
  planner.setMap(map);
  const auto result = planner.plan({0.2, 1.0}, {2.6, 1.0});

  ASSERT_TRUE(result.success);
  for (const auto & point : result.path) {
    EXPECT_TRUE(planner.isFreeWorld(point.x, point.y));
  }
}
