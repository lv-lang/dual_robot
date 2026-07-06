#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "robot_dispatch/marker_builder.hpp"
#include "robot_dispatch/task_points.hpp"

namespace
{

const std::string kTaskPointsPath =
  std::string(ROBOT_DISPATCH_SOURCE_DIR) + "/config/task_points.yaml";

const visualization_msgs::msg::Marker * findMarker(
  const visualization_msgs::msg::MarkerArray & markers,
  const std::string & ns)
{
  const auto it = std::find_if(markers.markers.begin(), markers.markers.end(), [&](const auto & marker) {
    return marker.ns == ns;
  });
  if (it == markers.markers.end()) {
    return nullptr;
  }
  return &(*it);
}

const visualization_msgs::msg::Marker * findLabelContaining(
  const visualization_msgs::msg::MarkerArray & markers,
  const std::string & text)
{
  const auto it = std::find_if(markers.markers.begin(), markers.markers.end(), [&](const auto & marker) {
    return marker.ns == "task_state" && marker.text.find(text) != std::string::npos;
  });
  if (it == markers.markers.end()) {
    return nullptr;
  }
  return &(*it);
}

void expectColorNear(
  const std_msgs::msg::ColorRGBA & actual,
  const std_msgs::msg::ColorRGBA & expected)
{
  EXPECT_NEAR(actual.r, expected.r, 1.0e-5);
  EXPECT_NEAR(actual.g, expected.g, 1.0e-5);
  EXPECT_NEAR(actual.b, expected.b, 1.0e-5);
  EXPECT_NEAR(actual.a, expected.a, 1.0e-5);
}

}  // namespace

TEST(TaskPoints, LoadsSharedMapSemanticPointConfig)
{
  const auto config = robot_dispatch::loadTaskPointsFromYaml(kTaskPointsPath);

  EXPECT_EQ(config.frame_id, "map");
  ASSERT_GE(config.points.size(), 7u);
  EXPECT_EQ(config.points.at("W1").kind, robot_dispatch::PointKind::WAITING_AREA);
  EXPECT_EQ(config.points.at("W2").kind, robot_dispatch::PointKind::WAITING_AREA);
  EXPECT_EQ(config.points.at("PICKUP_A").kind, robot_dispatch::PointKind::PICKUP);
  EXPECT_EQ(config.points.at("DELIVERY_C").kind, robot_dispatch::PointKind::DELIVERY);
  EXPECT_EQ(config.points.at("P1").kind, robot_dispatch::PointKind::INSPECTION);
  EXPECT_EQ(config.points.at("P2").kind, robot_dispatch::PointKind::INSPECTION);
  EXPECT_EQ(config.points.at("P3").kind, robot_dispatch::PointKind::INSPECTION);

  EXPECT_EQ(config.robots.at("robot1").waiting_area_id, "W1");
  EXPECT_EQ(config.robots.at("robot2").waiting_area_id, "W2");
  EXPECT_EQ(config.routes.at("delivery_demo").size(), 2u);
  EXPECT_EQ(config.routes.at("inspection_demo").size(), 3u);
}

TEST(MarkerBuilder, PublishesExpectedTopicAndBaselineMarkers)
{
  const auto config = robot_dispatch::loadTaskPointsFromYaml(kTaskPointsPath);
  const auto markers = robot_dispatch::buildTaskPointMarkers(config);

  EXPECT_STREQ(robot_dispatch::kMarkerTopic, "/robot_dispatch/markers");
  EXPECT_EQ(markers.markers.size(), config.points.size() * 2u);
  ASSERT_NE(findMarker(markers, "waiting_area_points"), nullptr);
  ASSERT_NE(findMarker(markers, "pickup_points"), nullptr);
  ASSERT_NE(findMarker(markers, "delivery_points"), nullptr);
  ASSERT_NE(findMarker(markers, "inspection_points"), nullptr);
  ASSERT_NE(findMarker(markers, "task_state"), nullptr);
  ASSERT_NE(findLabelContaining(markers, "PICKUP PICKUP_A A 取货点"), nullptr);
  ASSERT_NE(findLabelContaining(markers, "DROP DELIVERY_C C 配送点"), nullptr);
  ASSERT_NE(findLabelContaining(markers, "INSP P1 P1 巡检点"), nullptr);

  for (const auto & marker : markers.markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
  }
}

TEST(MarkerBuilder, OverlayStatesUseDeterministicNamespacesAndColors)
{
  const auto config = robot_dispatch::loadTaskPointsFromYaml(kTaskPointsPath);
  robot_dispatch::MarkerSceneState state;
  state.locked_point_ids.insert("P1");
  state.active_target_ids.insert("PICKUP_A");
  state.abnormal_point_ids.insert("P2");
  state.recheck_target_ids.insert("P3");

  const auto markers = robot_dispatch::buildTaskPointMarkers(config, state);

  ASSERT_NE(findMarker(markers, "pickup_points"), nullptr);
  ASSERT_NE(findMarker(markers, "inspection_points"), nullptr);

  const auto * lock_marker = findMarker(markers, "resource_locks");
  ASSERT_NE(lock_marker, nullptr);
  expectColorNear(lock_marker->color, robot_dispatch::lockedPointColor());

  const auto * active_marker = findMarker(markers, "active_goal");
  ASSERT_NE(active_marker, nullptr);
  expectColorNear(active_marker->color, robot_dispatch::activeTargetColor());

  const auto * abnormal_marker = findMarker(markers, "abnormal");
  ASSERT_NE(abnormal_marker, nullptr);
  expectColorNear(abnormal_marker->color, robot_dispatch::abnormalPointColor());

  const auto * recheck_marker = findMarker(markers, "recheck");
  ASSERT_NE(recheck_marker, nullptr);
  expectColorNear(recheck_marker->color, robot_dispatch::recheckTargetColor());
}

TEST(MarkerBuilder, NormalizesLegacySlashPrefixedMapFrameForRviz)
{
  auto config = robot_dispatch::loadTaskPointsFromYaml(kTaskPointsPath);
  config.frame_id = "/map";

  const auto markers = robot_dispatch::buildTaskPointMarkers(config);

  for (const auto & marker : markers.markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
  }
}
