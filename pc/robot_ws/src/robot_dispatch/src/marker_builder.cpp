#include "robot_dispatch/marker_builder.hpp"

#include <cmath>

#include "geometry_msgs/msg/point.hpp"

namespace robot_dispatch
{

namespace
{

std::string rvizFrameId(const TaskPointConfig & config)
{
  std::string frame_id = config.frame_id;
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id.empty() ? "map" : frame_id;
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

geometry_msgs::msg::Pose poseFrom2D(const Pose2D & pose)
{
  geometry_msgs::msg::Pose out;
  out.position.x = pose.x;
  out.position.y = pose.y;
  out.position.z = 0.05;
  const double half_yaw = pose.yaw * 0.5;
  out.orientation.z = std::sin(half_yaw);
  out.orientation.w = std::cos(half_yaw);
  return out;
}

visualization_msgs::msg::Marker makeBaseMarker(
  const TaskPointConfig & config,
  const TaskPoint & point,
  int id,
  const std::string & ns)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = rvizFrameId(config);
  marker.ns = ns;
  marker.id = id;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseFrom2D(point.pose);
  marker.scale.x = 0.35;
  marker.scale.y = 0.35;
  marker.scale.z = 0.12;
  marker.color = colorForPointKind(point.kind);
  return marker;
}

const char * namespaceForPointKind(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return "waiting_area_points";
    case PointKind::PICKUP:
      return "pickup_points";
    case PointKind::DELIVERY:
      return "delivery_points";
    case PointKind::INSPECTION:
      return "inspection_points";
  }
  return "business_points";
}

int markerTypeForPointKind(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return visualization_msgs::msg::Marker::CUBE;
    case PointKind::PICKUP:
      return visualization_msgs::msg::Marker::ARROW;
    case PointKind::DELIVERY:
      return visualization_msgs::msg::Marker::CYLINDER;
    case PointKind::INSPECTION:
      return visualization_msgs::msg::Marker::SPHERE;
  }
  return visualization_msgs::msg::Marker::SPHERE;
}

void configureBaseScale(visualization_msgs::msg::Marker * marker, PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      marker->scale.x = 0.45;
      marker->scale.y = 0.45;
      marker->scale.z = 0.12;
      break;
    case PointKind::PICKUP:
      marker->scale.x = 0.55;
      marker->scale.y = 0.12;
      marker->scale.z = 0.20;
      break;
    case PointKind::DELIVERY:
      marker->scale.x = 0.42;
      marker->scale.y = 0.42;
      marker->scale.z = 0.18;
      break;
    case PointKind::INSPECTION:
      marker->scale.x = 0.36;
      marker->scale.y = 0.36;
      marker->scale.z = 0.22;
      break;
  }
}

const char * labelPrefixForPointKind(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return "WAIT";
    case PointKind::PICKUP:
      return "PICKUP";
    case PointKind::DELIVERY:
      return "DROP";
    case PointKind::INSPECTION:
      return "INSP";
  }
  return "POINT";
}

visualization_msgs::msg::Marker makeTextMarker(
  const TaskPointConfig & config,
  const TaskPoint & point,
  int id,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & text_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = rvizFrameId(config);
  marker.ns = "task_state";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = poseFrom2D(point.pose);
  marker.pose.position.z = 0.45;
  marker.scale.z = 0.25;
  marker.color = text_color;
  marker.text = text;
  return marker;
}

visualization_msgs::msg::Marker makeOverlayMarker(
  const TaskPointConfig & config,
  const TaskPoint & point,
  int id,
  const std::string & ns,
  const std_msgs::msg::ColorRGBA & overlay_color,
  double scale)
{
  auto marker = makeBaseMarker(config, point, id, ns);
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.pose.position.z = 0.08;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = 0.08;
  marker.color = overlay_color;
  return marker;
}

}  // namespace

std_msgs::msg::ColorRGBA colorForPointKind(PointKind kind)
{
  switch (kind) {
    case PointKind::WAITING_AREA:
      return color(0.15F, 0.55F, 1.0F);
    case PointKind::PICKUP:
      return color(0.1F, 0.85F, 0.35F);
    case PointKind::DELIVERY:
      return color(1.0F, 0.65F, 0.05F);
    case PointKind::INSPECTION:
      return color(0.75F, 0.35F, 1.0F);
  }
  return idlePointColor();
}

std_msgs::msg::ColorRGBA idlePointColor()
{
  return color(0.75F, 0.75F, 0.75F);
}

std_msgs::msg::ColorRGBA lockedPointColor()
{
  return color(1.0F, 0.2F, 0.15F);
}

std_msgs::msg::ColorRGBA activeTargetColor()
{
  return color(0.0F, 1.0F, 1.0F);
}

std_msgs::msg::ColorRGBA abnormalPointColor()
{
  return color(1.0F, 0.0F, 0.0F);
}

std_msgs::msg::ColorRGBA recheckTargetColor()
{
  return color(1.0F, 1.0F, 0.0F);
}

visualization_msgs::msg::MarkerArray buildTaskPointMarkers(
  const TaskPointConfig & config,
  const MarkerSceneState & state)
{
  visualization_msgs::msg::MarkerArray array;
  int id = 1;
  for (const auto & [point_id, point] : config.points) {
    auto marker = makeBaseMarker(config, point, id++, namespaceForPointKind(point.kind));
    marker.type = markerTypeForPointKind(point.kind);
    configureBaseScale(&marker, point.kind);

    array.markers.push_back(marker);

    const std::string text =
      std::string(point.temporary ? "TMP " : "") +
      labelPrefixForPointKind(point.kind) + " " + point_id + " " + point.label;
    array.markers.push_back(makeTextMarker(config, point, id++, text, marker.color));

    if (state.locked_point_ids.count(point_id) != 0) {
      array.markers.push_back(
        makeOverlayMarker(config, point, id++, "resource_locks", lockedPointColor(), 0.50));
    }
    if (state.active_target_ids.count(point_id) != 0) {
      array.markers.push_back(
        makeOverlayMarker(config, point, id++, "active_goal", activeTargetColor(), 0.62));
    }
    if (state.abnormal_point_ids.count(point_id) != 0) {
      array.markers.push_back(
        makeOverlayMarker(config, point, id++, "abnormal", abnormalPointColor(), 0.74));
    }
    if (state.recheck_target_ids.count(point_id) != 0) {
      array.markers.push_back(
        makeOverlayMarker(config, point, id++, "recheck", recheckTargetColor(), 0.86));
    }
  }
  return array;
}

}  // namespace robot_dispatch
