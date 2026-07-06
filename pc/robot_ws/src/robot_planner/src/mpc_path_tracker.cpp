#include "robot_planner/mpc_path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot_planner
{

void MpcPathTracker::configure(const MpcDwbParams & params)
{
  params_ = params;
}

void MpcPathTracker::reset()
{
  closest_index_ = 0;
  last_cmd_ = Velocity2D{};
}

PlannerResult MpcPathTracker::computeCommand(
  const Pose2D & pose,
  const Velocity2D & current_velocity,
  const std::vector<Point2D> & global_path,
  double front_clearance)
{
  PlannerResult result;
  result.state = "EMPTY_PATH";
  if (global_path.empty()) {
    last_cmd_ = Velocity2D{};
    result.valid = true;
    return result;
  }

  if (std::isfinite(front_clearance) && front_clearance < params_.hard_stop_distance) {
    last_cmd_ = Velocity2D{};
    result.state = "BLOCKED_STOP";
    result.valid = true;
    return result;
  }

  const Point2D & goal = global_path.back();
  const double dist_to_goal = distance2D(pose.x, pose.y, goal.x, goal.y);
  if (dist_to_goal <= params_.xy_goal_tolerance) {
    last_cmd_ = Velocity2D{};
    result.state = "GOAL_REACHED";
    result.valid = true;
    return result;
  }

  const std::size_t closest = findClosestIndex(pose, global_path);
  const std::size_t heading_index = findLookaheadIndex(
    global_path, closest, params_.lookahead_distance);
  const double heading_error = std::abs(angleDiff(
      pose.yaw, pathHeading(global_path, heading_index)));
  const bool allow_rotate_in_place = heading_error > params_.rotate_in_place_heading_error;
  PlannerResult best;
  best.score = std::numeric_limits<double>::infinity();
  best.state = "TRACK_PATH";

  const double period = 1.0 / std::max(1.0, params_.control_frequency);
  const double requested_vx_floor = allow_rotate_in_place ?
    params_.min_vx : std::max(params_.min_vx, params_.min_tracking_vx);
  const double vx_high = std::min(params_.max_vx, current_velocity.vx + params_.acc_lim_x * period);
  const double vx_low = std::min(
    vx_high,
    std::max(requested_vx_floor, current_velocity.vx - params_.acc_lim_x * period));
  const double wz_low = std::max(-params_.max_wz, current_velocity.wz - params_.acc_lim_theta * period);
  const double wz_high = std::min(params_.max_wz, current_velocity.wz + params_.acc_lim_theta * period);

  const int vx_samples = std::max(1, params_.vx_samples);
  const int wz_samples = std::max(1, params_.wz_samples);
  for (int i = 0; i < vx_samples; ++i) {
    const double vx = vx_samples == 1 ? vx_high :
      vx_low + (vx_high - vx_low) * static_cast<double>(i) / static_cast<double>(vx_samples - 1);
    for (int j = 0; j < wz_samples; ++j) {
      const double wz = wz_samples == 1 ? 0.0 :
        wz_low + (wz_high - wz_low) * static_cast<double>(j) / static_cast<double>(wz_samples - 1);
      PlannerResult candidate = evaluateCandidate(vx, wz, pose, current_velocity, global_path, closest);
      if (candidate.valid && candidate.score < best.score) {
        best = candidate;
      }
    }
  }

  if (!best.valid) {
    last_cmd_ = Velocity2D{};
    result.state = "NO_VALID_TRAJECTORY";
    result.valid = true;
    return result;
  }

  best.cmd = smoothCommand(best.cmd, current_velocity);
  best.cmd.vy = 0.0;
  last_cmd_ = best.cmd;
  return best;
}

std::size_t MpcPathTracker::findLookaheadIndex(
  const std::vector<Point2D> & path,
  std::size_t start_index,
  double lookahead_distance) const
{
  if (path.empty()) {
    return 0;
  }
  std::size_t index = std::min(start_index, path.size() - 1);
  double accumulated = 0.0;
  while (index + 1 < path.size() && accumulated < lookahead_distance) {
    accumulated += distance2D(path[index].x, path[index].y, path[index + 1].x, path[index + 1].y);
    ++index;
  }
  return index;
}

std::size_t MpcPathTracker::findClosestIndex(const Pose2D & pose, const std::vector<Point2D> & path)
{
  if (path.empty()) {
    closest_index_ = 0;
    return 0;
  }
  closest_index_ = std::min(closest_index_, path.size() - 1);
  const std::size_t start = closest_index_;
  const std::size_t end = std::min(
    path.size() - 1,
    closest_index_ + static_cast<std::size_t>(std::max(1, params_.closest_search_window)));
  std::size_t best = start;
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = start; i <= end; ++i) {
    const double dist = distance2D(pose.x, pose.y, path[i].x, path[i].y);
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }

  if (best_dist > params_.reacquire_distance) {
    best = findNearestIndex(pose.x, pose.y, path, 0);
  }
  closest_index_ = std::max(closest_index_, best);
  return closest_index_;
}

std::size_t MpcPathTracker::findNearestIndex(
  double x,
  double y,
  const std::vector<Point2D> & path,
  std::size_t start_index) const
{
  std::size_t best = std::min(start_index, path.size() - 1);
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = best; i < path.size(); ++i) {
    const double dist = distance2D(x, y, path[i].x, path[i].y);
    if (dist < best_dist) {
      best_dist = dist;
      best = i;
    }
  }
  return best;
}

double MpcPathTracker::pathHeading(const std::vector<Point2D> & path, std::size_t index) const
{
  if (path.size() < 2) {
    return 0.0;
  }
  const std::size_t i0 = std::min(index, path.size() - 2);
  const std::size_t i1 = std::min(path.size() - 1, i0 + 1);
  return std::atan2(path[i1].y - path[i0].y, path[i1].x - path[i0].x);
}

PlannerResult MpcPathTracker::evaluateCandidate(
  double vx,
  double wz,
  const Pose2D & pose,
  const Velocity2D & current_velocity,
  const std::vector<Point2D> & global_path,
  std::size_t closest_index) const
{
  PlannerResult result;
  result.valid = true;
  result.state = "TRACK_PATH";
  result.cmd = Velocity2D{vx, 0.0, wz};
  result.score = 0.0;

  Pose2D sim = pose;
  std::size_t last_nearest = closest_index;
  double path_cost = 0.0;
  double heading_cost = 0.0;

  for (int step = 0; step < std::max(1, params_.horizon_steps); ++step) {
    sim.x += vx * std::cos(sim.yaw) * params_.dt;
    sim.y += vx * std::sin(sim.yaw) * params_.dt;
    sim.yaw = normalizeAngle(sim.yaw + wz * params_.dt);
    result.trajectory.push_back(sim);

    const std::size_t nearest = findNearestIndex(sim.x, sim.y, global_path, last_nearest);
    last_nearest = std::max(last_nearest, nearest);
    const std::size_t heading_index = findLookaheadIndex(
      global_path, nearest, params_.lookahead_distance);
    path_cost += distance2D(sim.x, sim.y, global_path[nearest].x, global_path[nearest].y);
    heading_cost += std::abs(angleDiff(sim.yaw, pathHeading(global_path, heading_index)));
  }

  const Point2D & goal = global_path.back();
  const double goal_cost = distance2D(sim.x, sim.y, goal.x, goal.y);
  const double progress = static_cast<double>(last_nearest) - static_cast<double>(closest_index);
  const double velocity_cost = std::abs(vx - params_.target_speed);
  const double smoothness_cost = std::abs(vx - current_velocity.vx) +
    0.5 * std::abs(wz - current_velocity.wz) +
    std::abs(vx - last_cmd_.vx) +
    0.5 * std::abs(wz - last_cmd_.wz);
  const double angular_cost = std::abs(wz);

  result.score =
    params_.path_weight * path_cost +
    params_.goal_weight * goal_cost +
    params_.heading_weight * heading_cost +
    params_.velocity_weight * velocity_cost +
    params_.smoothness_weight * smoothness_cost +
    params_.angular_weight * angular_cost -
    params_.progress_weight * progress;
  return result;
}

Velocity2D MpcPathTracker::smoothCommand(const Velocity2D & target, const Velocity2D & current) const
{
  const double period = 1.0 / std::max(1.0, params_.control_frequency);
  Velocity2D cmd = target;
  cmd.vx = std::clamp(cmd.vx, current.vx - params_.acc_lim_x * period,
      current.vx + params_.acc_lim_x * period);
  cmd.wz = std::clamp(cmd.wz, current.wz - params_.acc_lim_theta * period,
      current.wz + params_.acc_lim_theta * period);
  cmd.vx = std::clamp(cmd.vx, params_.min_vx, params_.max_vx);
  cmd.wz = std::clamp(cmd.wz, -params_.max_wz, params_.max_wz);
  if (std::abs(cmd.vx) < 0.01) {
    cmd.vx = 0.0;
  }
  if (std::abs(cmd.wz) < 0.02) {
    cmd.wz = 0.0;
  }
  return cmd;
}

}  // namespace robot_planner
