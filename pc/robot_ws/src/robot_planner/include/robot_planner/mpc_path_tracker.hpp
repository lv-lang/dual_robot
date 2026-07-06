#ifndef ROBOT_PLANNER__MPC_PATH_TRACKER_HPP_
#define ROBOT_PLANNER__MPC_PATH_TRACKER_HPP_

#include <cstddef>
#include <vector>

#include "robot_planner/planner_types.hpp"

namespace robot_planner
{

struct MpcDwbParams
{
  double control_frequency{15.0};
  double xy_goal_tolerance{0.12};
  double lookahead_distance{0.65};
  double reacquire_distance{0.75};
  int closest_search_window{40};

  double min_vx{0.0};
  double min_tracking_vx{0.06};
  double max_vx{0.32};
  double max_wz{0.65};
  double rotate_in_place_heading_error{2.35};
  double target_speed{0.24};
  double acc_lim_x{0.50};
  double acc_lim_theta{0.80};

  int vx_samples{6};
  int wz_samples{9};
  int horizon_steps{14};
  double dt{0.10};

  double path_weight{5.0};
  double goal_weight{1.0};
  double heading_weight{0.55};
  double progress_weight{0.65};
  double velocity_weight{0.35};
  double smoothness_weight{0.90};
  double angular_weight{0.35};

  double hard_stop_distance{0.18};
};

class MpcPathTracker
{
public:
  void configure(const MpcDwbParams & params);
  void reset();
  PlannerResult computeCommand(
    const Pose2D & pose,
    const Velocity2D & current_velocity,
    const std::vector<Point2D> & global_path,
    double front_clearance);

private:
  std::size_t findClosestIndex(const Pose2D & pose, const std::vector<Point2D> & path);
  std::size_t findNearestIndex(
    double x,
    double y,
    const std::vector<Point2D> & path,
    std::size_t start_index) const;
  std::size_t findLookaheadIndex(
    const std::vector<Point2D> & path,
    std::size_t start_index,
    double lookahead_distance) const;
  double pathHeading(const std::vector<Point2D> & path, std::size_t index) const;
  PlannerResult evaluateCandidate(
    double vx,
    double wz,
    const Pose2D & pose,
    const Velocity2D & current_velocity,
    const std::vector<Point2D> & global_path,
    std::size_t closest_index) const;
  Velocity2D smoothCommand(const Velocity2D & target, const Velocity2D & current) const;

  MpcDwbParams params_;
  std::size_t closest_index_{0};
  Velocity2D last_cmd_;
};

}  // namespace robot_planner

#endif  // ROBOT_PLANNER__MPC_PATH_TRACKER_HPP_
