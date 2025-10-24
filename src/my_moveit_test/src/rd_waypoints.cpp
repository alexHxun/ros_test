// rd_waypoints.cpp
#include "rd_waypoints.hpp"
#include "auto_planner.hpp"   // 必须包含 AutoPlannerCpp 的类声明
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_processing/iterative_time_parameterization.h>
#include <cmath>
#include <vector>

void AutoPlannerCpp::reduceTrajectoryPointsRDP(moveit::planning_interface::MoveGroupInterface::Plan &plan, size_t max_points)
{
  if (!move_group_) {
    RCLCPP_WARN(this->get_logger(), "move_group_ not initialized; cannot reduce trajectory.");
    return;
  }
  if (max_points < 2) {
    RCLCPP_WARN(this->get_logger(), "max_points must be >= 2");
    return;
  }
  if (plan.trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Trajectory empty; nothing to reduce.");
    return;
  }

  moveit::core::RobotModelConstPtr robot_model = move_group_->getCurrentState()->getRobotModel();
  const std::string group_name = move_group_->getName();
  robot_trajectory::RobotTrajectory rt(robot_model, group_name);
  rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);
  const size_t wp_count = rt.getWayPointCount();
  if (wp_count <= max_points) {
    RCLCPP_INFO(this->get_logger(), "Waypoints (%zu) already <= %zu", wp_count, max_points);
    return;
  }

  // collect EE positions
  std::vector<Eigen::Vector3d> ee_positions; ee_positions.reserve(wp_count);
  const std::string ee_link = move_group_->getEndEffectorLink();
  for (size_t i = 0; i < wp_count; ++i) {
    const moveit::core::RobotState &rs = rt.getWayPoint(i);
    Eigen::Isometry3d tf = rs.getGlobalLinkTransform(ee_link);
    ee_positions.push_back(tf.translation());
  }

  // find eps by exponential growth then binary refine using rd_utils::rdp_indices
  double eps_low = 0.0;
  double eps_high = 1e-6;
  std::vector<int> chosen;
  for (int step = 0; step < 60; ++step) {
    chosen = rd_utils::rdp_indices(ee_positions, eps_high);
    if (chosen.size() <= max_points) break;
    eps_high *= 2.0;
    if (eps_high > 1.0) break;
  }

  // fallback: uniform downsample
  if (chosen.size() > max_points) {
    RCLCPP_WARN(this->get_logger(), "RDP could not reach target points; using uniform downsample fallback.");
    robot_trajectory::RobotTrajectory rt2(robot_model, group_name);
    for (size_t i = 0; i < max_points; ++i) {
      size_t idx = static_cast<size_t>(std::round(i * double(wp_count - 1) / double(max_points - 1)));
      rt2.addSuffixWayPoint(rt.getWayPoint(idx), 0.0);
    }
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt2)) {
      RCLCPP_WARN(this->get_logger(), "Time parameterization failed on fallback reduced trajectory; keeping original.");
      return;
    }
    rt2.getRobotTrajectoryMsg(plan.trajectory_);
    RCLCPP_INFO(this->get_logger(), "Reduced trajectory (fallback) from %zu -> %zu points", wp_count, max_points);
    return;
  }

  // binary refine
  for (int iter = 0; iter < 30; ++iter) {
    double mid = 0.5 * (eps_low + eps_high);
    if (mid <= eps_low) break;
    auto idx_mid = rd_utils::rdp_indices(ee_positions, mid);
    if (idx_mid.size() <= max_points) {
      eps_high = mid;
      chosen = idx_mid;
    } else {
      eps_low = mid;
    }
  }

  if (chosen.front() != 0) chosen.insert(chosen.begin(), 0);
  if (chosen.back() != static_cast<int>(wp_count) - 1) chosen.push_back(static_cast<int>(wp_count) - 1);

  // prune if still > max_points
  if (chosen.size() > max_points) {
    std::vector<int> final_idxs;
    for (size_t i = 0; i < max_points; ++i) {
      size_t idx = static_cast<size_t>(std::round(i * double(chosen.size() - 1) / double(max_points - 1)));
      final_idxs.push_back(chosen[idx]);
    }
    chosen.swap(final_idxs);
  }

  // construct reduced trajectory and re-time-parameterize
  robot_trajectory::RobotTrajectory rt2(robot_model, group_name);
  for (int idx : chosen) {
    if (idx < 0) continue;
    rt2.addSuffixWayPoint(rt.getWayPoint(static_cast<size_t>(idx)), 0.0);
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt2)) {
    RCLCPP_WARN(this->get_logger(), "Time parameterization failed on reduced trajectory; keeping original.");
    return;
  }
  rt2.getRobotTrajectoryMsg(plan.trajectory_);
  RCLCPP_INFO(this->get_logger(), "Reduced trajectory from %zu -> %zu points (RDP)", wp_count, rt2.getWayPointCount());
}
