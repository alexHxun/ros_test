#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <vector>
#include <string>
#include <memory>
#include <cmath>


class AutoPlannerCpp : public rclcpp::Node
{
public:
    AutoPlannerCpp();
    void init();

    // 轨迹处理
    void reduceTrajectoryPointsRDP(moveit::planning_interface::MoveGroupInterface::Plan &plan, size_t max_points);
    void smoothTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan);
    void planToPoseAndExport(const geometry_msgs::msg::PoseStamped &target_pose_in,
                                         const geometry_msgs::msg::PoseStamped *start_pose_in /*=nullptr*/);
    void exportEndEffectorTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                     const std::string &ee_link);
    void unwrapTrajectoryAngles(moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // 碰撞物体
    void addSimpleCollisionObject();
    bool load_environment_from_ply(const std::string &ply_path, const std::string &mesh_save_path="/tmp/env_mesh.stl");

private:
    bool hasLargeJointJumps(const trajectory_msgs::msg::JointTrajectory &traj, double max_delta_rad);

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    std::string base_frame_;
};
