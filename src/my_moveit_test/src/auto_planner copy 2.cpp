#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/parameter_client.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// 推荐用新版 tf2_geomet#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/parameter_client.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // 用新版 .hpp，替代旧的 .h

#include <moveit/robot_state/robot_state.h>         // RobotState
#include <moveit/robot_model/joint_model_group.h>   // JointModelGroup
#include <Eigen/Geometry>                            // Eigen types

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <filesystem>

// PCL includes
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>

#include "rd_waypoints.hpp"

class AutoPlannerCpp : public rclcpp::Node
{
public:
  void reduceTrajectoryPointsRDP(moveit::planning_interface::MoveGroupInterface::Plan &plan, size_t max_points);

  AutoPlannerCpp()
  : Node("auto_planner_cpp")
  {
    // planning_scene_ 不需要 node 指针，先构造
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    RCLCPP_INFO(get_logger(), "AutoPlannerCpp constructed (deferred MoveGroup init).");
  }

  // 必须在 node 已由 shared_ptr 管理且已加入 executor 后调用 init()
  void init()
  {
    auto node_shared = this->shared_from_this();

    // 读取 planning_group 参数（默认 panda_arm 以兼容 moveit_tutorials demo）
    std::string planning_group = this->declare_parameter<std::string>("planning_group", "panda_arm");
    RCLCPP_INFO(get_logger(), "Using planning group: %s", planning_group.c_str());

    // 等待 /move_group 节点出现在 graph（非严格匹配，任意包含 move_group）
    const int max_node_wait = 30;
    int node_waited = 0;
    while (rclcpp::ok() && node_waited < max_node_wait)
    {
      auto names = this->get_node_graph_interface()->get_node_names();
      bool found = false;
      for (const auto &n : names) {
        if (n.find("move_group") != std::string::npos) { found = true; break; }
      }
      if (found) break;
      ++node_waited;
      RCLCPP_INFO(get_logger(), "Waiting for any 'move_group' node to appear... (%d/%d)", node_waited, max_node_wait);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 直接尝试构造 MoveGroupInterface（以 planning_group 作为组名），失败则重试
    const int max_construct_attempts = 30;
    int attempt = 0;
    while (rclcpp::ok() && attempt < max_construct_attempts)
    {
      ++attempt;
      try
      {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, planning_group);
        // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        //     node_shared, "arm", rclcpp::NodeOptions().namespace_("motoman_gp12")
        // );

        base_frame_ = move_group_->getPlanningFrame();
        RCLCPP_INFO(get_logger(), "MoveGroupInterface created (group=%s). Planning frame: %s",
                    planning_group.c_str(), base_frame_.c_str());
        RCLCPP_INFO(get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
        return;
      }
      catch (const std::exception &ex)
      {
        RCLCPP_WARN(get_logger(), "Attempt %d/%d: failed to construct MoveGroupInterface: %s",
                    attempt, max_construct_attempts, ex.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }

    RCLCPP_ERROR(get_logger(), "Unable to construct MoveGroupInterface after %d attempts.", max_construct_attempts);
    throw std::runtime_error("Unable to construct MoveGroupInterface; check move_group, SRDF and planning group name");
  }

  // 新增函数：添加简单碰撞盒
  void addSimpleCollisionObject()
  {
      moveit_msgs::msg::CollisionObject co;
      co.id = "obstacle_box";
      co.header.frame_id = base_frame_.empty() ? "world" : base_frame_;

      // 使用 box 作为碰撞体
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = {0.51, 0.51, 0.02};  // x, y, z 尺寸 (米)

      // 位于机械臂和目标之间
      geometry_msgs::msg::Pose box_pose;
      box_pose.position.x = -0.25;  // 可以根据你的机械臂和目标调整
      box_pose.position.y = 0.7;
      box_pose.position.z = 0.65;
      box_pose.orientation.w = 1.0;

      co.primitives.push_back(box);
      co.primitive_poses.push_back(box_pose);
      co.operation = moveit_msgs::msg::CollisionObject::ADD;

      std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
      collision_objects.push_back(co);

      planning_scene_->addCollisionObjects(collision_objects);
      RCLCPP_INFO(get_logger(), "Added simple collision box to planning scene (frame: %s)", co.header.frame_id.c_str());
  }

  /// 从 PLY 点云生成三角网格，并添加至 PlanningScene 作为碰撞物体
  bool load_environment_from_ply(const std::string &ply_path, const std::string &mesh_save_path="/tmp/env_mesh.stl")
  {
    if (!std::filesystem::exists(ply_path)) {
      RCLCPP_ERROR(get_logger(), "PLY file not found: %s", ply_path.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Loading point cloud: %s", ply_path.c_str());

    // 1) 读取 PLY 点云到 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(ply_path, *cloud) == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to load PLY file: %s", ply_path.c_str());
      return false;
    }

    // 清除 NaN
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    if (cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "Point cloud empty after removing NaNs.");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Point cloud loaded: %zu points", cloud->size());

    // 2) 估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setKSearch(20);
    ne.compute(*normals);

    // 3) 合并点与法线，准备三角化
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // 4) 使用 GreedyProjectionTriangulation 进行三角化
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>());
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // 这些参数可按点云密度调节
    gp3.setSearchRadius(0.05);          // 邻域搜索半径（米）
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    RCLCPP_INFO(get_logger(), "Triangulation produced %zu polygons", triangles.polygons.size());

    // 5) 保存 STL（可选，但便于调试）
    try {
      pcl::io::savePolygonFileSTL(mesh_save_path, triangles);
      RCLCPP_INFO(get_logger(), "Saved mesh to: %s", mesh_save_path.c_str());
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to save STL: %s", e.what());
    }

    // 6) 将 pcl::PolygonMesh 转换为 shape_msgs::msg::Mesh
    // triangles.cloud 是 sensor_msgs::msg::PointCloud2
    pcl::PointCloud<pcl::PointXYZ> cloud_vertices;
    pcl::fromPCLPointCloud2(triangles.cloud, cloud_vertices);

    shape_msgs::msg::Mesh mesh_msg;
    mesh_msg.vertices.reserve(cloud_vertices.size());
    for (const auto &v : cloud_vertices.points) {
      geometry_msgs::msg::Point p;
      p.x = static_cast<double>(v.x);
      p.y = static_cast<double>(v.y);
      p.z = static_cast<double>(v.z);
      mesh_msg.vertices.push_back(p);
    }

    for (const auto &poly : triangles.polygons) {
      if (poly.vertices.size() < 3) continue; // 跳过异常多边形
      shape_msgs::msg::MeshTriangle tri;
      // 这里只保留前三个索引（三角形）
      tri.vertex_indices[0] = poly.vertices[0];
      tri.vertex_indices[1] = poly.vertices[1];
      tri.vertex_indices[2] = poly.vertices[2];
      mesh_msg.triangles.push_back(tri);
    }

    // 7) 构造 CollisionObject 并添加到 planning scene
    moveit_msgs::msg::CollisionObject co;
    co.id = "environment_mesh";
    co.header.frame_id = base_frame_.empty() ? "world" : base_frame_;

    co.meshes.push_back(mesh_msg);
    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    mesh_pose.position.z = -0.1;
    co.mesh_poses.push_back(mesh_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(co);

    planning_scene_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(get_logger(), "Added collision object 'environment_mesh' to planning scene (frame: %s)", co.header.frame_id.c_str());

    return true;
  }


// 在文件头部（你的 includes 已经包含所需头）
// helper: 检测轨迹相邻点是否存在大幅关节跳变
  bool hasLargeJointJumps(const trajectory_msgs::msg::JointTrajectory &traj, double max_delta_rad)
  {
    if (traj.points.size() < 2) return false;
    for (size_t i = 1; i < traj.points.size(); ++i) {
      const auto &prev = traj.points[i-1].positions;
      const auto &cur  = traj.points[i].positions;
      if (prev.size() != cur.size()) continue;
      for (size_t j = 0; j < prev.size(); ++j) {
        double d = std::fabs(cur[j] - prev[j]);
        if (d > max_delta_rad) {
          RCLCPP_WARN(this->get_logger(), "Detected large joint jump: joint[%zu] delta=%.4f rad > threshold=%.4f", j, d, max_delta_rad);
          return true;
        }
      }
    }
    return false;
  }

  // 平滑 / 时间参数化（成员版本）
  void smoothTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan)
  {
    if (!move_group_) return;

    robot_trajectory::RobotTrajectory rt(move_group_->getCurrentState()->getRobotModel(),
                                        move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), plan.trajectory_);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ok = iptp.computeTimeStamps(rt);
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Time parameterization failed, trajectory may be non-smooth.");
    } else {
      rt.getRobotTrajectoryMsg(plan.trajectory_);
    }
  }


  void exportEndEffectorTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                  const std::string &ee_link)
  {
    if (!move_group_) {
      RCLCPP_WARN(this->get_logger(), "move_group_ not initialized; cannot export EE trajectory.");
      return;
    }

    // 获取 robot model & joint group
    moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
    if (!current_state) {
      RCLCPP_WARN(this->get_logger(), "Failed to get current robot state for FK export.");
      return;
    }
    const moveit::core::RobotModelConstPtr &robot_model = current_state->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(move_group_->getName());
    if (!jmg) {
      RCLCPP_WARN(this->get_logger(), "Failed to get JointModelGroup '%s' for export.", move_group_->getName().c_str());
      return;
    }

    std::ofstream ofs("/tmp/ee_trajectory.csv");
    if (!ofs.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Cannot open /tmp/ee_trajectory.csv for writing.");
      return;
    }
    ofs << "x,y,z\n";

    const auto &traj = plan.trajectory_.joint_trajectory;
    for (const auto &pt : traj.points) {
      // set joint positions
      std::vector<double> vals(pt.positions.begin(), pt.positions.end());
      current_state->setJointGroupPositions(jmg, vals);
      current_state->update(); // update transforms
      const Eigen::Isometry3d &tf = current_state->getGlobalLinkTransform(ee_link);
      const Eigen::Vector3d &t = tf.translation();
      ofs << t.x() << "," << t.y() << "," << t.z() << "\n";
    }
    ofs.close();
    RCLCPP_INFO(this->get_logger(), "Exported EE poses to /tmp/ee_trajectory.csv (ee_link=%s)", ee_link.c_str());
  }

  void planToPoseAndExport(const geometry_msgs::msg::PoseStamped &target_pose)
  {
    if (!move_group_) {
      RCLCPP_ERROR(get_logger(), "move_group_ not initialized. Call init() first.");
      return;
    }

    // 参数
    double joint_jump_threshold = this->get_parameter_or<double>("joint_jump_threshold", 0.8); // rad
    double cartesian_jump_threshold = this->get_parameter_or<double>("cartesian_jump_threshold", 2.0);
    double eef_step = this->get_parameter_or<double>("eef_step", 0.01);
    bool cartesian_fallback = this->get_parameter_or<bool>("cartesian_fallback_enabled", true);

    // 起始状态为当前
    move_group_->setStartStateToCurrentState();

    // 读取当前 pose（用于笛卡尔回退起点）
    geometry_msgs::msg::PoseStamped current_pose;
    try {
      current_pose = move_group_->getCurrentPose();
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Unable to get current pose via move_group_->getCurrentPose()");
      // 如果无法读到，仍继续，但笛卡尔回退可能失败
    }

    // 保证四元数连续与归一化
    geometry_msgs::msg::Quaternion q_target = target_pose.pose.orientation;
    {
      // normalize target quaternion
      double n = std::sqrt(q_target.x*q_target.x + q_target.y*q_target.y + q_target.z*q_target.z + q_target.w*q_target.w);
      if (n > 0.0) {
        q_target.x /= n; q_target.y /= n; q_target.z /= n; q_target.w /= n;
      }
    }
    if (!current_pose.header.frame_id.empty()) {
      geometry_msgs::msg::Quaternion q_current = current_pose.pose.orientation;
      // normalize current too
      double nc = std::sqrt(q_current.x*q_current.x + q_current.y*q_current.y + q_current.z*q_current.z + q_current.w*q_current.w);
      if (nc > 0.0) {
        q_current.x /= nc; q_current.y /= nc; q_current.z /= nc; q_current.w /= nc;
      }
      double dot = q_current.w * q_target.w + q_current.x * q_target.x +
                  q_current.y * q_target.y + q_current.z * q_target.z;
      if (dot < 0.0) {
        // flip sign to ensure shortest path
        q_target.w = -q_target.w;
        q_target.x = -q_target.x;
        q_target.y = -q_target.y;
        q_target.z = -q_target.z;
      }
    }

    geometry_msgs::msg::Pose corrected_pose = target_pose.pose;
    corrected_pose.orientation = q_target;

    // 设置目标并规划（普通姿态规划）
    move_group_->setPoseTarget(corrected_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto status = move_group_->plan(plan);
    bool success = (status == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Initial planning failed (setPoseTarget).");
      if (!cartesian_fallback) return;
    }

    // 检查大跳变
    bool has_jump = false;
    if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
      has_jump = hasLargeJointJumps(plan.trajectory_.joint_trajectory, joint_jump_threshold);
      if (has_jump) {
        RCLCPP_WARN(this->get_logger(), "Planned trajectory has joint jumps > %.3f rad.", joint_jump_threshold);
      }
    }

    // Dense Cartesian fallback: 用大量 waypoints 做线性插值 (位置线性, 方向 slerp)
    if (has_jump && cartesian_fallback) {
      RCLCPP_INFO(this->get_logger(), "Attempting dense Cartesian fallback...");

      // 如果无法获取 current_pose，则无法做笛卡尔回退
      if (current_pose.header.frame_id.empty()) {
        RCLCPP_WARN(this->get_logger(), "Current pose unavailable; cannot perform Cartesian fallback.");
      } else {
        // 使用 Eigen::Quaterniond 做 slerp
        Eigen::Quaterniond q_start, q_end;
        {
          const auto &qs = current_pose.pose.orientation;
          q_start = Eigen::Quaterniond(qs.w, qs.x, qs.y, qs.z);
          q_start.normalize();
        }
        {
          const auto &qe = corrected_pose.orientation;
          q_end = Eigen::Quaterniond(qe.w, qe.x, qe.y, qe.z);
          q_end.normalize();
        }

        geometry_msgs::msg::Pose start_pose = current_pose.pose;
        geometry_msgs::msg::Pose end_pose = corrected_pose;

        const int N = 80; // 插值步数，可调 (50~150)
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.reserve(N+1);
        for (int i = 0; i <= N; ++i) {
          double t = static_cast<double>(i) / static_cast<double>(N);
          geometry_msgs::msg::Pose wp;
          wp.position.x = start_pose.position.x + t * (end_pose.position.x - start_pose.position.x);
          wp.position.y = start_pose.position.y + t * (end_pose.position.y - start_pose.position.y);
          wp.position.z = start_pose.position.z + t * (end_pose.position.z - start_pose.position.z);
          Eigen::Quaterniond qi = q_start.slerp(t, q_end);
          qi.normalize();
          geometry_msgs::msg::Quaternion qmsg;
          qmsg.w = qi.w(); qmsg.x = qi.x(); qmsg.y = qi.y(); qmsg.z = qi.z();
          wp.orientation = qmsg;
          waypoints.push_back(wp);
        }

        moveit_msgs::msg::RobotTrajectory cart_traj_msg;
        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, cartesian_jump_threshold,
                                                          cart_traj_msg, true);
        RCLCPP_INFO(this->get_logger(), "Dense Cartesian path fraction: %.3f", fraction);

        if (fraction > 0.95 && !cart_traj_msg.joint_trajectory.points.empty()) {
          plan.trajectory_ = cart_traj_msg;
          success = true;
          RCLCPP_INFO(this->get_logger(), "Using dense Cartesian fallback trajectory.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Dense Cartesian fallback not sufficient (fraction=%.3f).", fraction);
          if (!success) {
            RCLCPP_ERROR(this->get_logger(), "No valid plan available after Cartesian fallback.");
            return;
          }
        }
      }
    }

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Planning ultimately failed.");
      return;
    }

    // 时间参数化 / 平滑
    smoothTrajectory(plan);
    reduceTrajectoryPointsRDP(plan, 10);
    RCLCPP_INFO(get_logger(), "Planning succeeded: %zu trajectory points", plan.trajectory_.joint_trajectory.points.size());

    // 导出 joint CSV
    std::ofstream ofs("/tmp/trajectory.csv");
    if (!ofs.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Cannot open /tmp/trajectory.csv for writing.");
    } else {
      const auto &names = plan.trajectory_.joint_trajectory.joint_names;
      for (size_t i = 0; i < names.size(); ++i) {
        ofs << names[i];
        if (i + 1 < names.size()) ofs << ",";
      }
      ofs << "\n";
      for (const auto &pt : plan.trajectory_.joint_trajectory.points) {
        for (size_t i = 0; i < pt.positions.size(); ++i) {
          ofs << pt.positions[i];
          if (i + 1 < pt.positions.size()) ofs << ",";
        }
        ofs << "\n";
      }
      ofs.close();
      RCLCPP_INFO(get_logger(), "Trajectory saved to /tmp/trajectory.csv");
    }

    // 导出末端位姿 CSV（用于可视化）
    std::string ee_link = move_group_->getEndEffectorLink();
    exportEndEffectorTrajectory(plan, ee_link);
  }



private:
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::string base_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 1) 创建 node（shared_ptr）并注册到 executor
  auto node = std::make_shared<AutoPlannerCpp>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 2) 在 node 已由 shared_ptr 管理并加入 executor 后，初始化 MoveGroupInterface
  node->init();

  // 3) 后台 spin
  std::thread spin_thread([&executor]() { executor.spin(); });

  // 等待 MoveIt 建立连接
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // 4) 加载点云并添加碰撞体（请修改为你的实际路径）
  // std::string ply_file = "/mnt/hgfs/shared_sim/movtion_planner/data/1/cpcd_in_base.ply";
  // if (std::filesystem::exists(ply_file)) {
  //   node->load_environment_from_ply(ply_file, "/tmp/env_mesh.stl");
  // } else {
  //   RCLCPP_WARN(node->get_logger(), "PLY file not found: %s (skipping env load)", ply_file.c_str());
  // }
  // node->addSimpleCollisionObject();

  // 5) 设置目标位姿并规划
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = node->get_parameter_or<std::string>("planning_frame", "base_link");
  target_pose.pose.position.x = 0.65;
  target_pose.pose.position.y = 0.25;
  target_pose.pose.position.z = 0.5;
  target_pose.pose.orientation.w = -1.0;

  node->planToPoseAndExport(target_pose);

  // 6) 清理退出
  std::this_thread::sleep_for(std::chrono::seconds(1));
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();

  return 0;
}
