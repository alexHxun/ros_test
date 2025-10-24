// src/auto_planner.cpp
#include "auto_planner.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/conversions.h>
#include <pcl/PolygonMesh.h>

// note: we don't call savePolygonFileSTL here to avoid PCL-version issues
// If you want explicit STL/VTK output later, we can add conditional code.

#include <filesystem>
#include <fstream>
#include <chrono>
#include <cmath>
#include <Eigen/Geometry>

AutoPlannerCpp::AutoPlannerCpp()
    : Node("auto_planner_cpp")
{
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    RCLCPP_INFO(this->get_logger(), "AutoPlannerCpp constructed (deferred MoveGroup init).");
}

void AutoPlannerCpp::init()
{
    auto node_shared = this->shared_from_this();
    std::string planning_group = this->declare_parameter<std::string>("planning_group", "panda_arm");
    RCLCPP_INFO(this->get_logger(), "Using planning group: %s", planning_group.c_str());

    const int max_node_wait = 30;
    int node_waited = 0;
    while (rclcpp::ok() && node_waited < max_node_wait)
    {
        auto names = this->get_node_graph_interface()->get_node_names();
        bool found = false;
        for (const auto &n : names)
        {
            if (n.find("move_group") != std::string::npos) { found = true; break; }
        }
        if (found) break;
        ++node_waited;
        RCLCPP_INFO(this->get_logger(), "Waiting for any 'move_group' node... (%d/%d)", node_waited, max_node_wait);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    const int max_construct_attempts = 30;
    int attempt = 0;
    while (rclcpp::ok() && attempt < max_construct_attempts)
    {
        ++attempt;
        try
        {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared, planning_group);
            base_frame_ = move_group_->getPlanningFrame();

            // 等待 current robot state 可用（超时参数可调）
            const double wait_timeout = this->get_parameter_or<double>("wait_for_current_state_timeout", 5.0); // seconds
            const double poll_interval = 0.2; // sec
            double waited = 0.0;
            RCLCPP_INFO(this->get_logger(), "Waiting up to %.1f s for valid current robot state...", wait_timeout);

            bool got_valid_state = false;
            while (rclcpp::ok() && waited < wait_timeout) {
                auto current_state = move_group_->getCurrentState();
                if (current_state) {
                    // 使用 RobotState 的变量计数检查（更通用）
                    size_t var_count = current_state->getVariableCount();
                    if (var_count > 0) {
                        RCLCPP_INFO(this->get_logger(), "Got non-empty current robot state (variables=%zu).", var_count);
                        got_valid_state = true;
                        break;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Current robot state variables empty (count=0).");
                    }
                } else {
                    RCLCPP_DEBUG(this->get_logger(), "CurrentState pointer is null yet.");
                }
                std::this_thread::sleep_for(std::chrono::duration<double>(poll_interval));
                waited += poll_interval;
            }
            if (!got_valid_state) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for current robot state — /joint_states may not be published. Continue but planning may fail.");
            }

            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface created. Planning frame: %s", base_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
            return;
        }
        catch (const std::exception &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Attempt %d/%d: failed to construct MoveGroupInterface: %s",
                        attempt, max_construct_attempts, ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    RCLCPP_ERROR(this->get_logger(), "Unable to construct MoveGroupInterface.");
    throw std::runtime_error("Check move_group, SRDF, and planning group name");
}

void AutoPlannerCpp::addSimpleCollisionObject()
{
    moveit_msgs::msg::CollisionObject co;
    co.id = "obstacle_box";
    co.header.frame_id = base_frame_.empty() ? "base_link" : base_frame_.c_str();

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {0.05, 0.10, 0.15};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.55;
    box_pose.position.y = 0.10;
    box_pose.position.z = 0.40;
    box_pose.orientation.w = 1.0;

    co.primitives.push_back(box);
    co.primitive_poses.push_back(box_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_->addCollisionObjects({co});
    RCLCPP_INFO(this->get_logger(), "Added collision box at (%.2f, %.2f, %.2f)",
                box_pose.position.x, box_pose.position.y, box_pose.position.z);
}


bool AutoPlannerCpp::load_environment_from_ply(const std::string &ply_path, const std::string &mesh_save_path)
{
    if (!std::filesystem::exists(ply_path)) {
        RCLCPP_ERROR(this->get_logger(), "PLY file not found: %s", ply_path.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Loading point cloud: %s", ply_path.c_str());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPLYFile(ply_path, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PLY file: %s", ply_path.c_str());
        return false;
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    if (cloud->empty()) {
        RCLCPP_ERROR(this->get_logger(), "Point cloud empty after removing NaNs.");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Point cloud loaded: %zu points", cloud->size());

    // estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    ne.setKSearch(20);
    ne.compute(*normals);

    // concatenate fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // triangulate
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>());
    tree2->setInputCloud(cloud_with_normals);

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    gp3.setSearchRadius(0.05);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4);
    gp3.setMinimumAngle(M_PI / 18);
    gp3.setMaximumAngle(2 * M_PI / 3);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    RCLCPP_INFO(this->get_logger(), "Triangulation produced %zu polygons", triangles.polygons.size());

    // NOTE: don't call pcl::io::savePolygonFileSTL here to avoid PCL-API/headers mismatch.
    // If you need to save mesh to disk, add an explicit PCL/VTK save path for your environment.
    RCLCPP_INFO(this->get_logger(), "Skipping mesh file save (mesh_save_path=%s) to avoid PCL-version issues", mesh_save_path.c_str());

    // convert to shape_msgs::Mesh
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
        if (poly.vertices.size() < 3) continue;
        shape_msgs::msg::MeshTriangle tri;
        tri.vertex_indices[0] = poly.vertices[0];
        tri.vertex_indices[1] = poly.vertices[1];
        tri.vertex_indices[2] = poly.vertices[2];
        mesh_msg.triangles.push_back(tri);
    }

    moveit_msgs::msg::CollisionObject co;
    co.id = "environment_mesh";
    co.header.frame_id = base_frame_.empty() ? "world" : base_frame_.c_str();
    co.meshes.push_back(mesh_msg);

    geometry_msgs::msg::Pose mesh_pose;
    mesh_pose.orientation.w = 1.0;
    mesh_pose.position.z = -0.1;
    co.mesh_poses.push_back(mesh_pose);
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    planning_scene_->addCollisionObjects({co});
    RCLCPP_INFO(this->get_logger(), "Added collision object 'environment_mesh' to planning scene (frame: %s)",
                co.header.frame_id.c_str());

    return true;
}

bool AutoPlannerCpp::hasLargeJointJumps(const trajectory_msgs::msg::JointTrajectory &traj, double max_delta_rad)
{
    if (traj.points.size() < 2) return false;
    for (size_t i = 1; i < traj.points.size(); ++i) {
        const auto &prev = traj.points[i-1].positions;
        const auto &cur  = traj.points[i].positions;
        if (prev.size() != cur.size()) continue;
        for (size_t j = 0; j < prev.size(); ++j) {
            double d = std::fabs(cur[j] - prev[j]);
            if (d > max_delta_rad) {
                RCLCPP_WARN(this->get_logger(), "Detected large joint jump: joint[%zu] delta=%.4f rad > threshold=%.4f",
                            j, d, max_delta_rad);
                return true;
            }
        }
    }
    return false;
}

void AutoPlannerCpp::smoothTrajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan)
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

void AutoPlannerCpp::exportEndEffectorTrajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                                                 const std::string &ee_link)
{
    if (!move_group_) {
        RCLCPP_WARN(this->get_logger(), "move_group_ not initialized; cannot export EE trajectory.");
        return;
    }

    auto current_state = move_group_->getCurrentState();
    if (!current_state) {
        RCLCPP_WARN(this->get_logger(), "Failed to get current robot state for FK export.");
        return;
    }

    const auto &robot_model = current_state->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(move_group_->getName());
    if (!jmg) {
        RCLCPP_WARN(this->get_logger(), "Failed to get JointModelGroup '%s' for export.", move_group_->getName().c_str());
        return;
    }

    std::ofstream ofs("/tmp/ee_trajectory_full.csv");
    if (!ofs.is_open()) {
        RCLCPP_WARN(this->get_logger(), "Cannot open /tmp/ee_trajectory_full.csv for writing.");
        return;
    }
    ofs << "x,y,z,qx,qy,qz,qw,roll,pitch,yaw\n";

    const auto &traj = plan.trajectory_.joint_trajectory;
    for (const auto &pt : traj.points) {
        std::vector<double> vals(pt.positions.begin(), pt.positions.end());
        current_state->setJointGroupPositions(jmg, vals);
        current_state->update();
        const Eigen::Isometry3d &tf = current_state->getGlobalLinkTransform(ee_link);
        const Eigen::Vector3d t = tf.translation();
        Eigen::Quaterniond q(tf.rotation());
        // compute RPY
        Eigen::Vector3d rpy = tf.rotation().eulerAngles(0,1,2); // roll, pitch, yaw
        ofs << t.x() << "," << t.y() << "," << t.z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
            << rpy.x() << "," << rpy.y() << "," << rpy.z() << "\n";
    }
    ofs.close();
    RCLCPP_INFO(this->get_logger(), "Exported EE full poses to /tmp/ee_trajectory_full.csv (ee_link=%s)", ee_link.c_str());
}


// 在 class AutoPlannerCpp 的实现文件中替换原有 planToPoseAndExport
void AutoPlannerCpp::planToPoseAndExport(const geometry_msgs::msg::PoseStamped &target_pose_in,
                                         const geometry_msgs::msg::PoseStamped *start_pose_in /*=nullptr*/)
{
    if (!move_group_) {
        RCLCPP_ERROR(this->get_logger(), "move_group_ not initialized. Call init() first.");
        return;
    }

    // PARAMETERS (和你原先保持一致)
    double joint_jump_threshold = this->get_parameter_or<double>("joint_jump_threshold", 0.8);
    double cartesian_jump_threshold_cfg = this->get_parameter_or<double>("cartesian_jump_threshold", 0.4);
    double eef_step_cfg = this->get_parameter_or<double>("eef_step", 0.02);
    bool cartesian_fallback = this->get_parameter_or<bool>("cartesian_fallback_enabled", true);

    // NOTE: 不再在此处强制 setStartStateToCurrentState() —— 如果用户提供了 start_pose，会在下面替换 start state
    move_group_->setStartStateToCurrentState(); // 先设置为当前状态，可能被下文替换

    // 获取 current pose（仅用于 default orientation / diagnostics）
    geometry_msgs::msg::PoseStamped current_pose;
    try {
        current_pose = move_group_->getCurrentPose();
    } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Unable to get current pose via move_group_->getCurrentPose()");
    }

    std::string planning_frame = move_group_->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame.c_str());

    // --- 如果用户传入了 start_pose_in，尝试把它作为规划的 start state（通过 IK 转换为 joint）
    if (start_pose_in) {
        // frame 检查
        if (!start_pose_in->header.frame_id.empty() && start_pose_in->header.frame_id != planning_frame) {
            RCLCPP_WARN(this->get_logger(),
                        "start_pose frame '%s' != planning frame '%s'. Start pose must be expressed in planning frame (or transform it).",
                        start_pose_in->header.frame_id.c_str(), planning_frame.c_str());
            // 我们仍会尝试，但坐标语义可能不正确
        }

        // 取当前 robot_state 作为基础
        auto current_state = move_group_->getCurrentState();
        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Cannot get current RobotState; will use current as start.");
        } else {
            // 准备 RobotState rs，并用 IK 求解 start_pose 的关节解（用当前关节作为 seed）
            moveit::core::RobotState rs(*current_state);
            const moveit::core::JointModelGroup* jmg = rs.getJointModelGroup(move_group_->getName());
            if (!jmg) {
                RCLCPP_WARN(this->get_logger(), "Failed to get JointModelGroup for start-pose IK.");
            } else {
                // 把 geometry_msgs::Pose -> Eigen::Isometry3d
                Eigen::Quaterniond qs(start_pose_in->pose.orientation.w,
                                      start_pose_in->pose.orientation.x,
                                      start_pose_in->pose.orientation.y,
                                      start_pose_in->pose.orientation.z);
                qs.normalize();
                Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
                iso.linear() = qs.toRotationMatrix();
                iso.translation() = Eigen::Vector3d(start_pose_in->pose.position.x,
                                                    start_pose_in->pose.position.y,
                                                    start_pose_in->pose.position.z);

                // 用带 seed 的 setFromIK（timeout 可调）
                std::vector<double> seed;
                rs.copyJointGroupPositions(jmg, seed);            // 读取当前 joint 值作为 seed
                rs.setJointGroupPositions(jmg, seed);             // 把 seed 写回 rs（确保 IK solver 以此为初始解）

                double ik_timeout = 0.02; // seconds
                bool ik_ok = false;

                // 直接使用带 tip + timeout 的重载；该函数会使用 rs 当前的 joint 值作为 seed
                try {
                    ik_ok = rs.setFromIK(jmg, iso, move_group_->getEndEffectorLink(), ik_timeout);
                } catch (...) {
                    // 作为兼容回退，可尝试不带 tip 的签名（部分版本存在）
                    ik_ok = rs.setFromIK(jmg, iso, ik_timeout);
                }

                if (!ik_ok) {
                    RCLCPP_WARN(this->get_logger(), "IK failed for provided start_pose (seeded).");
                } else {
                    // IK 成功：rs 已包含解，可把它设为 move_group_ 的 start state
                    move_group_->setStartState(rs);
                    RCLCPP_INFO(this->get_logger(), "Start pose IK success; using it as planning start state.");
                }
            }
        }
    }

    // ------------- 下面是你原来的目标 pose 处理逻辑（几乎未改，仅把 target_pose_in 处理为 target_pose） --------------
    geometry_msgs::msg::PoseStamped target_pose = target_pose_in;

    // 如果目标 orientation 是单位四元数 (1,0,0,0)（你之前用在 main 中），可以用当前朝向代替（可选）
    if ((std::abs(target_pose.pose.orientation.w - 1.0) < 1e-9) &&
        std::abs(target_pose.pose.orientation.x) < 1e-9 &&
        std::abs(target_pose.pose.orientation.y) < 1e-9 &&
        std::abs(target_pose.pose.orientation.z) < 1e-9) {
        if (!current_pose.header.frame_id.empty()) {
            target_pose.pose.orientation = current_pose.pose.orientation;
            RCLCPP_INFO(this->get_logger(), "Target orientation was identity; replaced with current EE orientation to avoid flips.");
        }
    }

    if (target_pose.header.frame_id.empty()) {
        target_pose.header.frame_id = planning_frame;
    } else if (target_pose.header.frame_id != planning_frame) {
        RCLCPP_WARN(this->get_logger(),
                    "Target pose frame '%s' != planning frame '%s'. Ensure pose is expressed in planning frame or provide TF between them.",
                    target_pose.header.frame_id.c_str(), planning_frame.c_str());
        target_pose.header.frame_id = planning_frame;
    }

    // normalize quaternion
    auto normalize_q = [](geometry_msgs::msg::Quaternion &q){
        double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (n > 0.0) { q.x/=n; q.y/=n; q.z/=n; q.w/=n; }
    };
    normalize_q(target_pose.pose.orientation);
    normalize_q(current_pose.pose.orientation);

    // 保证四元数最短弧（与你原来相同）
    if (!current_pose.header.frame_id.empty()) {
        double dot = current_pose.pose.orientation.w*target_pose.pose.orientation.w +
                     current_pose.pose.orientation.x*target_pose.pose.orientation.x +
                     current_pose.pose.orientation.y*target_pose.pose.orientation.y +
                     current_pose.pose.orientation.z*target_pose.pose.orientation.z;
        if (dot < 0.0) {
            target_pose.pose.orientation.w = -target_pose.pose.orientation.w;
            target_pose.pose.orientation.x = -target_pose.pose.orientation.x;
            target_pose.pose.orientation.y = -target_pose.pose.orientation.y;
            target_pose.pose.orientation.z = -target_pose.pose.orientation.z;
        }
    }

    // ------------- 此处继续你原来的 Cartesian / Joint planning 流程（未改） --------------
    // 先尝试笛卡尔路径（start state 已经基于 start_pose_in 可能被覆盖）
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        // 注意： computeCartesianPath 采用的 waypoints 是 geometry_msgs::Pose（无 header），
        // 并且是相对于 planning frame 的 pose；这里我们用已经在 planning frame 的 current_pose 与 target_pose
        waypoints.push_back(current_pose.pose);
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory cart_traj_msg;
        double frac = move_group_->computeCartesianPath(waypoints, eef_step_cfg, cartesian_jump_threshold_cfg, cart_traj_msg, true);
        RCLCPP_INFO(this->get_logger(), "Initial Cartesian trial: fraction=%.3f (eef_step=%.3f, jump_thresh=%.3f)", frac, eef_step_cfg, cartesian_jump_threshold_cfg);

        if (frac > 0.90 && !cart_traj_msg.joint_trajectory.points.empty()) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = cart_traj_msg;
            smoothTrajectory(plan);
            reduceTrajectoryPointsRDP(plan, 10);
            RCLCPP_INFO(this->get_logger(), "Using Cartesian trajectory (simple two-point interpolation). Points=%zu",
                        plan.trajectory_.joint_trajectory.points.size());
            exportEndEffectorTrajectory(plan, move_group_->getEndEffectorLink());
            return;
        } else {
            RCLCPP_WARN(this->get_logger(), "Initial Cartesian attempt insufficient (fraction=%.3f). Will try denser waypoints then constrained joint-space.", frac);
            moveit_msgs::msg::RobotTrajectory debug_traj;
            double frac_no_coll = move_group_->computeCartesianPath(waypoints, eef_step_cfg, cartesian_jump_threshold_cfg, debug_traj, false);
            RCLCPP_INFO(this->get_logger(), "Cartesian (no collisions) fraction=%.3f", frac_no_coll);
        }
    }

    // 若笛卡尔失败，继续 orientation constraint + joint-space planning（你的原有逻辑）
    auto current_state2 = move_group_->getCurrentState();
    if (!current_state2) {
        RCLCPP_ERROR(this->get_logger(), "Cannot get current RobotState; aborting.");
        return;
    }

    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = move_group_->getEndEffectorLink();
    oc.header.frame_id = planning_frame;
    oc.orientation = target_pose.pose.orientation;
    oc.absolute_x_axis_tolerance = 0.12;
    oc.absolute_y_axis_tolerance = 0.12;
    oc.absolute_z_axis_tolerance = 0.20;
    oc.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(oc);
    move_group_->setPathConstraints(path_constraints);

    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(6.0);
    move_group_->setNumPlanningAttempts(6);

    move_group_->setStartStateToCurrentState(); // ensure start state consistent with internal move_group_ if needed
    move_group_->setPoseTarget(target_pose.pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto status = move_group_->plan(plan);
    bool success = (status == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Constrained joint-space planning failed; clearing constraints and trying unconstrained replan.");
        move_group_->clearPathConstraints();

        // 回退：无约束关节空间规划
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target_pose.pose);
        auto status2 = move_group_->plan(plan);
        success = (status2 == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
        move_group_->clearPathConstraints();
    }

    if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed after Cartesian and joint-space attempts. Check TF frames (planning_frame='%s'), IK solver and collision objects.", planning_frame.c_str());
        return;
    }

    // 最终轨迹后处理
    unwrapTrajectoryAngles(plan);
    smoothTrajectory(plan);
    reduceTrajectoryPointsRDP(plan, 10);
    RCLCPP_INFO(this->get_logger(), "Planning succeeded: %zu trajectory points", plan.trajectory_.joint_trajectory.points.size());

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

    exportEndEffectorTrajectory(plan, move_group_->getEndEffectorLink());
}

void AutoPlannerCpp::unwrapTrajectoryAngles(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    auto &traj = plan.trajectory_.joint_trajectory;
    if (traj.points.empty() || traj.joint_names.empty()) return;

    const double TWO_PI = 2.0 * M_PI;
    const double PI = M_PI;

    // 对每个 joint independently 做 unwrap（以 trajectory 的第一个点为基准）
    size_t nj = traj.joint_names.size();
    // 复制第一个点作为基准（后续点会被调整为与前一点最接近）
    std::vector<double> prev;
    prev.resize(nj);
    for (size_t j = 0; j < nj; ++j) {
        prev[j] = (traj.points.front().positions.size() > j) ? traj.points.front().positions[j] : 0.0;
    }

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto &pt = traj.points[i];
        // positions 数量应等于 nj（但保险起见检查）
        size_t psize = pt.positions.size();
        if (psize < nj) {
            // 不常见：跳过不完整点
            continue;
        }
        for (size_t j = 0; j < nj; ++j) {
            double cur = pt.positions[j];
            double diff = cur - prev[j];

            // 把 diff 移到 (-pi, pi]，通过对 cur 加/减 2π
            if (diff > PI || diff <= -PI) {
                // 把 cur 调整到与 prev 最近的等价值
                // 计算需要加/减的倍数 k
                double k = std::round((prev[j] - cur) / TWO_PI);
                cur += k * TWO_PI;
                // 再次微调（以防 numerical edge）
                while ((cur - prev[j]) > PI) cur -= TWO_PI;
                while ((cur - prev[j]) <= -PI) cur += TWO_PI;
            }

            // 将调整后的值写回
            pt.positions[j] = cur;
            prev[j] = cur; // 更新 prev，供下一时间步使用
        }
    }
}
