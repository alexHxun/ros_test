// src/main.cpp
#include "auto_planner.hpp"
#include "helper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // 创建 node 并交给 executor 管理
  auto node = std::make_shared<AutoPlannerCpp>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // 先启动后台 spin，让订阅回调能被处理（非常关键）
  std::thread spin_thread([&executor]() {
    executor.spin();
  });

  // 等待一点时间让 executor 真正开始（不是必须，但能提高稳定性）
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 现在安全地调用 init()，init() 内会等待 current_state（此时回调已在后台运行）
  try {
    node->init();
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to init node: %s", ex.what());
    rclcpp::shutdown();
    if (spin_thread.joinable()) spin_thread.join();
    return 1;
  }

  // 稍作延迟，确保 MoveIt 端也准备好（可选）
  std::this_thread::sleep_for(std::chrono::seconds(1));

  node->addSimpleCollisionObject();
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "world";
  // target_pose.header.frame_id = node->get_parameter_or<std::string>("planning_frame", "base_link");
  target_pose.pose.position.x = 0.65;
  target_pose.pose.position.y = 0.25;
  target_pose.pose.position.z = 0.6;
  // 绕 Y 轴旋转 20°，使末端法向与 XOY 平面成 20°角
  double pitch_deg = 90;
  double pitch_rad = pitch_deg * M_PI / 180.0;

  // roll 和 yaw 设为 0，仅改变俯仰角
  tf2::Quaternion q;
  q.setRPY(0.0, pitch_rad, 0.0);
  q.normalize();

  // 转为 geometry_msgs::Quaternion
  target_pose.pose.orientation = tf2::toMsg(q);

  // 构造显式的起始 pose（示例值）
  // 注意：start_pose 的位置/朝向应以与 planning_frame 相同的 frame 表达
  geometry_msgs::msg::PoseStamped start_pose;
  start_pose.header.frame_id = "world";

  // start_pose.header.frame_id = node->get_parameter_or<std::string>("planning_frame", "base_link");
  // 例：把起始位姿放在比当前更后方一点（请根据你的实际场景填写）
  start_pose.pose.position.x = 0.3;
  start_pose.pose.position.y = 0.0;
  start_pose.pose.position.z = 0.1;
  // 明确指定起始朝向（最好用四元数）
  start_pose.pose.orientation.w = 1.0;
  start_pose.pose.orientation.x = 0.0;
  start_pose.pose.orientation.y = 0.0;
  start_pose.pose.orientation.z = 0.0;

  // 调用带起始位姿的 planning 接口
  auto t_start = std::chrono::steady_clock::now();
  node->planToPoseAndExport(target_pose, &start_pose);
  auto t_end = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  RCLCPP_INFO(node->get_logger(), "planToPoseAndExport 耗时: %.3f 秒 (%.0f 毫秒)",
              elapsed_ms / 1000.0, (double)elapsed_ms);
  // optionally diagnose
  diagnoseEeCsv("/tmp/ee_trajectory_full.csv", node);

  // 退出前稍等
  std::this_thread::sleep_for(std::chrono::seconds(1));

  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  return 0;
}
