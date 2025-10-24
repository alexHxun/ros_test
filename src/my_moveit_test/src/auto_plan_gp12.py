#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
import open3d as o3d
import csv
import os

class AutoPlanner(Node):
    def __init__(self):
        super().__init__('auto_planner_gp12')

        # 初始化 MoveIt 接口
        roscpp_initialize([])
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

        self.base_frame = self.group.get_planning_frame()
        self.get_logger().info(f"Planning frame: {self.base_frame}")

    def load_environment_from_ply(self, ply_path, mesh_save_path="/tmp/env_mesh.stl"):
        """从点云生成三角网格并添加到场景"""
        self.get_logger().info(f"Loading point cloud: {ply_path}")
        pcd = o3d.io.read_point_cloud(ply_path)
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=0.02)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(mesh_save_path, mesh)
        self.get_logger().info(f"Converted to mesh: {mesh_save_path}")

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.orientation.w = 1.0
        self.scene.add_mesh("environment", pose, mesh_save_path)
        self.get_logger().info("Environment mesh added to planning scene.")
    
    def plan_and_export(self, target_pose, output_csv="/tmp/trajectory.csv"):
        """规划到目标位姿并导出轨迹"""
        self.group.set_pose_target(target_pose)

        self.get_logger().info("Planning...")
        plan = self.group.plan()
        if not plan or len(plan.joint_trajectory.points) == 0:
            self.get_logger().error("Planning failed.")
            return

        self.get_logger().info(f"Planning success! {len(plan.joint_trajectory.points)} trajectory points generated.")
        
        # 导出轨迹
        with open(output_csv, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(plan.joint_trajectory.joint_names)
            for p in plan.joint_trajectory.points:
                writer.writerow(p.positions)
        self.get_logger().info(f"Trajectory saved to: {output_csv}")

def main():
    rclpy.init()
    node = AutoPlanner()

    # 1️⃣ 加载环境点云
    ply_file = "/mnt/hgfs/shared_sim/movtion_planner/data/1/cpp/scene.ply"   # 修改为你的实际路径
    node.load_environment_from_ply(ply_file)

    # 2️⃣ 设置目标位姿
    target_pose = PoseStamped()
    target_pose.header.frame_id = node.base_frame
    target_pose.pose.position.x = 0.45
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.35
    target_pose.pose.orientation.w = 1.0

    # 3️⃣ 规划并导出轨迹
    node.plan_and_export(target_pose)

    rclpy.shutdown()
    roscpp_shutdown()

if __name__ == "__main__":
    main()
