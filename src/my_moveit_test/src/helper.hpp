#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>        // ✅ 缺失
#include <vector>         // ✅ 缺失
#include <rclcpp/rclcpp.hpp>

// ===============================================
// 计算两个四元数之间的旋转角度（弧度）
// ===============================================
inline double angleBetweenQuaternions(const Eigen::Quaterniond &a,
                                      const Eigen::Quaterniond &b)
{
    double dot = a.coeffs().dot(b.coeffs());  // coeffs() 顺序: (x,y,z,w)
    // clamp
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    // angle between (treat antipodal same): angle = 2*acos(|dot|)
    return 2.0 * std::acos(std::abs(dot));
}

// ===============================================
// 诊断导出的末端轨迹 CSV 文件，查看相邻姿态差角
// ===============================================
// node 参数可为 nullptr，这样只打印到 stdout
inline void diagnoseEeCsv(const std::string &csv_path = "/tmp/ee_trajectory_full.csv",
                          const std::shared_ptr<rclcpp::Node> &node = nullptr)
{
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        if (node)
            RCLCPP_WARN(node->get_logger(), "Cannot open %s", csv_path.c_str());
        else
            std::cerr << "[WARN] Cannot open " << csv_path << std::endl;
        return;
    }

    std::string line;
    std::getline(ifs, line); // skip header

    Eigen::Quaterniond prev_q;
    bool first = true;
    double max_angle = 0.0;
    int idx = 0;

    while (std::getline(ifs, line)) {
        std::stringstream ss(line);
        std::vector<double> vals;
        std::string item;
        while (std::getline(ss, item, ',')) {
            try {
                vals.push_back(std::stod(item));
            } catch (...) {
                vals.push_back(0.0);
            }
        }

        if (vals.size() < 7) continue;
        // csv顺序: x,y,z,qx,qy,qz,qw,...
        Eigen::Quaterniond q(vals[6], vals[3], vals[4], vals[5]); // qw, qx, qy, qz
        q.normalize();

        if (!first) {
            double ang = angleBetweenQuaternions(prev_q, q);
            if (node)
                RCLCPP_INFO(node->get_logger(),
                            "Frame %d -> %d  Δθ = %.3f rad (%.1f°)",
                            idx - 1, idx, ang, ang * 180.0 / M_PI);
            else
                std::cout << "Frame " << idx - 1 << " -> " << idx
                          << "  Δθ = " << ang << " rad ("
                          << ang * 180.0 / M_PI << "°)" << std::endl;

            if (ang > max_angle) max_angle = ang;
        } else {
            first = false;
        }

        prev_q = q;
        ++idx;
    }

    if (node)
        RCLCPP_INFO(node->get_logger(),
                    "Max EE angular step = %.3f rad (%.1f°)",
                    max_angle, max_angle * 180.0 / M_PI);
    else
        std::cout << "Max EE angular step = " << max_angle
                  << " rad (" << max_angle * 180.0 / M_PI << "°)" << std::endl;

    ifs.close();
}
