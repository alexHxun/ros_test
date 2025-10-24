#pragma once
// rd_waypoints.hpp - pure helpers for RDP, no class members

#include <vector>
#include <algorithm>
#include <Eigen/Geometry>

namespace rd_utils {

inline void rdp_rec(const std::vector<Eigen::Vector3d> &pts, double eps, int first, int last, std::vector<int> &out_idx) {
  if (last <= first + 1) return;
  const Eigen::Vector3d A = pts[first];
  const Eigen::Vector3d B = pts[last];
  double max_dist = -1.0;
  int idx = -1;
  const Eigen::Vector3d AB = (B - A);
  const double ab_norm2 = AB.squaredNorm();
  for (int i = first + 1; i < last; ++i) {
    double dist;
    if (ab_norm2 == 0.0) {
      dist = (pts[i] - A).norm();
    } else {
      double t = ((pts[i] - A).dot(AB)) / ab_norm2;
      Eigen::Vector3d proj = A + t * AB;
      dist = (pts[i] - proj).norm();
    }
    if (dist > max_dist) { max_dist = dist; idx = i; }
  }
  if (max_dist > eps && idx != -1) {
    rdp_rec(pts, eps, first, idx, out_idx);
    out_idx.push_back(idx);
    rdp_rec(pts, eps, idx, last, out_idx);
  }
}

inline std::vector<int> rdp_indices(const std::vector<Eigen::Vector3d> &pts, double eps) {
  std::vector<int> idx;
  if (pts.empty()) return idx;
  idx.push_back(0);
  rdp_rec(pts, eps, 0, static_cast<int>(pts.size()) - 1, idx);
  idx.push_back(static_cast<int>(pts.size()) - 1);
  std::sort(idx.begin(), idx.end());
  idx.erase(std::unique(idx.begin(), idx.end()), idx.end());
  return idx;
}

} // namespace rd_utils
