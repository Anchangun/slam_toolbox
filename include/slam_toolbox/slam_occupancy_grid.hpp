/*
 * Author
 * Copyright (c) 2019 Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: NTQues */

#ifndef SLAM_TOOLBOX__SLAM_OCCUPANCY_GRID_HPP_
#define SLAM_TOOLBOX__SLAM_OCCUPANCY_GRID_HPP_

#include <rclcpp/macros.hpp>
#include <unordered_set>

#include "karto_sdk/Karto.h"

namespace std {
template <>
struct hash<karto::Vector2<kt_double>> {
  std::size_t operator()(const karto::Vector2<kt_double>& v) const noexcept {
    size_t h1 = std::hash<double>{}(std::round(v.GetX() * 100.0) * 0.01);
    size_t h2 = std::hash<double>{}(std::round(v.GetY() * 100.0) * 0.01);
    return h1 ^ (h2 << 1);
  }
};
}  // namespace std

namespace mapper_utils {

struct RayTracedScan {
  karto::Pose2 scan_pose;
  std::unordered_map<karto::Vector2<kt_double>, kt_int32u> hit_cells;
  std::unordered_map<karto::Vector2<kt_double>, kt_int32u> pass_cells;

  RayTracedScan() = default;
};

class OccupancyGrid : public karto::OccupancyGrid {
 public:
  RCLCPP_SMART_PTR_DEFINITIONS(mapper_utils::OccupancyGrid)

  OccupancyGrid(kt_int32s width = 0, kt_int32s height = 0,
                const karto::Vector2<kt_double>& rOffset = {0, 0},
                kt_double resolution = 0.05);

  void init(const karto::LocalizedRangeScanVector& rScans, kt_double resolution,
            kt_int32u min_pass_through, kt_double occupancy_threshold);

  bool isValid();

  void updateAllScans(const karto::LocalizedRangeScanVector& rScans);

 protected:
  void draw();

  void addScan(karto::LocalizedRangeScan* pScan);
  void updateScanPose(kt_int32s scan_id, const karto::Pose2& new_pose);

  void rayTrace(const karto::Vector2<double>& rWorldFrom,
                const karto::Vector2<double>& rWorldTo, kt_bool isEndPointValid,
                RayTracedScan* traced_scan);

  void clearCache();

  bool isSamePose(karto::Pose2 p1, karto::Pose2 p2);
  void realloc(const karto::LocalizedRangeScanVector& rScans,
               kt_double resolution);

 private:
  std::map<kt_int32s, RayTracedScan> traced_scans_;
};

}  // namespace mapper_utils

#endif  // SLAM_TOOLBOX__SLAM_OCCUPANCY_GRID_HPP_
