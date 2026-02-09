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
#include <unordered_map>

#include "karto_sdk/Karto.h"

namespace mapper_utils {

struct GridCell
{
  int32_t x, y;
  kt_int32u count;

  GridCell() = delete;
  GridCell(kt_int32s x, kt_int32s y, kt_int32u count = 0)
  {
    this->x = x;
    this->y = y;
    this->count = count;
  }

  static GridCell fromWorld(kt_double wx, kt_double wy, kt_int32u count = 0)
  {
    kt_int32s x = static_cast<kt_int32s>(std::round(wx * 100));
    kt_int32s y = static_cast<kt_int32s>(std::round(wy * 100));

    GridCell cell(x, y, count);
    return cell;
  }

  void setFromWorld(const karto::Vector2<kt_double> & world_pos)
  {
    x = static_cast<kt_int32s>(std::round(world_pos.GetX() * 100));
    y = static_cast<kt_int32s>(std::round(world_pos.GetY() * 100));
  }

  karto::Vector2<kt_double> toWorld() const
  {
    return {x * 0.01, y * 0.01};
  }

  bool operator==(const GridCell& other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator<(const GridCell& other) const {
    if (x != other.x)
    {
      return x < other.x;
    }
    return y < other.y;
  }
};

struct RayTracedScan {
  karto::Pose2 scan_pose;
  std::vector<GridCell> hit_cells;
  std::vector<GridCell> pass_cells;
`
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
  void drawScanToGrid(const RayTracedScan & scan);
  void draw();
  void drawPartial(kt_int32s id);

  void addScan(karto::LocalizedRangeScan* pScan);
  void updateScanPose(kt_int32s scan_id, const karto::Pose2& new_pose);

  void rayTrace(const karto::Vector2<double>& rWorldFrom,
                const karto::Vector2<double>& rWorldTo, kt_bool isEndPointValid,
                RayTracedScan* traced_scan);

  void finalizeVector(std::vector<GridCell> & vec);

  bool isSamePose(karto::Pose2 p1, karto::Pose2 p2);
  void realloc(const karto::LocalizedRangeScanVector& rScans,
               kt_double resolution);

 private:
  std::map<kt_int32s, RayTracedScan> traced_scans_;
};

}  // namespace mapper_utils

#endif  // SLAM_TOOLBOX__SLAM_OCCUPANCY_GRID_HPP_
