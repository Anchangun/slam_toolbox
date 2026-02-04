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

#include "karto_sdk/Karto.h"

namespace mapper_utils
{
  class OccupancyGrid : public karto::OccupancyGrid
  {
  public:
    OccupancyGrid(kt_int32s width = 0, kt_int32s height = 0,
                  const karto::Vector2<kt_double> &rOffset = {0, 0},
                  kt_double resolution = 0.05);

    void init(const karto::LocalizedRangeScanVector &rScans, kt_double resolution,
              kt_int32u min_pass_through, kt_double occupancy_threshold);

    bool isValid();

    void updateAllScans(const karto::LocalizedRangeScanVector &rScans);

  protected:
    void CreateFromScans(const karto::LocalizedRangeScanVector &rScans) override;

    bool isSamePose(karto::Pose2 p1, karto::Pose2 p2);
    void realloc(const karto::LocalizedRangeScanVector &rScans, kt_double resolution);

  private:
    std::map<kt_int32s, karto::Pose2> prev_scan_poses_;
  };
} // namespace mapper_utils

#endif   // SLAM_TOOLBOX__SLAM_OCCUPANCY_GRID_HPP_
