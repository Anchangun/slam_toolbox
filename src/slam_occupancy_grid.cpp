//
// Created by ing on 2/4/26.
//

#include "slam_toolbox/slam_occupancy_grid.hpp"

namespace mapper_utils
{
  OccupancyGrid::OccupancyGrid(kt_int32s width, kt_int32s height,
                               const karto::Vector2<kt_double> &rOffset, kt_double resolution)
    : karto::OccupancyGrid(width, height, rOffset, resolution) {
  }

  void OccupancyGrid::init(const karto::LocalizedRangeScanVector &rScans, kt_double resolution,
                           kt_int32u min_pass_through, kt_double occupancy_threshold) {
    realloc(rScans, resolution);

    SetMinPassThrough(min_pass_through);
    SetOccupancyThreshold(occupancy_threshold);

    CreateFromScans(rScans);
  }

  bool OccupancyGrid::isValid() {
    return GetWidth() > 0 && GetHeight() > 0;
  }

  void OccupancyGrid::updateAllScans(const karto::LocalizedRangeScanVector &rScans) {
    if (rScans.empty()) {
      return;
    }

    kt_int32s prev_width = GetWidth();
    kt_int32s prev_height = GetHeight();

    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(rScans, this->GetResolution(), width, height, offset);

    bool is_map_size_increased = prev_width != width || prev_height != height;
    bool is_loop_closed = false;
    karto::LocalizedRangeScanVector process_scans;

    for (karto::LocalizedRangeScan *scan: rScans) {
      kt_int32s id = scan->GetUniqueId();
      karto::Pose2 current_pose = scan->GetSensorPose();

      auto it = prev_scan_poses_.find(id);
      if (it != prev_scan_poses_.end()) {
        karto::Pose2 &old_pose = it->second;

        if (std::abs(current_pose.GetX() - old_pose.GetX()) > 0.01 ||
            std::abs(current_pose.GetY() - old_pose.GetY()) > 0.01 ||
            std::abs(current_pose.GetHeading() - old_pose.GetHeading()) > 0.01) {
          is_loop_closed = true;
        }
      } else {
        process_scans.push_back(scan);
      }
    }

    if (is_map_size_increased) {
      Clear();
      realloc(rScans, GetResolution());

      process_scans = rScans;

      std::cout << width << " x " << height << std::endl;
    } else if (is_loop_closed) {
      Clear();

      process_scans = rScans;

      std::cout << "Loop Closed" << std::endl;
    }

    CreateFromScans(process_scans);
  }

  void OccupancyGrid::realloc(const karto::LocalizedRangeScanVector &rScans, kt_double resolution) {
    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(offset);
    Resize(width, height);
  }
}
