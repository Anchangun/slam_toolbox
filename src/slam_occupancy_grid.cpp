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

    bool is_map_geometry_changed = prev_width != width || prev_height != height;
    bool is_loop_closed = false;

    int changed_pose_count = 0;
    karto::LocalizedRangeScanVector process_scans;

    for (karto::LocalizedRangeScan *scan: rScans) {
      kt_int32s id = scan->GetUniqueId();
      karto::Pose2 current_pose = scan->GetSensorPose();

      auto it = prev_scan_poses_.find(id);
      if (it != prev_scan_poses_.end()) {
        if (!isSamePose(current_pose, it->second)) {
          is_loop_closed = true;
          changed_pose_count++;

          it->second = current_pose;
        }
      } else {
        process_scans.push_back(scan);
        prev_scan_poses_[id] = current_pose;
      }
    }

    if (is_map_geometry_changed) {
      realloc(rScans, GetResolution());
      process_scans = rScans;
      std::cout << width << " x " << height << std::endl;
    } else if (is_loop_closed) {
      Clear();
      process_scans = rScans;
      std::cout << "[Loop Closed] Pose changed: " << changed_pose_count
                    << " / " << rScans.size() << std::endl;
    }

    CreateFromScans(process_scans);
  }

  void OccupancyGrid::CreateFromScans(const karto::LocalizedRangeScanVector &rScans) {
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    const_forEach(karto::LocalizedRangeScanVector, &rScans)
    {
      if (*iter == nullptr) {
        continue;
      }

      karto::LocalizedRangeScan * pScan = *iter;
      AddScan(pScan);
    }

    Update();
  }

  bool OccupancyGrid::isSamePose(karto::Pose2 p1, karto::Pose2 p2) {
    const double pos_eps = 0.001;
    const double ang_eps = 0.01;

    return (std::abs(p1.GetX() - p2.GetX()) < pos_eps &&
            std::abs(p1.GetY() - p2.GetY()) < pos_eps &&
            std::abs(karto::math::NormalizeAngle(p1.GetHeading() - p2.GetHeading())) < ang_eps);
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
