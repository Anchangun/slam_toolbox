//
// Created by ing on 2/4/26.
//

#include "slam_toolbox/slam_occupancy_grid.hpp"

namespace mapper_utils
{
  OccupancyGrid::OccupancyGrid(kt_int32s width, kt_int32s height,
                               const karto::Vector2<kt_double>& rOffset, kt_double resolution)
    : karto::OccupancyGrid(width, height, rOffset, resolution)
  {
  }

  void OccupancyGrid::init(const karto::LocalizedRangeScanVector& rScans, kt_double resolution,
                           kt_int32u min_pass_through, kt_double occupancy_threshold)
  {
    realloc(rScans, resolution);

    SetMinPassThrough(min_pass_through);
    SetOccupancyThreshold(occupancy_threshold);

    CreateFromScans(rScans);
  }

  bool OccupancyGrid::isValid()
  {
    return GetWidth() > 0 && GetHeight() > 0;
  }

  void OccupancyGrid::updateAllScans(const karto::LocalizedRangeScanVector& rScans)
  {
    if (rScans.empty())
    {
      return;
    }

    kt_int32s prev_width = GetWidth();
    kt_int32s prev_height = GetHeight();

    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(rScans, this->GetResolution(), width, height, offset);

    bool is_map_geometry_changed = prev_width != width || prev_height != height;
    bool is_loop_closed = false;

    karto::LocalizedRangeScanVector process_scans;

    for (karto::LocalizedRangeScan* scan : rScans)
    {
      kt_int32s id = scan->GetUniqueId();
      karto::Pose2 current_pose = scan->GetSensorPose();

      auto it = scan_poses_.find(id);
      if (it != scan_poses_.end())
      {
        if (!isSamePose(current_pose, it->second))
        {
          is_loop_closed = true;
          it->second = current_pose;
        }
      }
      else
      {
        process_scans.push_back(scan);
        scan_poses_[id] = current_pose;
      }
    }

    if (is_map_geometry_changed)
    {
      realloc(rScans, GetResolution());
      process_scans = rScans;
    }
    else if (is_loop_closed)
    {
      Clear();
      process_scans = rScans;
    }

    CreateFromScans(process_scans);
  }

  void OccupancyGrid::CreateFromScans(const karto::LocalizedRangeScanVector& rScans)
  {
    m_pCellPassCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());
    m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

    const_forEach(karto::LocalizedRangeScanVector, &rScans)
    {
      if (*iter == nullptr)
      {
        continue;
      }

      karto::LocalizedRangeScan* pScan = *iter;
      AddScan(pScan);
    }

    Update();
  }

  kt_bool OccupancyGrid::AddScan(karto::LocalizedRangeScan* pScan, kt_bool doUpdate)
  {
    karto::LaserRangeFinder* laserRangeFinder = pScan->GetLaserRangeFinder();
    kt_double rangeThreshold = laserRangeFinder->GetRangeThreshold();
    kt_double maxRange = laserRangeFinder->GetMaximumRange();
    kt_double minRange = laserRangeFinder->GetMinimumRange();

    karto::Vector2<kt_double> scanPosition = pScan->GetSensorPose().GetPosition();
    // get scan point readings
    const karto::PointVectorDouble& rPointReadings = pScan->GetPointReadings(false);

    kt_bool isAllInMap = true;

    // draw lines from scan position to all point readings
    int pointIndex = 0;
    const_forEachAs(karto::PointVectorDouble, &rPointReadings, pointsIter)
    {
      karto::Vector2<kt_double> point = *pointsIter;
      kt_double rangeReading = pScan->GetRangeReadings()[pointIndex];
      kt_bool isEndPointValid = rangeReading < (rangeThreshold - karto::KT_TOLERANCE);

      if (rangeReading <= minRange || rangeReading >= maxRange || std::isnan(rangeReading))
      {
        // ignore these readings
        pointIndex++;
        continue;
      }
      else if (rangeReading >= rangeThreshold)
      {
        // trace up to range reading
        kt_double ratio = rangeThreshold / rangeReading;
        kt_double dx = point.GetX() - scanPosition.GetX();
        kt_double dy = point.GetY() - scanPosition.GetY();
        point.SetX(scanPosition.GetX() + ratio * dx);
        point.SetY(scanPosition.GetY() + ratio * dy);
      }

      kt_bool isInMap = RayTrace(scanPosition, point, isEndPointValid, doUpdate);
      if (!isInMap)
      {
        isAllInMap = false;
      }

      pointIndex++;
    }

    return isAllInMap;
  }

  Cell OccupancyGrid::RayTracing(const karto::Vector2<double>& rWorldFrom,
                                 const karto::Vector2<double>& rWorldTo,
                                 kt_bool isEndPointValid,
                                 kt_bool doUpdate)
  {
    karto::Vector2<kt_int32s> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
    karto::Vector2<kt_int32s> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);


  }

  bool OccupancyGrid::isSamePose(karto::Pose2 p1, karto::Pose2 p2)
  {
    return p1.GetX() == p2.GetX() && p1.GetY() == p2.GetY() &&
      p1.GetHeading() == p2.GetHeading();
  }

  void OccupancyGrid::realloc(const karto::LocalizedRangeScanVector& rScans, kt_double resolution)
  {
    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(rScans, resolution, width, height, offset);

    GetCoordinateConverter()->SetScale(1.0 / resolution);
    GetCoordinateConverter()->SetOffset(offset);
    Resize(width, height);
  }
}
