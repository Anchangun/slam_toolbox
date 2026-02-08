//
// Created by ing on 2/4/26.
//

#include "slam_toolbox/slam_occupancy_grid.hpp"

namespace mapper_utils {
OccupancyGrid::OccupancyGrid(kt_int32s width, kt_int32s height,
                             const karto::Vector2<kt_double>& rOffset,
                             kt_double resolution)
    : karto::OccupancyGrid(width, height, rOffset, resolution) {}

void OccupancyGrid::init(const karto::LocalizedRangeScanVector& rScans,
                         kt_double resolution, kt_int32u min_pass_through,
                         kt_double occupancy_threshold) {
  realloc(rScans, resolution);

  SetMinPassThrough(min_pass_through);
  SetOccupancyThreshold(occupancy_threshold);

  CreateFromScans(rScans);
}

bool OccupancyGrid::isValid() { return GetWidth() > 0 && GetHeight() > 0; }

void OccupancyGrid::updateAllScans(
    const karto::LocalizedRangeScanVector& rScans) {
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

  karto::LocalizedRangeScanVector process_scans;

  for (karto::LocalizedRangeScan* scan : rScans) {
    kt_int32s id = scan->GetUniqueId();
    karto::Pose2 current_pose = scan->GetSensorPose();

    auto it = traced_scans_.find(id);
    if (it != traced_scans_.end()) {
      if (!isSamePose(current_pose, it->second.scan_pose)) {
        is_loop_closed = true;
        it->second.scan_pose = current_pose;
      }
    } else {
      process_scans.push_back(scan);
    }
  }

  if (is_map_geometry_changed) {
    realloc(rScans, GetResolution());
    Clear();
    applyAllCachedScans();
  } else if (is_loop_closed) {
    Clear();
    applyAllCachedScans();
  } else {
    CreateFromScans(process_scans);
  }
}

void OccupancyGrid::CreateFromScans(
    const karto::LocalizedRangeScanVector& rScans) {
  m_pCellPassCnt->GetCoordinateConverter()->SetOffset(
      GetCoordinateConverter()->GetOffset());
  m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(
      GetCoordinateConverter()->GetOffset());

  const_forEach(karto::LocalizedRangeScanVector, &rScans) {
    if (*iter == nullptr) {
      continue;
    }

    karto::LocalizedRangeScan* pScan = *iter;
    AddScan(pScan);
  }

  Update();
}

kt_bool OccupancyGrid::AddScan(karto::LocalizedRangeScan* pScan,
                               kt_bool doUpdate) {
  kt_int32s scan_id = pScan->GetUniqueId();

  // 캐시 확인
  auto it = traced_scans_.find(scan_id);
  if (it != traced_scans_.end()) {
    // 캐시된 결과 재사용
    applyCachedScan(&it->second);
    return true;
  }

  // 캐시 미스 - 새로운 레이 트레이싱 수행
  karto::LaserRangeFinder* laserRangeFinder = pScan->GetLaserRangeFinder();
  kt_double rangeThreshold = laserRangeFinder->GetRangeThreshold();
  kt_double maxRange = laserRangeFinder->GetMaximumRange();
  kt_double minRange = laserRangeFinder->GetMinimumRange();

  karto::Vector2<kt_double> scanPosition = pScan->GetSensorPose().GetPosition();
  const karto::PointVectorDouble& rPointReadings =
      pScan->GetPointReadings(false);

  // 새로운 RayTracedScan 생성
  RayTracedScan new_traced_scan;
  new_traced_scan.scan_pose = pScan->GetSensorPose();

  kt_bool isAllInMap = true;

  int pointIndex = 0;
  const_forEachAs(karto::PointVectorDouble, &rPointReadings, pointsIter) {
    karto::Vector2<kt_double> point = *pointsIter;
    kt_double rangeReading = pScan->GetRangeReadings()[pointIndex];
    kt_bool isEndPointValid =
        rangeReading < (rangeThreshold - karto::KT_TOLERANCE);

    if (rangeReading <= minRange || rangeReading >= maxRange ||
        std::isnan(rangeReading)) {
      pointIndex++;
      continue;
    } else if (rangeReading >= rangeThreshold) {
      kt_double ratio = rangeThreshold / rangeReading;
      kt_double dx = point.GetX() - scanPosition.GetX();
      kt_double dy = point.GetY() - scanPosition.GetY();
      point.SetX(scanPosition.GetX() + ratio * dx);
      point.SetY(scanPosition.GetY() + ratio * dy);
    }

    kt_bool isInMap = rayTrace(scanPosition, point, isEndPointValid,
                                doUpdate, &new_traced_scan);
    if (!isInMap) {
      isAllInMap = false;
    }

    pointIndex++;
  }

  // 캐시에 저장
  traced_scans_[scan_id] = new_traced_scan;

  return isAllInMap;
}

kt_bool OccupancyGrid::rayTrace(const karto::Vector2<double>& rWorldFrom,
                                const karto::Vector2<double>& rWorldTo,
                                kt_bool isEndPointValid,
                                kt_bool doUpdate,
                                RayTracedScan* traced_scan) {
  karto::Vector2<kt_int32s> gridFrom = m_pCellPassCnt->WorldToGrid(rWorldFrom);
  karto::Vector2<kt_int32s> gridTo = m_pCellPassCnt->WorldToGrid(rWorldTo);

  kt_int32s x0 = gridFrom.GetX();
  kt_int32s y0 = gridFrom.GetY();
  kt_int32s x1 = gridTo.GetX();
  kt_int32s y1 = gridTo.GetY();

  kt_bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  kt_int32s deltaX = x1 - x0;
  kt_int32s deltaY = abs(y1 - y0);
  kt_int32s error = 0;
  kt_int32s ystep;
  kt_int32s y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  kt_int32s pointX;
  kt_int32s pointY;
  kt_bool isInMap = true;

  for (kt_int32s x = x0; x <= x1; x++) {
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }

    kt_bool isEndPoint = (x == x1);

    if (isEndPoint) {
      if (isEndPointValid) {
        // Hit cell
        if (traced_scan != nullptr) {
          Cell cell;
          cell.x = pointX;
          cell.y = pointY;
          cell.count = 0;
          traced_scan->hit_cells.insert(cell);
        }

        if (doUpdate) {
          kt_int32s index = m_pCellHitsCnt->GridIndex(
              karto::Vector2<kt_int32s>(pointX, pointY), false);
          if (index != -1) {
            m_pCellHitsCnt->GetDataPointer()[index]++;
          } else {
            isInMap = false;
          }
        }
      }
    } else {
      // Pass through cell
      if (traced_scan != nullptr) {
        Cell cell;
        cell.x = pointX;
        cell.y = pointY;
        cell.count = 0;
        traced_scan->pass_cells.insert(cell);
      }

      if (doUpdate) {
        kt_int32s index = m_pCellPassCnt->GridIndex(
            karto::Vector2<kt_int32s>(pointX, pointY), false);
        if (index != -1) {
          m_pCellPassCnt->GetDataPointer()[index]++;
        } else {
          isInMap = false;
        }
      }
    }
  }

  return isInMap;
}

void OccupancyGrid::applyCachedScan(const RayTracedScan* cached_scan) {
  if (cached_scan == nullptr) {
    return;
  }

  // Hit cells 적용
  for (const auto& cell : cached_scan->hit_cells) {
    kt_int32s index = m_pCellHitsCnt->GridIndex(
        karto::Vector2<kt_int32s>(cell.x, cell.y), false);
    if (index != -1) {
      m_pCellHitsCnt->GetDataPointer()[index]++;
    }
  }

  // Pass cells 적용
  for (const auto& cell : cached_scan->pass_cells) {
    kt_int32s index = m_pCellPassCnt->GridIndex(
        karto::Vector2<kt_int32s>(cell.x, cell.y), false);
    if (index != -1) {
      m_pCellPassCnt->GetDataPointer()[index]++;
    }
  }
}

void OccupancyGrid::applyAllCachedScans() {
  m_pCellPassCnt->GetCoordinateConverter()->SetOffset(
      GetCoordinateConverter()->GetOffset());
  m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(
      GetCoordinateConverter()->GetOffset());

  // 모든 캐시된 스캔 재적용
  for (auto& [scan_id, traced_scan] : traced_scans_) {
    applyCachedScan(&traced_scan);
  }

  Update();
}

void OccupancyGrid::clearCache() {
  traced_scans_.clear();
}

bool OccupancyGrid::isSamePose(karto::Pose2 p1, karto::Pose2 p2) {
  return p1.GetX() == p2.GetX() && p1.GetY() == p2.GetY() &&
         p1.GetHeading() == p2.GetHeading();
}

void OccupancyGrid::realloc(const karto::LocalizedRangeScanVector& rScans,
                            kt_double resolution) {
  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  ComputeDimensions(rScans, resolution, width, height, offset);

  GetCoordinateConverter()->SetScale(1.0 / resolution);
  GetCoordinateConverter()->SetOffset(offset);
  Resize(width, height);
}

}  // namespace mapper_utils