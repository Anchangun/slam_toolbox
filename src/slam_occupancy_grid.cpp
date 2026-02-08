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
}

bool OccupancyGrid::isValid() { return GetWidth() > 0 && GetHeight() > 0; }

void OccupancyGrid::updateAllScans(
    const karto::LocalizedRangeScanVector& rScans) {
  if (rScans.empty()) {
    return;
  }

  kt_int32s width, height;
  karto::Vector2<kt_double> offset;
  this->ComputeDimensions(rScans, this->GetResolution(), width, height, offset);
  this->realloc(rScans, this->GetResolution());

  for (karto::LocalizedRangeScan* scan : rScans) {
    kt_int32s id = scan->GetUniqueId();
    karto::Pose2 current_pose = scan->GetSensorPose();

    auto it = traced_scans_.find(id);
    if (it != traced_scans_.end()) {
      if (!isSamePose(current_pose, it->second.scan_pose)) {
        this->updateScanPose(id, current_pose);
      }
    } else {  // new scan
      this->addScan(scan);
    }
  }

  this->draw();
}

void OccupancyGrid::draw() {
  if (!isValid()) {
    return;
  }

  this->Clear();
  m_pCellHitsCnt->Clear();
  m_pCellPassCnt->Clear();

  for (const auto& [uid, scan] : traced_scans_) {
    for (const auto& [world_coord, cnt] : scan.hit_cells) {
      karto::Vector2<kt_int32s> grid_coord = this->WorldToGrid(world_coord);

      if (grid_coord.GetX() >= 0 && grid_coord.GetX() < GetWidth() &&
          grid_coord.GetY() >= 0 && grid_coord.GetY() < GetHeight()) {
        kt_int32s index = this->GridIndex(grid_coord, false);
        m_pCellHitsCnt->GetDataPointer()[index] += cnt;
      }
    }

    for (const auto& [world_coord, cnt] : scan.pass_cells) {
      karto::Vector2<kt_int32s> grid_coord = this->WorldToGrid(world_coord);

      if (grid_coord.GetX() >= 0 && grid_coord.GetX() < GetWidth() &&
          grid_coord.GetY() >= 0 && grid_coord.GetY() < GetHeight()) {
        kt_int32s index = this->GridIndex(grid_coord, false);
        m_pCellPassCnt->GetDataPointer()[index] += cnt;
      }
    }
  }

  this->Update();
}

void OccupancyGrid::addScan(karto::LocalizedRangeScan* pScan) {
  karto::LaserRangeFinder* laserRangeFinder = pScan->GetLaserRangeFinder();
  kt_double rangeThreshold = laserRangeFinder->GetRangeThreshold();
  kt_double maxRange = laserRangeFinder->GetMaximumRange();
  kt_double minRange = laserRangeFinder->GetMinimumRange();

  karto::Vector2<kt_double> scanPosition = pScan->GetSensorPose().GetPosition();
  const karto::PointVectorDouble& rPointReadings =
      pScan->GetPointReadings(false);

  kt_int32s scan_id = pScan->GetUniqueId();
  RayTracedScan& traced_scan = traced_scans_[scan_id];
  traced_scan.scan_pose = pScan->GetSensorPose();

  int pointIndex = 0;
  const_forEachAs(karto::PointVectorDouble, &rPointReadings, pointsIter) {
    karto::Vector2<kt_double> point = *pointsIter;
    kt_double rangeReading = pScan->GetRangeReadings()[pointIndex];
    kt_bool isEndPointValid =
        rangeReading < (rangeThreshold - karto::KT_TOLERANCE);

    if (rangeReading <= minRange || rangeReading >= maxRange ||
        std::isnan(rangeReading)) {
      // ignore these readings
      pointIndex++;
      continue;
    } else if (rangeReading >= rangeThreshold) {
      // trace up to range reading
      kt_double ratio = rangeThreshold / rangeReading;
      kt_double dx = point.GetX() - scanPosition.GetX();
      kt_double dy = point.GetY() - scanPosition.GetY();
      point.SetX(scanPosition.GetX() + ratio * dx);
      point.SetY(scanPosition.GetY() + ratio * dy);
    }

    this->rayTrace(scanPosition, point, isEndPointValid, &traced_scan);

    pointIndex++;
  }
}

void OccupancyGrid::updateScanPose(kt_int32s scan_id,
                                   const karto::Pose2& new_pose) {
  auto iter = traced_scans_.find(scan_id);
  if (iter == traced_scans_.end()) {
    return;
  }

  RayTracedScan& traced_scan = iter->second;
  karto::Pose2 old_pose = traced_scan.scan_pose;

  kt_double dtheta = new_pose.GetHeading() - old_pose.GetHeading();
  kt_double cos_theta = std::cos(dtheta);
  kt_double sin_theta = std::sin(dtheta);

  std::unordered_map<karto::Vector2<kt_double>, kt_int32u> new_hit_cells;
  for (const auto& [world_point, count] : traced_scan.hit_cells) {
    kt_double rel_x = world_point.GetX() - old_pose.GetX();
    kt_double rel_y = world_point.GetY() - old_pose.GetY();

    kt_double rotated_x = rel_x * cos_theta - rel_y * sin_theta;
    kt_double rotated_y = rel_x * sin_theta + rel_y * cos_theta;

    kt_double new_world_x = new_pose.GetX() + rotated_x;
    kt_double new_world_y = new_pose.GetY() + rotated_y;

    new_hit_cells[{new_world_x, new_world_y}] = count;
  }

  std::unordered_map<karto::Vector2<kt_double>, kt_int32u> new_pass_cells;
  for (const auto& [world_point, count] : traced_scan.pass_cells) {
    kt_double rel_x = world_point.GetX() - old_pose.GetX();
    kt_double rel_y = world_point.GetY() - old_pose.GetY();

    kt_double rotated_x = rel_x * cos_theta - rel_y * sin_theta;
    kt_double rotated_y = rel_x * sin_theta + rel_y * cos_theta;

    kt_double new_world_x = new_pose.GetX() + rotated_x;
    kt_double new_world_y = new_pose.GetY() + rotated_y;

    new_pass_cells[{new_world_x, new_world_y}] = count;
  }

  traced_scan.hit_cells = std::move(new_hit_cells);
  traced_scan.pass_cells = std::move(new_pass_cells);
  traced_scan.scan_pose = new_pose;
}

void OccupancyGrid::rayTrace(const karto::Vector2<double>& rWorldFrom,
                             const karto::Vector2<double>& rWorldTo,
                             kt_bool isEndPointValid,
                             RayTracedScan* traced_scan) {
  karto::Vector2<kt_int32s> gridFrom = WorldToGrid(rWorldFrom);
  karto::Vector2<kt_int32s> gridTo = WorldToGrid(rWorldTo);

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

    if (x == x1 && isEndPointValid) {
      traced_scan->hit_cells[this->GridToWorld({pointX, pointY})]++;
    } else {
      traced_scan->pass_cells[this->GridToWorld({pointX, pointY})]++;
    }
  }
}

void OccupancyGrid::clearCache() { traced_scans_.clear(); }

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