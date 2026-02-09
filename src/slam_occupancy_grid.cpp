//
// Created by ing on 2/4/26.
//

#include "slam_toolbox/slam_occupancy_grid.hpp"

#include <oneapi/tbb/parallel_for.h>

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
  ComputeDimensions(rScans, this->GetResolution(), width, height, offset);

  bool should_full_draw = false;
  if (this->GetWidth() < width || this->GetHeight() < height)
  {
    Resize(width, height);
    should_full_draw = true;
  }

  this->GetCoordinateConverter()->SetOffset(offset);
  m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(offset);
  m_pCellPassCnt->GetCoordinateConverter()->SetOffset(offset);

  std::vector<kt_int32s> new_scans;

  for (karto::LocalizedRangeScan* scan : rScans) {
    kt_int32s id = scan->GetUniqueId();
    karto::Pose2 current_pose = scan->GetSensorPose();

    auto it = traced_scans_.find(id);
    if (it != traced_scans_.end()) {
      if (!isSamePose(current_pose, it->second.scan_pose)) {
        this->updateScanPose(id, current_pose);
        should_full_draw = true;
      }
    } else {  // new scan
      this->addScan(scan);
      new_scans.push_back(id);
    }
  }

  if (should_full_draw)
  {
    this->draw();
  } else
  {
    for (kt_int32s id : new_scans)
    {
      this->drawPartial(id);
    }
  }

  this->Update();
}

void OccupancyGrid::drawScanToGrid(const RayTracedScan& scan)
{
  auto* hit_data = m_pCellHitsCnt->GetDataPointer();
  auto* pass_data = m_pCellPassCnt->GetDataPointer();

  for (const GridCell & cell : scan.hit_cells)
  {
    karto::Vector2<kt_int32s> grid_pos = this->WorldToGrid(cell.toWorld());
    if (this->IsValidGridIndex(grid_pos))
    {
      hit_data[this->GridIndex(grid_pos, false)] += cell.count;
    }
  }

  for (const GridCell & cell : scan.pass_cells)
  {
    karto::Vector2<kt_int32s> grid_pos = this->WorldToGrid(cell.toWorld());
    if (this->IsValidGridIndex(grid_pos))
    {
      pass_data[this->GridIndex(grid_pos, false)] += cell.count;
    }
  }
}

void OccupancyGrid::draw() {
  if (!isValid()) return;

  this->Clear();
  m_pCellHitsCnt->Clear();
  m_pCellPassCnt->Clear();

  for (const auto& [uid, scan] : traced_scans_) {
    this->drawScanToGrid(scan);
  }
}

void OccupancyGrid::drawPartial(kt_int32s id)
{
  if (!isValid()) return;

  auto it = traced_scans_.find(id);
  if (it != traced_scans_.end()) {
    drawScanToGrid(it->second);
  }
}

void OccupancyGrid::updateScanPose(kt_int32s scan_id, const karto::Pose2& new_pose) {
  auto iter = traced_scans_.find(scan_id);
  if (iter == traced_scans_.end()) {
    return;
  }

  RayTracedScan& traced_scan = iter->second;
  karto::Pose2 old_pose = traced_scan.scan_pose;

  kt_double dtheta = new_pose.GetHeading() - old_pose.GetHeading();
  kt_double cos_theta = std::cos(dtheta);
  kt_double sin_theta = std::sin(dtheta);

  auto transform_cells = [&](std::vector<GridCell>& cells)
  {
    for (GridCell & cell : cells)
    {
      karto::Vector2<kt_double> w_pos = cell.toWorld();

      kt_double rel_x = w_pos.GetX() - old_pose.GetX();
      kt_double rel_y = w_pos.GetY() - old_pose.GetY();

      kt_double rotated_x = rel_x * cos_theta - rel_y * sin_theta;
      kt_double rotated_y = rel_x * sin_theta + rel_y * cos_theta;

      kt_double new_world_x = new_pose.GetX() + rotated_x;
      kt_double new_world_y = new_pose.GetY() + rotated_y;

      cell.setFromWorld({new_world_x, new_world_y});
    }
  };

  transform_cells(traced_scan.hit_cells);
  transform_cells(traced_scan.pass_cells);

  traced_scan.scan_pose = new_pose;
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

  this->finalizeVector(traced_scan.hit_cells);
  this->finalizeVector(traced_scan.pass_cells);
}

void OccupancyGrid::rayTrace(const karto::Vector2<double>& rWorldFrom,
                             const karto::Vector2<double>& rWorldTo,
                             kt_bool isEndPointValid,
                             RayTracedScan* traced_scan) {
  karto::Vector2<kt_int32s> gridFrom = WorldToGrid(rWorldFrom);
  karto::Vector2<kt_int32s> gridTo = WorldToGrid(rWorldTo);

  kt_int32s real_end_x = gridTo.GetX();
  kt_int32s real_end_y = gridTo.GetY();

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

  for (kt_int32s x = x0; x <= x1; x++) {
    kt_int32s px = steep ? y : x;
    kt_int32s py = steep ? x : y;

    karto::Vector2<kt_double> world_pos = this->GridToWorld({px, py});
    GridCell cell = GridCell::fromWorld(world_pos.GetX(), world_pos.GetY(), 1);

    if (px == real_end_x && py == real_end_y && isEndPointValid) {
      traced_scan->hit_cells.push_back(cell);
    } else {
      traced_scan->pass_cells.push_back(cell);
    }

    error -= deltaY;
    if (error < 0) {
      y += ystep;
      error += deltaX;
    }
  }
}

void OccupancyGrid::finalizeVector(std::vector<GridCell>& vec)
{
  if (vec.empty())
  {
    return;
  }

  std::sort(vec.begin(), vec.end());

  auto it = vec.begin();
  auto write_it = vec.begin();

  while (++it != vec.end())
  {
    if (*it == *write_it)
    {
      write_it->count += it->count;
    } else
    {
      *(++write_it) = *it;
    }
  }

  vec.erase(++write_it, vec.end());
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

  this->Resize(width, height);

  this->GetCoordinateConverter()->SetScale(1.0 / resolution);
  this->GetCoordinateConverter()->SetOffset(offset);

  m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(offset);
  m_pCellPassCnt->GetCoordinateConverter()->SetOffset(offset);

}

}  // namespace mapper_utils