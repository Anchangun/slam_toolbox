/*
 * toolbox_types
 * Copyright (c) 2019, Samsung Research America
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

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX__LASER_UTILS_HPP_
#define SLAM_TOOLBOX__LASER_UTILS_HPP_

#include <string>
#include <memory>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "slam_toolbox/toolbox_types.hpp"
#include "tf2/utils.h"
#include "karto_sdk/Karto.h"

namespace laser_utils
{

using batch_float = xsimd::batch<float>;
using batch_double = xsimd::batch<double>;

// Convert a laser scan to a vector of readings
inline karto::RangeReadingsVector scanToReadings(
  const sensor_msgs::msg::LaserScan & scan,
  const bool & inverted)
{
  const size_t n = scan.ranges.size();
  karto::RangeReadingsVector readings(n);

  if (inverted) {
    for (size_t i = 0; i < n; ++i) {
      readings[i] = static_cast<double>(scan.ranges[n - 1 - i]);
    }
  } else {
    constexpr size_t simd_size = batch_float::size;
    size_t i = 0;
    size_t vec_end = n - (n % simd_size);

    for (; i < vec_end; i += simd_size) {
      batch_float f_batch = batch_float::load_unaligned(&scan.ranges[i]);

      alignas(xsimd::default_arch::alignment()) float temp[simd_size];
      f_batch.store_aligned(temp);

      for (size_t j = 0; j < simd_size; ++j) {
        readings[i + j] = static_cast<double>(temp[j]);
      }
    }

    for (; i < n; ++i) {
      readings[i] = static_cast<double>(scan.ranges[i]);
    }
  }

  return readings;

  /*if (inverted) {
    for (std::vector<float>::const_reverse_iterator it = scan.ranges.rbegin();
      it != scan.ranges.rend(); ++it)
    {
      readings.push_back(*it);
    }
  } else {
    for (std::vector<float>::const_iterator it = scan.ranges.begin(); it != scan.ranges.end();
      ++it)
    {
      readings.push_back(*it);
    }
  }*/

  return readings;
}

// Store laser scanner information
class LaserMetadata
{
public:
  LaserMetadata();
  ~LaserMetadata();
  LaserMetadata(karto::LaserRangeFinder * lsr, bool invert);
  bool isInverted() const;
  karto::LaserRangeFinder * getLaser();
  void invertScan(sensor_msgs::msg::LaserScan & scan) const;

private:
  karto::LaserRangeFinder * laser;
  bool inverted;
};

// Help take a scan from a laser and create a laser object
class LaserAssistant
{
public:
  template<class NodeT>
  LaserAssistant(
    NodeT node, tf2_ros::Buffer * tf,
    const std::string & base_frame);
  ~LaserAssistant();
  LaserMetadata toLaserMetadata(sensor_msgs::msg::LaserScan scan);

private:
  karto::LaserRangeFinder * makeLaser(const double & mountingYaw);
  bool isInverted(double & mountingYaw);

  rclcpp::Logger logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
  tf2_ros::Buffer * tf_;
  sensor_msgs::msg::LaserScan scan_;
  std::string frame_, base_frame_;
  geometry_msgs::msg::TransformStamped laser_pose_;
};

// Hold some scans and utilities around them
class ScanHolder
{
public:
  explicit ScanHolder(std::map<std::string, laser_utils::LaserMetadata> & lasers);
  ~ScanHolder();
  sensor_msgs::msg::LaserScan getCorrectedScan(const int & id);
  void addScan(const sensor_msgs::msg::LaserScan scan);

private:
  std::unique_ptr<std::vector<sensor_msgs::msg::LaserScan>> current_scans_;
  std::map<std::string, laser_utils::LaserMetadata> & lasers_;
};

}  // namespace laser_utils

#endif  // SLAM_TOOLBOX__LASER_UTILS_HPP_
