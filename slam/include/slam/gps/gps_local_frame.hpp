#ifndef SLAM__GPS__GPS_LOCAL_FRAME_HPP_
#define SLAM__GPS__GPS_LOCAL_FRAME_HPP_

#include <stdexcept>

#include <Eigen/Dense>

#include <px4_msgs/msg/sensor_gps.hpp>

#include "common/def_slam_core.hpp"

class GpsLocalFrame
{
public:
  /**
   * @brief Return whether a local GPS anchor has been initialized.
   */
  bool hasAnchor() const;

  /**
   * @brief Set the local ENU anchor used for future GPS projections.
   */
  void setAnchor(const LocalFrameAnchor& anchor);

  /**
   * @brief Return the current local ENU anchor.
   */
  const LocalFrameAnchor& anchor() const;

  /**
   * @brief Convert a geodetic reference into anchor-relative ENU.
   */
  Eigen::Vector3d toEnu(const GpsReference& reference) const;

  /**
   * @brief Convert a GPS message into anchor-relative ENU.
   */
  Eigen::Vector3d toEnu(const px4_msgs::msg::SensorGps& msg) const;

private:
  bool _hasAnchor = false;
  LocalFrameAnchor _anchor;
};

#endif  // SLAM__GPS__GPS_LOCAL_FRAME_HPP_