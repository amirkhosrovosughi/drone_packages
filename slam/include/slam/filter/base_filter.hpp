#ifndef SLAM__FILTER_HPP_
#define SLAM__FILTER_HPP_

#include <functional>
#include <Eigen/Dense>
#include "def_slam.hpp"
#include "slam_logger.hpp"

class BaseFilter {
public:
    virtual ~BaseFilter() = default;
    virtual void prediction(const OdometryInfo& odom) = 0;
    virtual void correction(const Measurements& meas) = 0;
    virtual void registerCallback(std::function<void(const MapSummary& map)> callback) = 0;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;
    virtual void setLogger(LoggerPtr logger) = 0;
};

#endif  // SLAM__FILTER_HPP_