#ifndef SLAM__FAST_SLAM_HPP_
#define SLAM__FAST_SLAM_HPP_

#include "base_filter.hpp"

/**
 * @brief Implementation of FastSLAM filter. TODO
 */
class FastSlam : public BaseFilter {
public:
    FastSlam();
    void prediction(const OdometryInfo& odom) override;
    void correction(const Measurements& meas) override;
    void registerCallback(std::function<void(const MapSummary& map)> callback) override;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) override {}
    void setLogger(LoggerPtr logger) override {}

private:
    std::function<void(const MapSummary& map)> _callback;
};

#endif  // SLAM__FAST_SLAM_HPP_