#ifndef MEASUREMENT__MODEL_HPP_
#define MEASUREMENT__MODEL_HPP_

#include <Eigen/Dense>
#include "def_slam.hpp"

class MeasurementModel {
public:
    virtual ~MeasurementModel() = default;
    virtual int getDimension() = 0;
    virtual void updateLandmark(const Measurements& meas) = 0;
    virtual void setSensorInfo(const Eigen::Matrix4d& transform) = 0;
    virtual bool directMeasurementModel(const Pose& robotPose, const Position& landmarkPosition, Measurement& expectedMeasurement) = 0;
};

#endif  // MEASUREMENT__MODEL_HPP_