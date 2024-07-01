#ifndef POSITION_MEASUREMENT__MODEL_HPP_
#define POSITION_MEASUREMENT__MODEL_HPP_

#include <Eigen/Dense>

#include "filter/models/measurement_model.hpp"
#include "def_slam.hpp"

class PositionMeasurementModel : public MeasurementModel {
public:
    PositionMeasurementModel() = default;
    virtual int getDimension();
    virtual void updateLandmark(const Measurements& meas);
    void setSensorTransformation(const Eigen::Matrix4d& sensorTranformation);
    bool directMeasurementModel(const Pose& robotPose, const Position& landmarkPosition, Position& expectedMeasurement);

private:
    Eigen::Matrix4d _sensorTranformation;
    bool _sensorTranformationLoaded = false;

};

#endif  // POSITION_MEASUREMENT__MODEL_HPP_