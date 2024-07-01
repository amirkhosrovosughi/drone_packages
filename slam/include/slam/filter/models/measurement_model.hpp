#ifndef MEASUREMENT__MODEL_HPP_
#define MEASUREMENT__MODEL_HPP_

#include <Eigen/Dense>
#include "def_slam.hpp"

class MeasurementModel {
public:
    virtual ~MeasurementModel() = default;
    virtual int getDimension() = 0;
    virtual void updateLandmark(const Measurements& meas) = 0;   
};

#endif  // MEASUREMENT__MODEL_HPP_