#ifndef TWO_DIMENSION_MEASUREMENT__MODEL_HPP_
#define TWO_DIMENSION_MEASUREMENT__MODEL_HPP_

#include <Eigen/Dense>

#include "filter/models/measurement_model.hpp"
#include "def_slam.hpp"

class TwoDimensionMeasurementModel : public MeasurementModel {
public:
    PositionMeasurementModel() = default;
    virtual int getDimension();
    virtual void updateLandmark(const Measurements& meas); 
};

#endif  //TWO_DIMENSION_MEASUREMENT__MODEL_HPP_