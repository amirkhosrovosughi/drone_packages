#include "filter/models/two_dimension_measurement_model.hpp"

static int MEASUREMENT_DIMENSION = 3;


int TwoDimensionMeasurementModel::getDimension()
{
    return MEASUREMENT_DIMENSION;
}

void TwoDimensionMeasurementModel::updateLandmark(const Measurements& meas) //need to define a a different/abstract measurement type for it in def_slam.hpp
{
    //TODO  
}

