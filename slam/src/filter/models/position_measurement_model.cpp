#include "filter/models/position_measurement_model.hpp"

static int MEASUREMENT_DIMENSION = 3;


int PositionMeasurementModel::getDimension()
{
    return MEASUREMENT_DIMENSION;
}

void PositionMeasurementModel::updateLandmark(const Measurements& meas)
{
    //TODO    
}


 /**
 * @brief obsevation model.
 *
 * Based on give postion of robot and postion of the landmark, return the expected values 3d feature observation
 *
 * @param robotPose robot pose
 * @param landmarkPosition landmark position
 * @param expectedMeasurement the expected values 3d feature detection
 * @return true if it is successful
 */
bool PositionMeasurementModel::directMeasurementModel(const Pose& robotPose, const Position& landmarkPosition, Position& expectedMeasurement)
{
    if (!_sensorTranformationLoaded)
    {
        return false;
    }

    //TODO
    // determine, out of range somehow -> return false if not

    return true;
}

void PositionMeasurementModel::setSensorTransformation(const Eigen::Matrix4d& sensorTranformation)
{
    _sensorTranformation = sensorTranformation;
    _sensorTranformationLoaded = true;
}

