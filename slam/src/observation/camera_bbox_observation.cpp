#include "observation/camera_bbox_observation.hpp"

CameraBBoxObservation::CameraBBoxObservation(int xMin_, int yMin_, int xMax_, int yMax_,
                                             int classId_, float confidence_)
    : xMin(xMin_), yMin(yMin_), xMax(xMax_), yMax(yMax_),
      classId(classId_), confidence(confidence_) {}

int CameraBBoxObservation::width() const {
    return xMax - xMin;
}

int CameraBBoxObservation::height() const {
    return yMax - yMin;
}

Eigen::Vector2f CameraBBoxObservation::center() const {
    return Eigen::Vector2f(0.5f * (xMin + xMax), 0.5f * (yMin + yMax));
}
