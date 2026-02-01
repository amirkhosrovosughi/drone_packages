#include "observation/camera_bbox_observation.hpp"

CameraBBoxObservation::CameraBBoxObservation(int x_min_, int y_min_, int x_max_, int y_max_,
                                             int class_id_, float confidence_)
    : x_min(x_min_), y_min(y_min_), x_max(x_max_), y_max(y_max_),
      class_id(class_id_), confidence(confidence_) {}

int CameraBBoxObservation::width() const {
    return x_max - x_min;
}

int CameraBBoxObservation::height() const {
    return y_max - y_min;
}

Eigen::Vector2f CameraBBoxObservation::center() const {
    return Eigen::Vector2f(0.5f * (x_min + x_max), 0.5f * (y_min + y_max));
}
