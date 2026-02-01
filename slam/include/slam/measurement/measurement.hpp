// measurement.hpp
#pragma once

#include <memory>
#include <Eigen/Dense>

class MeasurementModel;  // forward declaration

struct Measurement {
  Eigen::VectorXd payload; // actual measurement vector
  std::shared_ptr<MeasurementModel> model; // math

  Measurement() = default;
};

struct AssignedMeasurement {
    Measurement measurement;
    int id;
    Position position;
    bool isNew;

    AssignedMeasurement() = default;
    AssignedMeasurement(const Measurement measurement, int id) {
        this->measurement = measurement;
        this->id = id;
        this->isNew = false;
    }
};


// Vector of Measurements
using Measurements = std::vector<Measurement>;
using AssignedMeasurements = std::vector<AssignedMeasurement>;
