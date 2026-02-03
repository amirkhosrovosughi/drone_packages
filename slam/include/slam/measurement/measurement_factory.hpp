#ifndef MEASUREMENT_FACTORY_HPP_
#define MEASUREMENT_FACTORY_HPP_

#include <vector>
#include <memory>
#include "observation/observation.hpp"
#include "measurement/measurement_model.hpp"
#include "measurement/point3d_measurement_model.hpp"
#include "measurement/bbox_measurement_model.hpp"

#include "common/def_slam.hpp"

class MeasurementFactory {
public:
  MeasurementFactory();
  ~MeasurementFactory() = default;

  void setCameraInfo(const CameraInfo& info);

  /**
   * @brief Build measurements from a vector of observations.
   */
  std::vector<Measurement> build(const std::vector<slam::Observation>& observations);

private:
  // Singleton models
  std::shared_ptr<Point3DMeasurementModel> _point3dModel;
  std::shared_ptr<BBoxMeasurementModel> _bboxModel;

  // Helper builders
  Measurement buildPoint3D(const slam::Observation& obs);
  Measurement buildBBox(const slam::Observation& obs);
};

#endif  // MEASUREMENT_FACTORY_HPP_
