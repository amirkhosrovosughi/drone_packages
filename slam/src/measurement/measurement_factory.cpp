#include "measurement/measurement_factory.hpp"

MeasurementFactory::MeasurementFactory()
{
  // Create singleton models once
  _point3dModel = std::make_shared<Point3DMeasurementModel>(); //TODO, make MeasurementModel singleton and user getter later
  _bboxModel    = std::make_shared<BBoxMeasurementModel>();
  _bearingModel = std::make_shared<BearingMeasurementModel>();
}

std::vector<Measurement> MeasurementFactory::build(
    const std::vector<slam::Observation>& observations)
{
  std::vector<Measurement> measurements;
  measurements.reserve(observations.size());

  for (const auto& obs : observations)
  {
    if (std::holds_alternative<slam::Point3D>(obs.payload))
    {
      measurements.push_back(buildPoint3D(obs));
    }
    else if (std::holds_alternative<slam::BoundingBox>(obs.payload))
    {
      measurements.push_back(buildBBox(obs));
    }
    else if (std::holds_alternative<slam::Bearing>(obs.payload))
    {
      measurements.push_back(buildBearing(obs));
    }
  }

  return measurements;
}

Measurement MeasurementFactory::buildPoint3D(const slam::Observation& obs)
{
  Measurement m;

  const slam::Point3D& p = std::get<slam::Point3D>(obs.payload);

  // measurement vector is simply the 3D position
  m.payload = Eigen::VectorXd(3);
  m.payload << p.position.x(), p.position.y(), p.position.z();

  // attach singleton model (snapshot under lock)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    m.model = _point3dModel;
  }

  return m;
}

Measurement MeasurementFactory::buildBBox(const slam::Observation& obs)
{
  Measurement m;

  // Support older BoundingBox type (if present) and new Bearing type.
  if (std::holds_alternative<slam::BoundingBox>(obs.payload)) {
    const slam::BoundingBox& b = std::get<slam::BoundingBox>(obs.payload);
    m.payload = Eigen::VectorXd(4);
    m.payload << b.x, b.y, b.width, b.height;
  }

  // attach singleton model (legacy bounding-box model)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    m.model = _bboxModel;
  }

  return m;
}

Measurement MeasurementFactory::buildBearing(const slam::Observation& obs)
{
  Measurement m;
  const slam::Bearing& br = std::get<slam::Bearing>(obs.payload);
  m.payload = Eigen::VectorXd(2);
  m.payload << br.yaw, br.pitch;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    m.model = _bearingModel;
  }
  return m;
}

void MeasurementFactory::setCameraInfo(const CameraInfo& info)
{
  // Create fresh models pre-configured with camera info and swap them in under lock.
  auto new_bbox = std::make_shared<BBoxMeasurementModel>();
  new_bbox->setCameraInfo(info);

  auto new_bearing = std::make_shared<BearingMeasurementModel>();
  new_bearing->setCameraInfo(info);

  std::lock_guard<std::mutex> lock(_mutex);
  _bboxModel = std::move(new_bbox);
  _bearingModel = std::move(new_bearing);
}
