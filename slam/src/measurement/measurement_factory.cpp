#include "measurement/measurement_factory.hpp"

MeasurementFactory::MeasurementFactory()
{
  // Create singleton models once
  _point3dModel = std::make_shared<Point3DMeasurementModel>(); //TODO, make MeasurementModel singleton and user getter later
  _bboxModel    = std::make_shared<BBoxMeasurementModel>();
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

  // attach singleton model
  m.model = _point3dModel;

  return m;
}

Measurement MeasurementFactory::buildBBox(const slam::Observation& obs)
{
  Measurement m;

  const slam::BoundingBox& b = std::get<slam::BoundingBox>(obs.payload);

  // bounding box measurement vector (x,y,w,h)
  m.payload = Eigen::VectorXd(4);
  m.payload << b.x, b.y, b.width, b.height;

  // attach singleton model
  m.model = _bboxModel;

  return m;
}
