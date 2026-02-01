#include "observation/observation_builder.hpp"

namespace slam
{

std::vector<Observation>
ObservationBuilder::fromPointList(const drone_msgs::msg::PointList& msg,
                                  double timeTag)
{
  std::vector<Observation> observations;
  observations.reserve(msg.points.size());

  int temp_id = 0;

  for (const auto& pt : msg.points)
  {
    Point3D point;
    point.position = Eigen::Vector3d(pt.x, pt.y, pt.z);

    Observation obs(timeTag, point);
    obs.id = temp_id++;  // temporary hint; association may overwrite

    observations.push_back(std::move(obs));
  }

  return observations;
}

std::vector<Observation>
ObservationBuilder::fromBoundingBoxes(
    const drone_msgs::msg::BoundingBoxes& msg,
    double timeTag)
{
  std::vector<Observation> observations;
  observations.reserve(msg.boxes.size());

  int temp_id = 0;

  for (const auto& box : msg.boxes)
  {
    BoundingBox bbox;
    bbox.x = box.x;
    bbox.y = box.y;
    bbox.width = box.width;
    bbox.height = box.height;
    bbox.confidence = box.probability;
    bbox.class_label = box.class_label;

    Observation obs(timeTag, bbox);
    obs.id = temp_id++;  // optional hint

    observations.push_back(std::move(obs));
  }

  return observations;
}

} // namespace slam
