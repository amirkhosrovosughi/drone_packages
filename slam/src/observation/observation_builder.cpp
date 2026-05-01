// Observation builder: converts raw sensor messages into Observation objects
#include "observation/observation_builder.hpp"
#include <cmath>
#include <optional>

namespace slam
{

std::vector<Observation>
ObservationBuilder::fromPointList(const drone_msgs::msg::PointList& msg,
                                  double timeTag)
{
  std::vector<Observation> observations;
  observations.reserve(msg.points.size());

  int tempId = 0;

  for (const auto& pt : msg.points)
  {
    Point3D point;
    point.position = Eigen::Vector3d(pt.x, pt.y, pt.z);

    Observation obs(timeTag, point);
    obs.id = tempId++;  // temporary hint; association may overwrite

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

  int tempId = 0;

  for (const auto& box : msg.boxes)
  {
    BoundingBox bbox;
    bbox.x = box.x;
    bbox.y = box.y;
    bbox.width = box.width;
    bbox.height = box.height;
    bbox.confidence = box.probability;
    bbox.classLabel = box.class_label;

    Observation obs(timeTag, bbox);
    obs.id = tempId++;  // optional hint

    observations.push_back(std::move(obs));
  }

  return observations;
}

// Definition of static member declared in header
std::optional<CameraInfo> ObservationBuilder::sCameraInfo = std::nullopt;

void ObservationBuilder::setCameraInfo(const CameraInfo& info)
{
  ObservationBuilder::sCameraInfo = info;
}

std::vector<Observation>
ObservationBuilder::fromBboxArray(
    const vision_msgs::msg::Detection3DArray& msg,
    double timeTag)
{
  std::vector<Observation> observations;
  if (!ObservationBuilder::sCameraInfo.has_value())
    return observations;

  const auto& cam = ObservationBuilder::sCameraInfo.value();
  observations.reserve(msg.detections.size());

  int tempId = 0;

  for (const auto &det : msg.detections)
  {
    // Expect normalized image coordinates in bbox.center.position.x/y (0..1)
    double cxNorm = det.bbox.center.position.x;
    double cyNorm = det.bbox.center.position.y;

    // Convert normalized to pixel coordinates
    double px = cxNorm * static_cast<double>(cam.intrinsic.width);
    double py = cyNorm * static_cast<double>(cam.intrinsic.height);

    // Normalized image plane coordinates (camera frame)
    double uNorm = (px - cam.intrinsic.cx) / cam.intrinsic.fx;
    double vNorm = (py - cam.intrinsic.cy) / cam.intrinsic.fy;

    double yaw = std::atan(uNorm);
    double pitch = std::atan(vNorm);

    Bearing b;
    b.yaw = yaw;
    b.pitch = pitch;

    Observation obs(timeTag, b);
    obs.id = tempId++;

    observations.push_back(std::move(obs));
  }

  return observations;
}

} // namespace slam
