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

// Definition of static member declared in header
std::optional<CameraInfo> ObservationBuilder::s_camera_info = std::nullopt;

void ObservationBuilder::setCameraInfo(const CameraInfo& info)
{
  ObservationBuilder::s_camera_info = info;
}

std::vector<Observation>
ObservationBuilder::fromBboxArray(
    const vision_msgs::msg::Detection3DArray& msg,
    double timeTag)
{
  std::vector<Observation> observations;
  if (!ObservationBuilder::s_camera_info.has_value())
    return observations;

  const auto& cam = ObservationBuilder::s_camera_info.value();
  observations.reserve(msg.detections.size());

  int temp_id = 0;

  for (const auto &det : msg.detections)
  {
    // Expect normalized image coordinates in bbox.center.position.x/y (0..1)
    double cx_norm = det.bbox.center.position.x;
    double cy_norm = det.bbox.center.position.y;

    // Convert normalized to pixel coordinates
    double px = cx_norm * static_cast<double>(cam.intrinsic.width);
    double py = cy_norm * static_cast<double>(cam.intrinsic.height);

    // Normalized image plane coordinates (camera frame)
    double u_n = (px - cam.intrinsic.cx) / cam.intrinsic.fx;
    double v_n = (py - cam.intrinsic.cy) / cam.intrinsic.fy;

    double yaw = std::atan(u_n);
    double pitch = std::atan(v_n);

    Bearing b;
    b.yaw = yaw;
    b.pitch = pitch;

    Observation obs(timeTag, b);
    obs.id = temp_id++;

    observations.push_back(std::move(obs));
  }

  return observations;
}

} // namespace slam
