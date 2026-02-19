#ifndef OBSERVATION_BUILDER_HPP_
#define OBSERVATION_BUILDER_HPP_

#include "observation/observation.hpp"
#include "drone_msgs/msg/point_list.hpp"
#include "drone_msgs/msg/bounding_boxes.hpp"
#include <vision_msgs/msg/detection3_d_array.hpp>
#include "common/def_slam.hpp"

#include <vector>
#include <optional>

namespace slam
{

/**
 * @brief Converts raw sensor messages into Observation objects.
 *
 * This is the ONLY place where:
 * - ROS messages are interpreted
 * - Sensor geometry is handled
 */
class ObservationBuilder
{
public:
  /**
   * @brief Convert 3D point list message to observations.
   *
   * Each point becomes one Observation with Point3D payload.
   */
  static std::vector<Observation>
  fromPointList(const drone_msgs::msg::PointList& msg,
                double timeTag);

  /**
   * @brief Convert bounding box detections to observations.
   *
   * Each bounding box becomes one Observation with BoundingBox payload.
   */
  static std::vector<Observation>
  fromBoundingBoxes(const drone_msgs::msg::BoundingBoxes& msg,
                    double timeTag);

  /**
   * @brief Convert vision_msgs::Detection3DArray into bearing Observations.
   *
   * This method uses the camera intrinsics/extrinsics previously set via
   * `setCameraInfo` to compute purely geometric bearing observations
   * (yaw, pitch) for each detection.
   */
  static std::vector<Observation>
  fromBboxArray(const vision_msgs::msg::Detection3DArray& msg,
                double timeTag);

  /**
   * @brief Provide camera intrinsic/extrinsic info to the builder.
   *
   * ObservationBuilder stores camera geometry and converts image-space
   * detections into purely mathematical observations for the backend.
   */
  static void setCameraInfo(const CameraInfo& info);
private:
  static std::optional<CameraInfo> s_camera_info;
};

} // namespace slam

#endif  // OBSERVATION_BUILDER_HPP_
