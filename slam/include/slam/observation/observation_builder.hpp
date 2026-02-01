#ifndef OBSERVATION_BUILDER_HPP_
#define OBSERVATION_BUILDER_HPP_

#include "observation/observation.hpp"

#include "drone_msgs/msg/point_list.hpp"
#include "drone_msgs/msg/bounding_boxes.hpp"

#include <vector>

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
};

} // namespace slam

#endif  // OBSERVATION_BUILDER_HPP_
