#ifndef OBSERVATION_HPP_
#define OBSERVATION_HPP_

#include <Eigen/Dense>
#include <string>
#include <variant>
#include <vector>

namespace slam
{

/**
 * @brief 3D point observation (e.g., depth feature, triangulated landmark)
 */
struct Point3D
{
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

/**
 * @brief 2D bounding box observation (image space)
 */
struct BoundingBox
{
    float x = 0.0f;        ///< top-left x (pixels)
    float y = 0.0f;        ///< top-left y
    float width = 0.0f;
    float height = 0.0f;

    float confidence = 1.0f;
    std::string class_label;
};

/**
 * @brief Unified Observation abstraction for SLAM backend.
 *
 * Represents WHAT constraint was observed, not HOW it is used.
 */
struct Observation
{
    /// Landmark ID hint (-1 if unassociated)
    int id = -1;

    /// Timestamp (seconds)
    double timeTag = 0.0;

    /// Observation payload (constraint)
    using Payload = std::variant<Point3D, BoundingBox>;
    Payload payload;

    Observation() = default;

    template<typename T>
    Observation(double t, const T& data)
      : timeTag(t), payload(data)
    {}
};

using Observations = std::vector<Observation>;

} // namespace slam

#endif  // OBSERVATION_HPP_
