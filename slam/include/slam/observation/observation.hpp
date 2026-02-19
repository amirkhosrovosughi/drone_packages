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

/// Observation payload (constraint)
/**
 * @brief Bearing-only observation expressed in camera frame.
 *
 * Represents the direction from the camera optical center to the observed
 * feature as two angles: yaw (azimuth) and pitch (elevation). This is a
 * purely geometric/math representation and contains no image-dependent
 * bounding box metadata.
 */
struct Bearing
{
    double yaw = 0.0;   // azimuth (radians)
    double pitch = 0.0; // elevation (radians)
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

    using Payload = std::variant<Point3D, BoundingBox, Bearing>;
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
