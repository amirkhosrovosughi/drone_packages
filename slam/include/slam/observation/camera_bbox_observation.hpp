#ifndef SLAM__OBSERVATION__CAMERA_BBOX_OBSERVATION_HPP_
#define SLAM__OBSERVATION__CAMERA_BBOX_OBSERVATION_HPP_

#include <cstdint>
#include <string>
#include <Eigen/Dense>

/**
 * @brief Represents a bounding box observation from a camera.
 *        Can hold pixel coordinates, class id, and confidence.
 */
class CameraBBoxObservation {
public:
    /**
     * @brief Construct an empty bounding box observation.
     */
    CameraBBoxObservation() = default;

    /**
     * @brief Construct a bounding box observation.
     */
    CameraBBoxObservation(int xMin, int yMin, int xMax, int yMax,
                          int classId = -1, float confidence = 1.0f);

    /** Get the width of the bounding box in pixels */
    int width() const;

    /** Get the height of the bounding box in pixels */
    int height() const;

    /** Get the center of the bounding box in pixel coordinates */
    Eigen::Vector2f center() const;

public:
    int xMin{0};        ///< Top-left x
    int yMin{0};        ///< Top-left y
    int xMax{0};        ///< Bottom-right x
    int yMax{0};        ///< Bottom-right y
    int classId{-1};    ///< Detected object class id
    float confidence{1.0f}; ///< Confidence score
};

#endif  // SLAM__OBSERVATION__CAMERA_BBOX_OBSERVATION_HPP_
