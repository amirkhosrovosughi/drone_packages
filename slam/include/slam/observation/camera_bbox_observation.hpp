#ifndef CAMERA_BBOX_OBSERVATION_HPP_
#define CAMERA_BBOX_OBSERVATION_HPP_

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
     * @param x_min Top-left x pixel
     * @param y_min Top-left y pixel
     * @param x_max Bottom-right x pixel
     * @param y_max Bottom-right y pixel
     * @param class_id Class id of detected object
     * @param confidence Detection confidence (0..1)
     */
    CameraBBoxObservation(int x_min, int y_min, int x_max, int y_max,
                          int class_id = -1, float confidence = 1.0f);

    /** Get the width of the bounding box in pixels */
    int width() const;

    /** Get the height of the bounding box in pixels */
    int height() const;

    /** Get the center of the bounding box in pixel coordinates */
    Eigen::Vector2f center() const;

public:
    int x_min{0};       ///< Top-left x
    int y_min{0};       ///< Top-left y
    int x_max{0};       ///< Bottom-right x
    int y_max{0};       ///< Bottom-right y
    int class_id{-1};   ///< Detected object class id
    float confidence{1.0f}; ///< Confidence score
};

#endif  // CAMERA_BBOX_OBSERVATION_HPP_
