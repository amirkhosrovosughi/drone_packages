#ifndef SLAM__BACKEND__SLAM_BACKEND_HPP_
#define SLAM__BACKEND__SLAM_BACKEND_HPP_

#include <functional>
#include <memory>

#include "observation/observation.hpp"
#include "map/slam_map.hpp"
#include "motion/motion_model.hpp"
#include "common/slam_logger.hpp" // TODO: should be removed later ...

namespace slam
{

/**
 * @class SlamBackend
 * @brief Abstract interface for all SLAM backends (EKF, Graph-SLAM, FastSLAM, etc.).
 *
 * This class defines the algorithm-level contract between the ROS-facing
 * SlamManager and the SLAM algorithm implementation.
 *
 * Design rules:
 * - No ROS includes
 * - No sensor-specific logic
 * - No timing assumptions
 * - Operates only on abstract data structures
 */
class SlamBackend
{
public:
  virtual ~SlamBackend() = default;

  virtual void initialize() = 0;
  virtual void reset() = 0;
  virtual void processMotion(const MotionConstraint& m) = 0;
  virtual void processObservation(const Observations& o) = 0;
  virtual MapSummary getMap() const = 0;
  virtual void setLogger(LoggerPtr logger) = 0;
  // TODO: Not ROS-agnostic to be changed later
  // should be replaced with:
  // struct BackendLogger {
  //   virtual void info(std::string_view) = 0;
  //   virtual void warn(std::string_view) = 0;
  // };

  /**
   * @brief Register a callback to be invoked when the map is updated.
   *
   * The SlamManager typically uses this to publish map updates to ROS.
   *
   * @param callback Function to be called on map updates.
   */
  void setMapUpdateCallback(
    std::function<void(const SlamMap& map)> callback)
  {
    _mapUpdateCallback = std::move(callback);
  }

protected:
  /**
   * @brief Notify listeners that the map has been updated.
   *
   * Backends should call this method whenever a meaningful map update
   * occurs (e.g., after an EKF update or graph optimization).
   *
   * @param map Current SLAM map.
   */
  void notifyMapUpdate(const SlamMap& map) const
  {
    if (_mapUpdateCallback)
    {
      _mapUpdateCallback(map);
    }
  }

protected:
  std::function<void(const SlamMap& map)> _mapUpdateCallback; ///< Map update callback
};

}  // namespace slam

#endif  // SLAM__BACKEND__SLAM_BACKEND_HPP_