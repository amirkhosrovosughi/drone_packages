#ifndef SLAM__PIPELINE__SLAM_PIPELINE_HPP_
#define SLAM__PIPELINE__SLAM_PIPELINE_HPP_

#include <functional>
#include <memory>

#include "observation/observation.hpp"
#include "map/slam_map.hpp"
#include "motion/motion_model.hpp"
#include "common/def_slam_core.hpp"
#include "common/slam_logger.hpp"

namespace slam
{

/**
 * @class SlamPipeline
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
class SlamPipeline
{
public:
  virtual ~SlamPipeline() = default;

  virtual void initialize() = 0;
  virtual void reset() = 0;
  virtual void processMotion(const MotionConstraint& m) = 0;
  virtual void processObservation(const Observations& o) = 0;
  virtual MapSummary getMap() const = 0;
  virtual void setLogger(LoggerPtr logger) = 0;

  /**
   * @brief Align pipeline startup frame with the accepted GPS local anchor.
   *
   * Default behavior is no-op so existing backends remain source-compatible.
   * Backends can override this to initialize internal state consistently with
   * the startup local frame before GPS fusion is introduced.
   */
  virtual void applyStartupAnchor(const LocalFrameAnchor& anchor)
  {
    (void)anchor;
  }

  /**
   * @brief Fuse an absolute GPS position constraint into the pipeline.
   * Default behavior is no-op; backends override to implement the update.
   */
  virtual void processGpsMeasurement(const GpsConstraint& constraint)
  {
    (void)constraint;
  }
  // TODO: Not ROS-agnostic to be changed later
  // should be replaced with:
  // struct BackendLogger {
  //   virtual void info(std::string_view) = 0;
  //   virtual void warn(std::string_view) = 0;
  // };

  /**
   * @brief Register a callback to be invoked when the map is updated.
   */
  void setMapUpdateCallback(
    std::function<void(const SlamMap& map)> callback)
  {
    _mapUpdateCallback = std::move(callback);
  }

protected:
  /**
   * @brief Notify listeners that the map has been updated.
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

#endif  // SLAM__PIPELINE__SLAM_PIPELINE_HPP_