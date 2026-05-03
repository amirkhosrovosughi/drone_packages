#ifndef SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_
#define SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#ifdef USE_GPS
#include "startup/gps_startup_session.hpp"
#endif

struct SlamStartupContractConfig
{
  bool dropInputWhileWaitingGpsInit = true;
  bool allowDegradedNoGps = false;
  double gpsInitTimeoutSec = 15.0;
  std::string warningMessage;

#ifdef USE_GPS
  bool gpsRequiredInit = false;
  bool gpsFusionEnabled = false;
  std::unique_ptr<GpsStartupSession> gpsStartupSession;
#endif
};

namespace startup
{

/**
 * @brief Load a SlamStartupContractConfig from node parameters.
 */
SlamStartupContractConfig loadContractFromNode(rclcpp::Node& node);

}  // namespace startup

#endif  // SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_
