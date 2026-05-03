#ifndef SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_
#define SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "startup/gps_startup_session.hpp"

struct SlamStartupContractConfig
{
  bool gpsRequiredInit = false;
  bool dropInputWhileWaitingGpsInit = true;
  bool allowDegradedNoGps = false;
  double gpsInitTimeoutSec = 15.0;
  std::unique_ptr<GpsStartupSession> gpsStartupSession;
  std::string warningMessage;
};

namespace startup
{

/**
 * @brief Load a SlamStartupContractConfig from node parameters.
 */
SlamStartupContractConfig loadContractFromNode(rclcpp::Node& node);

}  // namespace startup

#endif  // SLAM__NODE__SLAM_STARTUP_CONTRACT_HPP_
