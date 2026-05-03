#include "startup/slam_startup_contract.hpp"

namespace startup
{

SlamStartupContractConfig loadContractFromNode(rclcpp::Node& node)
{
  SlamStartupContractConfig config;

  config.dropInputWhileWaitingGpsInit = node.declare_parameter<bool>(
    "startup.drop_input_while_waiting_gps_init",
    config.dropInputWhileWaitingGpsInit);
  config.allowDegradedNoGps = node.declare_parameter<bool>(
    "startup.allow_degraded_no_gps",
    config.allowDegradedNoGps);
  config.gpsInitTimeoutSec = node.declare_parameter<double>(
    "startup.gps_init_timeout_sec",
    config.gpsInitTimeoutSec);

#ifdef USE_GPS
  config.gpsRequiredInit = node.declare_parameter<bool>(
    "startup.enable_gps_initialization",
    false);
  config.gpsFusionEnabled = node.declare_parameter<bool>(
    "startup.enable_gps_fusion",
    false);

  if (config.gpsRequiredInit)
  {
    config.gpsStartupSession = createGpsStartupSessionFromNodeParameters(node);
  }
  else
  {
    const std::string mode = node.declare_parameter<std::string>(
      "startup.integration_mode",
      "gps_disabled");

    if (mode == "gps_required_init")
    {
      config.gpsRequiredInit = true;
      config.gpsStartupSession = createGpsStartupSessionFromNodeParameters(node);
    }
    else if (mode != "gps_disabled")
    {
      config.warningMessage =
        "Unknown startup.integration_mode='" + mode +
        "'. Falling back to 'gps_disabled'.";
    }
  }
#endif

  return config;
}

}  // namespace startup
