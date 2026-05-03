#include "startup/slam_startup_contract.hpp"

namespace startup
{

SlamStartupContractConfig loadContractFromNode(rclcpp::Node& node)
{
  SlamStartupContractConfig config;

  const std::string mode = node.declare_parameter<std::string>(
    "startup.integration_mode",
    "gps_disabled");

  config.dropInputWhileWaitingGpsInit = node.declare_parameter<bool>(
    "startup.drop_input_while_waiting_gps_init",
    config.dropInputWhileWaitingGpsInit);
  config.allowDegradedNoGps = node.declare_parameter<bool>(
    "startup.allow_degraded_no_gps",
    config.allowDegradedNoGps);
  config.gpsInitTimeoutSec = node.declare_parameter<double>(
    "startup.gps_init_timeout_sec",
    config.gpsInitTimeoutSec);

  if (mode == "gps_required_init")
  {
    config.gpsRequiredInit = true;
    config.gpsStartupSession = createGpsStartupSessionFromNodeParameters(node);
    return config;
  }

  if (mode != "gps_disabled")
  {
    config.warningMessage =
      "Unknown startup.integration_mode='" + mode +
      "'. Falling back to 'gps_disabled'.";
  }

  return config;
}

}  // namespace startup
