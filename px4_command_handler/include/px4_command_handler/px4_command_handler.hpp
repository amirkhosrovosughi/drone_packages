// px4_command_handler.hpp
#ifndef PX4_COMMAND_HANDLER_HPP_
#define PX4_COMMAND_HANDLER_HPP_

/**
 * @file px4_command_handler.hpp
 * @brief ROS 2 node for handling PX4 vehicle commands, offboard control, and drone direction inputs.
 *
 * This node acts as an interface between high-level drone commands
 * (keyboard/manual or service requests) and PX4 middleware by publishing 
 * vehicle commands, trajectory setpoints, and control modes.
 */

#include "rclcpp/rclcpp.hpp"
#include "common_utilities/transform_util.hpp"
#include "px4_command_handler_util.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>
#include <drone_msgs/msg/drone_direction_command.hpp>
#include <drone_msgs/srv/drone_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

/**
 * @class Px4CommandHandlerNode
 * @brief Node responsible for translating drone commands into PX4-compatible messages.
 */
class Px4CommandHandlerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct a new Px4CommandHandlerNode.
   *
   * Initializes publishers, subscribers, and services for PX4 offboard control.
   */
  Px4CommandHandlerNode();

private:
  /**
   * @brief Publish a generic PX4 vehicle command.
   *
   * @param command The PX4 command ID (MAVLink enum values).
   * @param param1 Optional first parameter for the command.
   * @param param2 Optional second parameter for the command.
   */
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  /**
   * @brief Periodically publish required PX4 topics (trajectory + control mode).
   */
  void publishTopics();

  /**
   * @brief Publish a trajectory setpoint to the PX4.
   *
   * Uses `_targetPosition` as the desired pose in ENU, 
   * converts it to NED, and publishes to PX4.
   */
  void publishTrajectorySetpoint();

  /**
   * @brief Publish the offboard control mode configuration.
   *
   * Ensures PX4 is in position-control mode while in offboard.
   */
  void publishOffboardControlMode();

  /**
   * @brief Callback for manual drone movement commands.
   *
   * @param command Drone direction command message.
   */
  void directionCallback(const drone_msgs::msg::DroneDirectionCommand& command);

  /**
   * @brief ROS 2 service callback for general drone commands (arm, disarm, etc.).
   *
   * @param request Request containing the drone mode command.
   * @param response Response with success flag.
   */
  void generalCommandService(const std::shared_ptr<drone_msgs::srv::DroneMode::Request> request,
                             std::shared_ptr<drone_msgs::srv::DroneMode::Response> response);

  /**
   * @brief Publish a PX4 control mode configuration.
   *
   * @param automatic True for autonomous mode, false for manual.
   */
  void publishSetpointConfig(bool automatic);

  /**
   * @brief Arm and switch the drone into offboard mode.
   *
   * @param automated If true, also prepare the drone for automated behavior.
   */
  void activateDrone(bool automated);

  /**
   * @brief Callback for receiving drone local position updates.
   *
   * @param position PX4 local position message.
   */
  void positionCallback(const px4_msgs::msg::VehicleLocalPosition& position);

  /**
   * @brief Execute a CLI drone command (e.g., go to origin, stop).
   *
   * @param command Command type (from DroneDirectionCommand).
   * @param commandValue Additional argument (e.g., heading angle or coordinates).
   */
  void executeCliCommand(uint8_t command, std::string command_value);

  // Member variables
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicleCommandPublisher;        ///< Publisher for PX4 vehicle commands.
  rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _configControlPublisher;     ///< Publisher for PX4 vehicle control mode configuration.
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboardControlModePublisher; ///< Publisher for PX4 offboard control mode.
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectorySetpointPublisher; ///< Publisher for PX4 trajectory setpoints.

  rclcpp::Subscription<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandSubscriber; ///< Subscriber for high-level drone direction commands.
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _localPositionSubscriber;  ///< Subscriber for drone local position updates from PX4.

  rclcpp::Service<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandServer; ///< Service server for general drone commands (arm, mode switching, etc.).

  rclcpp::TimerBase::SharedPtr _publishTimer; ///< Timer for periodic PX4 topic publishing.

  Eigen::Vector4f _targetPosition; ///< Target drone position and heading (ENU frame). Format: [x, y, z, yaw].
  Eigen::Vector4f _currentPosition; ///< Current drone position and heading (ENU frame). Format: [x, y, z, yaw].
};

#endif // PX4_COMMAND_HANDLER_HPP_

