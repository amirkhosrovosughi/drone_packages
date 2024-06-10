// px4_command_handler.hpp
#ifndef PX4_COMMAND_HANDLER_HPP_
#define PX4_COMMAND_HANDLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "common_utilities/transform_util.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>
#include <drone_msgs/msg/drone_direction_command.hpp>
#include <drone_msgs/srv/drone_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

//TODO: add doxygen

class Px4CommandHandlerNode : public rclcpp::Node {
public:
  Px4CommandHandlerNode();

private:
  // Function prototypes
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void publishtopics();
  void publishTrajectorySetpoint();
  void publishOffboardControlMode();
  void directionCallback(const drone_msgs::msg::DroneDirectionCommand command);
  void generalCommandService(const std::shared_ptr<drone_msgs::srv::DroneMode::Request> request,
                             std::shared_ptr<drone_msgs::srv::DroneMode::Response> response);
  void publishSetpointConfig(bool automatic);
  void activateDrone(bool automated);
  void positionCallback(px4_msgs::msg::VehicleLocalPosition position);
  template<typename Derived>
  std::string getStringFromVector(const Eigen::MatrixBase<Derived>& vector);
  void exectueCliCommand(uint8_t command, std::string command_value);
  bool safeParseFloat(const std::string& str, float& result);
  bool safeParseVector4f(const std::string& str, Eigen::Vector4f& result);

  // Member variables
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicleCommandPublisher;
  rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr _configControlPublisher;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboardControlModePublisher;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectorySetpointPublisher;

  rclcpp::Subscription<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandSubscriber;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _localPositionSubscriber;
  rclcpp::Service<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandServer;
  rclcpp::TimerBase::SharedPtr _timer;
  Eigen::Vector4f _targetPostion;
  Eigen::Vector4f _currentPostion;
};

#endif // PX4_COMMAND_HANDLER_HPP_

