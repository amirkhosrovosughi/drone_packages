// px4_command_handler.cpp

#include "px4_command_handler.hpp"
#include "rclcpp/qos.hpp"

static const float maxStepDisplace = 10.0;
static const float minStepDisplace = -10.0;

static const float maxStepRotate = 1.0;
static const float minStepRotate = -1.0;

Px4CommandHandlerNode::Px4CommandHandlerNode() : Node("px4_command_handler")
{

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

  _vehicleCommandPublisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
  _moveCommandSubscriber = this->create_subscription<drone_msgs::msg::DroneDirectionCommand>(
              "/keyboard_control/movement_command", 10, std::bind(&Px4CommandHandlerNode::directionCallback, this, std::placeholders::_1));
  _localPositionSubscriber = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
              "/fmu/out/vehicle_local_position", qos_profile, std::bind(&Px4CommandHandlerNode::positionCallback, this, std::placeholders::_1));
  _generalCommandServer = this->create_service<drone_msgs::srv::DroneMode>("/keyboard_control/general_command",
                std::bind(&Px4CommandHandlerNode::generalCommandService, this, std::placeholders::_1, std::placeholders::_2));


  // should be removed

  

  _timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Px4CommandHandlerNode::publishtopics, this));
}

void Px4CommandHandlerNode::publishtopics()
{
    publish_trajectory_setpoint();
}


void Px4CommandHandlerNode::publish_trajectory_setpoint()
{
  // see what is the correct position

  // see what command we have had
  
  // based of those, decide where to go
}

void Px4CommandHandlerNode::publishVehicleCommand(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicleCommandPublisher->publish(msg);
}



void Px4CommandHandlerNode::directionCallback(drone_msgs::msg::DroneDirectionCommand command)
{
      // Limit each variable up to the specified range
    command.vertical_step = std::max(std::min(command.vertical_step, maxStepDisplace), minStepDisplace);
    command.forward_step = std::max(std::min(command.forward_step, maxStepDisplace), minStepDisplace);
    command.lateral_step = std::max(std::min(command.lateral_step, maxStepDisplace), minStepDisplace);
    command.angular_step = std::max(std::min(command.angular_step, maxStepRotate), minStepRotate);

    // Print content of the message (ROS.Info)
    RCLCPP_INFO(this->get_logger(), "Received DroneDirectionCommand message:");
    RCLCPP_INFO(this->get_logger(), "Stop: %s", command.stop ? "true" : "false");
    if (command.vertical_step != 0.0f)
    {
      RCLCPP_INFO(this->get_logger(), "Vertical Step: %f", command.vertical_step);
    }
    if (command.forward_step != 0.0f)
    {
      RCLCPP_INFO(this->get_logger(), "Forward Step: %f", command.forward_step);
    }
    if (command.lateral_step != 0.0f)
    {
      RCLCPP_INFO(this->get_logger(), "Lateral Step: %f", command.lateral_step);
    }
    if (command.angular_step != 0.0f)
    {
      RCLCPP_INFO(this->get_logger(), "Angular Step: %f", command.angular_step);
    }
}

void Px4CommandHandlerNode::generalCommandService(const std::shared_ptr<drone_msgs::srv::DroneMode::Request> request,
                  std::shared_ptr<drone_msgs::srv::DroneMode::Response> response)
{
  // start subscriber to the drone state

  // switch-case 
  switch (request->mode)
    {
        case 0:
            // start publishing: 1) landing (later) 2) turn off drone arm 
            break;

        case 1:
            // change mode, enable arm
            activateDrone(false);
            break;

        case 2:
            // change mode, enalbe arm , ask to do some pre-defined automated manouver (later)
            activateDrone(true);
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Undefined command: %d", request->mode);
            break;
    }

  response->success = true;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "request is: %d", request->mode);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%s]", response->success ? "successful" : "unsuccessful");
}

void Px4CommandHandlerNode::positionCallback(px4_msgs::msg::VehicleLocalPosition local_position_msg)
{
  RCLCPP_INFO(get_logger(), "got the position ...");
  float north_position = local_position_msg.x;
  float east_position = local_position_msg.y;
  float down_position = local_position_msg.z;

  float yaw_angle = local_position_msg.heading;
}

void Px4CommandHandlerNode::publishSetpointConfig(bool automatic)
{
  RCLCPP_INFO(get_logger(), "publishing setpoint config");
	px4_msgs::msg::VehicleControlMode msg{};
	msg.flag_armed = true;
  msg.flag_control_manual_enabled = !automatic;
  msg.flag_control_auto_enabled = automatic;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_configControlPublisher->publish(msg);
}	


void Px4CommandHandlerNode::activateDrone(bool automated)
{
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
  RCLCPP_INFO(this->get_logger(), "Go to offboard control mode");

  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(this->get_logger(), "Arm the drone");

  if (automated)
  {
    //TODO:// listen to some topic to general the automated path
    // not sure to give position directly. or some path somehow, or behaviour, something to explore 
  }
}
