// px4_command_handler.cpp

#include "px4_command_handler.hpp"
#include "rclcpp/qos.hpp"

static const float maxStepDisplace = 10.0;
static const float minStepDisplace = -10.0;

static const float maxStepRotate = 1.0;
static const float minStepRotate = -1.0;

static const float maxHeading = M_PI;
static const float minHeading = -M_PI;

Px4CommandHandlerNode::Px4CommandHandlerNode() : Node("px4_command_handler")
{

  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

  _vehicleCommandPublisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
  _offboardControlModePublisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode",10);
  _trajectorySetpointPublisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

  _moveCommandSubscriber = this->create_subscription<drone_msgs::msg::DroneDirectionCommand>(
              "/keyboard_control/movement_command", 10, std::bind(&Px4CommandHandlerNode::directionCallback, this, std::placeholders::_1));
  _localPositionSubscriber = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
              "/fmu/out/vehicle_local_position", qos_profile, std::bind(&Px4CommandHandlerNode::positionCallback, this, std::placeholders::_1));
  _generalCommandServer = this->create_service<drone_msgs::srv::DroneMode>("/keyboard_control/general_command",
                std::bind(&Px4CommandHandlerNode::generalCommandService, this, std::placeholders::_1, std::placeholders::_2));

  _currentPostion = Eigen::Vector4f(0,0,0,0);
  _targetPostion = Eigen::Vector4f(0,0,2,0);


  // should be removed

  

  _timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Px4CommandHandlerNode::publishtopics, this));
}

void Px4CommandHandlerNode::publishtopics()
{
  publishOffboardControlMode();
  publishTrajectorySetpoint();
}


void Px4CommandHandlerNode::publishTrajectorySetpoint()
{
  // see what is the correct position

  // see what command we have had |||--> NO, apply it as it comes
  
  // based of those, decide where to go
  Eigen::Vector3f targetNED = TransformUtil::enuToNed(_targetPostion.head(3));

  px4_msgs::msg::TrajectorySetpoint msg{};
  	msg.position = {targetNED[0], targetNED[1], targetNED[2]};
  	msg.yaw = _targetPostion[3]; // [-PI:PI]
  	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  	_trajectorySetpointPublisher->publish(msg);
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



void Px4CommandHandlerNode::directionCallback(const drone_msgs::msg::DroneDirectionCommand command)
{
      // Limit each variable up to the specified range
    float vertical_step = std::max(std::min(command.vertical_step, maxStepDisplace), minStepDisplace);
    float forward_step = std::max(std::min(command.forward_step, maxStepDisplace), minStepDisplace);
    float lateral_step = std::max(std::min(command.lateral_step, maxStepDisplace), minStepDisplace);
    float angular_step = std::max(std::min(command.angular_step, maxStepRotate), minStepRotate);

    // Print content of the message (ROS.Info)
    RCLCPP_INFO(this->get_logger(), ">>>> Received DroneDirectionCommand message:");
    if (command.stop)
    {
      _targetPostion = _currentPostion;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Stop: %s", command.stop ? "true" : "false");
    if (vertical_step != 0.0f)
    {
      _targetPostion = _currentPostion + Eigen::Vector4f(0.0f, 0.0f, vertical_step, 0.0f);
      RCLCPP_INFO(this->get_logger(), "Vertical Step: %f", vertical_step);
    }
    if (forward_step != 0.0f || lateral_step != 0.0f)
    {
      RCLCPP_INFO(this->get_logger(), "Forward Step: %f", forward_step);
      RCLCPP_INFO(this->get_logger(), "Lateral Step: %f", lateral_step);

      // calculate needed rotation:
      Eigen::Vector2f relativeTarget = TransformUtil::rotate2D(Eigen::Vector2f(forward_step, lateral_step),angular_step);

      _targetPostion = _currentPostion + Eigen::Vector4f(relativeTarget[0], relativeTarget[1], 0.0f, 0.0f);



    }
    if (angular_step != 0.0f)
    {
      float targetHeading = std::max(std::min(angular_step, maxHeading), minHeading);
      _targetPostion = _currentPostion + Eigen::Vector4f(0.0f, 0.0f, 0.0f, targetHeading);
      RCLCPP_INFO(this->get_logger(), "Angular Step: %f", angular_step);
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
  // RCLCPP_INFO(get_logger(), "got the position ...");

  Eigen::Vector3f ned_coordinates(local_position_msg.x, local_position_msg.y, local_position_msg.z);


  // RCLCPP_INFO(get_logger(), "ned_coordinates is (%s)",getStringFromVector(ned_coordinates));

  Eigen::Vector3f enu_coordinates = TransformUtil::nedToEnu(ned_coordinates);
  // RCLCPP_INFO(get_logger(), "ned_coordinates is (%s)",getStringFromVector(enu_coordinates));

  float yaw_angle = local_position_msg.heading;

  _targetPostion = Eigen::Vector4f(enu_coordinates[0], enu_coordinates[1], enu_coordinates[2], local_position_msg.heading);

}

char* Px4CommandHandlerNode::getStringFromVector(const Eigen::Vector3f& vector)
{
    std::stringstream ss;
    ss << vector.transpose();
    
    std::string str = ss.str();
    char* result = new char[str.size() + 1]; // +1 for the null terminator
    std::strcpy(result, str.c_str());
    
    return result;
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



void Px4CommandHandlerNode::publishOffboardControlMode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_offboardControlModePublisher->publish(msg);
}
