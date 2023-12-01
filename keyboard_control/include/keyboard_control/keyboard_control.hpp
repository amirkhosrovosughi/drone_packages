// keyboard_control.hpp
#ifndef KEYBOARD_CONTROL_HPP_
#define KEYBOARD_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <drone_msgs/msg/drone_direction_command.hpp>
#include <drone_msgs/srv/drone_mode.hpp>

//TODO: add doxygen

enum class MovementCommand;
enum class GeneralCommand;

class KeyboardControl : public rclcpp::Node {
public:
    KeyboardControl();

private:
    void startKeyboardControl();
    void processKeyboardInput(char key);
    void processMovementCommand(MovementCommand command);
    void processGeneralCommand(GeneralCommand command);
    void increaseSpeed();
    void decreaseSpeed();
    void sendRequest(uint16_t mode);

private:
    rclcpp::Publisher<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandPublisher;
    rclcpp::Client<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandClient;
    
    float _stepDisplace;
    float _stepRotate;
    
};

enum class MovementCommand //: uint8_t
{
    STOP = 0,          // press space
    GO_UP = 1,         // press w key
    GO_DOWN = 2,       // press S key
    GO_FORWARD = 3,    // press up key
    GO_BACKWARD = 4,   // press down key
    GO_RIGHT = 5,      // press right key
    GO_LEFT = 6,       // press left key
    ROTATE_CW = 7,     // press d key
    ROTATE_ACW = 8     // press s key
};

enum class GeneralCommand //: uint8_t
{
    SLEEP_MODE = 0,       // press 0 key
    MANUAL_MODE = 1,      // press 1 key
    AUTOMATIC_MODE= 2,    // press 2 key
    INCREASE_SPEED = 3,   // press + key
    DECREASE_SPEED = 4,  // press - key
};

#endif  // KEYBOARD_CONTROL_HPP_
