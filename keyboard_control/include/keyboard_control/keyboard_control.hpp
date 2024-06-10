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
    void processMovementCommand(MovementCommand command, uint8_t cli_command, std::string cli_command_value);
    void processGeneralCommand(GeneralCommand command);
    void increaseSpeed();
    void decreaseSpeed();
    void sendRequest(uint16_t mode);
    void startCliCommandReceive();
    void getCommand();
    std::string getClicommand();
    int parseCliCommand(const std::string &cli_input, std::string &command_value);

private:
    rclcpp::Publisher<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandPublisher;
    rclcpp::Client<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandClient;
    
    float _stepDisplace;
    float _stepRotate;
    bool _takeCLICommand = false;
    const std::map<std::string, int> _cliCommandMap = { // CLI command list
        {"nomove", 0},
        {"gotoorigin", 1},
        {"goto", 2},
        {"headto", 3},
        {"stop", 4},
        {"headfw", 5},
        {"headbw", 6},
        {"headleft", 7},
        {"headright", 8}
    };
};

enum class MovementCommand //: uint8_t
{
    CLI_COMMAND = 0,   // press C key
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
