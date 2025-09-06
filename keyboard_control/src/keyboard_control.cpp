// keyboard_control.cpp
#include "keyboard_control.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <termios.h>
#include <unistd.h>

static constexpr float DEFAULT_STEP_DISPLACE = 0.2f;
static constexpr float MAX_STEP_DISPLACE     = 2.0f;
static constexpr float MIN_STEP_DISPLACE     = 0.05f;
static constexpr float CHANGE_STEP_DISPLACE  = 0.05f;

static constexpr float DEFAULT_STEP_ROTATE = 0.05f;
static constexpr float MAX_STEP_ROTATE     = 0.5f;
static constexpr float MIN_STEP_ROTATE     = 0.01f;
static constexpr float CHANGE_STEP_ROTATE  = 0.01f;

KeyboardControl::KeyboardControl(bool autostart)
: Node("keyboard_control")
{
    // Initialize publisher & client
    _moveCommandPublisher = this->create_publisher<drone_msgs::msg::DroneDirectionCommand>(
        "/keyboard_control/movement_command", 10);
    _generalCommandClient = create_client<drone_msgs::srv::DroneMode>(
        "/keyboard_control/general_command");

    _stepDisplace = DEFAULT_STEP_DISPLACE;
    _stepRotate = DEFAULT_STEP_ROTATE;

    if (autostart) {
        // Start listening to the keyboard
        startKeyboardControl();
    }
}

void KeyboardControl::startKeyboardControl()
{
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    while (rclcpp::ok())
    {
        if (!_takeCliCommand) {
            std::cin >> c;   // blocking single-char read
            processKeyboardInput(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        } else {
            startCliCommandReceive();
        }
    }

    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void KeyboardControl::startCliCommandReceive()
{
    std::string cliInput = getCliCommand();
    std::string commandValue;
    int cliCommand = parseCliCommand(cliInput, commandValue);
    processMovementCommand(MovementCommand::CLI_COMMAND, static_cast<uint8_t>(cliCommand), commandValue);

    // timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream time_stream;
    time_stream << std::put_time(std::localtime(&now_c), "%H:%M:%S");
    std::cout << "Command sent at: " << time_stream.str().c_str() << "\n";

    _takeCliCommand = false;
}

int KeyboardControl::parseCliCommand(const std::string &cliInput, std::string &commandValue)
{
    std::istringstream iss(cliInput);
    std::string command;
    iss >> command;

    std::getline(iss, commandValue);
    if (!commandValue.empty() && commandValue[0] == ' ') {
        commandValue.erase(0, 1);
    }

    auto it = _cliCommandMap.find(command);
    if (it != _cliCommandMap.end()) {
        std::cout << "command: " << it->second << " commandValue: " << commandValue << "\n";
        return it->second;
    } else {
        commandValue.clear();
        std::cout << "command not found, commandValue: " << commandValue << "\n";
        return 0;  // Unknown -> 0
    }
}

void KeyboardControl::processKeyboardInput(char key)
{
    switch (key) {
        case 'C': getCommand(); break;
        case 'W': processMovementCommand(MovementCommand::GO_UP);        RCLCPP_INFO(get_logger(), "pressed W"); break;
        case 'S': processMovementCommand(MovementCommand::GO_DOWN);      RCLCPP_INFO(get_logger(), "pressed S"); break;
        case 'A': processMovementCommand(MovementCommand::ROTATE_ACW);   RCLCPP_INFO(get_logger(), "pressed A"); break;
        case 'D': processMovementCommand(MovementCommand::ROTATE_CW);    RCLCPP_INFO(get_logger(), "pressed D"); break;
        case 'U': processMovementCommand(MovementCommand::GO_FORWARD);   RCLCPP_INFO(get_logger(), "pressed U"); break;
        case 'J': processMovementCommand(MovementCommand::GO_BACKWARD);  RCLCPP_INFO(get_logger(), "pressed J"); break;
        case 'H': processMovementCommand(MovementCommand::GO_LEFT);      RCLCPP_INFO(get_logger(), "pressed H"); break;
        case 'K': processMovementCommand(MovementCommand::GO_RIGHT);     RCLCPP_INFO(get_logger(), "pressed K"); break;
        case '0': processGeneralCommand(GeneralCommand::SLEEP_MODE);     break;
        case '1': processGeneralCommand(GeneralCommand::MANUAL_MODE);    break;
        case '2': processGeneralCommand(GeneralCommand::AUTOMATIC_MODE); break;
        case '=': processGeneralCommand(GeneralCommand::INCREASE_SPEED); break;
        case '-': processGeneralCommand(GeneralCommand::DECREASE_SPEED); break;
        default: break;
    }
}

void KeyboardControl::getCommand()
{
    _takeCliCommand = true;
    RCLCPP_INFO(get_logger(), "Enter your command:");
    std::cout << "Enter your command:";
}

std::string KeyboardControl::getCliCommand()
{
    struct termios oldt, newt;
    std::string command;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::getline(std::cin, command);

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    // ESC key (ASCII 27) cancels
    if (!command.empty() && static_cast<unsigned char>(command[0]) == 27U) {
        return "";
    }
    return command;
}

void KeyboardControl::processMovementCommand(MovementCommand command)
{
    processMovementCommand(command, 0, "");
}

void KeyboardControl::processMovementCommand(MovementCommand command, uint8_t cliCommand, std::string cliCommandValue)
{
    auto msg = drone_msgs::msg::DroneDirectionCommand();

    switch (command) {
        case MovementCommand::CLI_COMMAND:
            msg.cli_command = cliCommand;
            msg.cli_command_value = std::move(cliCommandValue);
            break;
        case MovementCommand::GO_UP:        msg.vertical_step =  _stepDisplace; break;
        case MovementCommand::GO_DOWN:      msg.vertical_step = -_stepDisplace; break;
        case MovementCommand::GO_FORWARD:   msg.forward_step =  _stepDisplace; break;
        case MovementCommand::GO_BACKWARD:  msg.forward_step = -_stepDisplace; break;
        case MovementCommand::GO_RIGHT:     msg.lateral_step  =  _stepDisplace; break;
        case MovementCommand::GO_LEFT:      msg.lateral_step  = -_stepDisplace; break;
        case MovementCommand::ROTATE_CW:    msg.angular_step  =  _stepRotate;   break;
        case MovementCommand::ROTATE_ACW:   msg.angular_step  = -_stepRotate;   break;
        default: break;
    }

    _moveCommandPublisher->publish(msg);
}

void KeyboardControl::processGeneralCommand(GeneralCommand command)
{
    switch (command) {
        case GeneralCommand::SLEEP_MODE:      sendRequest(0); break;
        case GeneralCommand::MANUAL_MODE:     sendRequest(1); break;
        case GeneralCommand::AUTOMATIC_MODE:  sendRequest(2); break;
        case GeneralCommand::INCREASE_SPEED:  increaseSpeed(); break;
        case GeneralCommand::DECREASE_SPEED:  decreaseSpeed(); break;
        default: break;
    }
}

// TODO: consider converting to a simple message publisher instead of service
void KeyboardControl::sendRequest(uint16_t mode)
{
    RCLCPP_INFO(get_logger(), "send request: %u", mode);
    auto request = std::make_shared<drone_msgs::srv::DroneMode::Request>();
    request->mode = mode;
    auto result = _generalCommandClient->async_send_request(request);

    auto timer = create_wall_timer(
        std::chrono::seconds(10),
        [&, mode]() {
            if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                auto response = result.get();
                if (response->success) {
                    RCLCPP_INFO(get_logger(), "Successfully set mode to %u", mode);
                } else {
                    RCLCPP_ERROR(get_logger(), "Failed to set mode to %u", mode);
                }
            } else {
                RCLCPP_ERROR(get_logger(), "Service call timed out. Unable to set mode to %u", mode);
            }
        }
    );
    (void)timer; // keep timer alive
}

void KeyboardControl::increaseSpeed()
{
    _stepDisplace = std::max(std::min(_stepDisplace + CHANGE_STEP_DISPLACE, MAX_STEP_DISPLACE), MIN_STEP_DISPLACE);
    _stepRotate   = std::max(std::min(_stepRotate + CHANGE_STEP_ROTATE, MAX_STEP_ROTATE), MIN_STEP_ROTATE);
    RCLCPP_INFO(get_logger(), "increase step size to %f", _stepDisplace);
}

void KeyboardControl::decreaseSpeed()
{
    _stepDisplace = std::max(std::min(_stepDisplace - CHANGE_STEP_DISPLACE, MAX_STEP_DISPLACE), MIN_STEP_DISPLACE);
    _stepRotate   = std::max(std::min(_stepRotate - CHANGE_STEP_ROTATE, MAX_STEP_ROTATE), MIN_STEP_ROTATE);
    RCLCPP_INFO(get_logger(), "decrease step size to %f", _stepDisplace);
}