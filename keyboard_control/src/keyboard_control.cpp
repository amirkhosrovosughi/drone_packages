// keyboard_control.cpp
#include "keyboard_control.hpp"
#include <termios.h>
#include <iostream>
// #include <rclcpp/executors.hpp>

static const float defaultStepDisplace = 2.0f;
static const float maxStepDisplace = 10.0f;
static const float minStepDisplace = 0.5f;
static const float changeStepDisplace = 0.5f;

static const float defaultStepRotate = 0.2f;
static const float maxStepRotate = 1.0f;
static const float minStepRotate = 0.05f;
static const float changeStepRotate = 0.05f;


KeyboardControl::KeyboardControl() : Node("keyboard_control") {
    // Initialize publisher
    _moveCommandPublisher = this->create_publisher<drone_msgs::msg::DroneDirectionCommand>("/keyboard_control/movement_command", 10);
    _generalCommandClient = create_client<drone_msgs::srv::DroneMode>("/keyboard_control/general_command");

    _stepDisplace = defaultStepDisplace;
    _stepRotate = defaultStepRotate;
    // Start listening to the keyboard
    startKeyboardControl();
}

void KeyboardControl::startKeyboardControl() {
    struct termios oldt, newt;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);

    // Apply new terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    while (rclcpp::ok()) {
        // Read a character from the keyboard
        std::cin >> c;

        processKeyboardInput(c);
    }

    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

void KeyboardControl::processKeyboardInput(char key) {
    // Convert the character to uppercase for case-insensitive comparison
    key = std::toupper(key);
    

    // Process the key based on the enums
    switch (key) {
        case 'Z':  // space key
            processMovementCommand(MovementCommand::STOP);
            break;
        case 'W':
            processMovementCommand(MovementCommand::GO_UP);
            break;
        case 'S':
            processMovementCommand(MovementCommand::GO_DOWN);
            break;
        case 'A':
            processMovementCommand(MovementCommand::ROTATE_ACW);
            break;
        case 'D':
            processMovementCommand(MovementCommand::ROTATE_CW);
            break;
        case 'U':
            processMovementCommand(MovementCommand::GO_FORWARD);
            break;
        case 'J':
            processMovementCommand(MovementCommand::GO_BACKWARD);
            break;
        case 'H':
            processMovementCommand(MovementCommand::GO_LEFT);
            break;
        case 'K':
            processMovementCommand(MovementCommand::GO_RIGHT);
            break;
        case '0':
            processGeneralCommand(GeneralCommand::SLEEP_MODE);
            break;
        case '1':
            processGeneralCommand(GeneralCommand::MANUAL_MODE);
            break;
        case '2':
            processGeneralCommand(GeneralCommand::AUTOMATIC_MODE);
            break;
        case '=':
            processGeneralCommand(GeneralCommand::INCREASE_SPEED);
            break;
        case '-':
            processGeneralCommand(GeneralCommand::DECREASE_SPEED);
            break;
        default:
            // Handle unrecognized keys
            break;
    }
}

void KeyboardControl::processMovementCommand(MovementCommand command) {
    auto directionCommand = drone_msgs::msg::DroneDirectionCommand();

    switch (command) {
        case MovementCommand::STOP:
            directionCommand.stop = true;
            break;

        case MovementCommand::GO_UP:
            // Handle GO_UP command
            directionCommand.vertical_step = _stepDisplace;
            break;

        case MovementCommand::GO_DOWN:
            // Handle GO_DOWN command
            directionCommand.vertical_step = _stepDisplace * -1.0f;
            break;

        case MovementCommand::GO_FORWARD:
            // Handle GO_FORWARD command
            directionCommand.forward_step = _stepDisplace;
            break;

        case MovementCommand::GO_BACKWARD:
            // Handle GO_FORWARD command
            directionCommand.forward_step = _stepDisplace * -1.0f;
            break;

        case MovementCommand::GO_RIGHT:
            // Handle GO_FORWARD command
            directionCommand.lateral_step = _stepDisplace;
            break;

        case MovementCommand::GO_LEFT:
            // Handle GO_FORWARD command
            directionCommand.lateral_step = _stepDisplace * -1.0f;
            break;

        case MovementCommand::ROTATE_CW:
            // Handle GO_FORWARD command
            directionCommand.angular_step = _stepRotate;
            break;

        case MovementCommand::ROTATE_ACW:
            // Handle GO_FORWARD command
            directionCommand.angular_step = _stepRotate * -1.0f;
            break;

        default:
            // Handle unknown command
            break;
    }

    _moveCommandPublisher->publish(directionCommand);
}

void KeyboardControl::processGeneralCommand(GeneralCommand command) {
    // Implement the logic for processing general commands
    switch (command)
    {
        case GeneralCommand::SLEEP_MODE:
            sendRequest(0);
            break;

        case GeneralCommand::MANUAL_MODE:
            sendRequest(1);
            break;

        case GeneralCommand::AUTOMATIC_MODE:
            sendRequest(2);
            break;

        case GeneralCommand::INCREASE_SPEED:
            increaseSpeed();
            break;

        case GeneralCommand::DECREASE_SPEED:
            decreaseSpeed();
            break;

        default:
            // Handle unknown command
            break;
    }
}

// TODO, convert it to simple message publisher
void KeyboardControl::sendRequest(uint16_t mode)
{
    RCLCPP_INFO(get_logger(), "send request: %d", mode);
    auto request = std::make_shared<drone_msgs::srv::DroneMode::Request>();
    request->mode = mode;
    auto result = _generalCommandClient->async_send_request(request);

    // Use a timer to check for a response or timeout
    auto timer = create_wall_timer(
        std::chrono::seconds(10),
            [&, mode]() {
        if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            auto response = result.get();
            if (response->success)
            {
                RCLCPP_INFO(get_logger(), "Successfully set mode to %u", mode);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Failed to set mode to %u", mode);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call timed out. Unable to set mode to %u", mode);
        }
    });
}

void KeyboardControl::increaseSpeed()
{
    _stepDisplace = std::max(std::min(_stepDisplace + changeStepDisplace, maxStepDisplace), minStepDisplace);
    _stepRotate = std::max(std::min(_stepRotate + changeStepRotate, maxStepRotate), minStepRotate);
    RCLCPP_INFO(get_logger(), "increase step size to %f", _stepDisplace);
}

void KeyboardControl::decreaseSpeed()
{
    _stepDisplace = std::max(std::min(_stepDisplace - changeStepDisplace, maxStepDisplace), minStepDisplace);
    _stepRotate = std::max(std::min(_stepRotate - changeStepRotate, maxStepRotate), minStepRotate);
    RCLCPP_INFO(get_logger(), "decrease step size to %f", _stepDisplace);
}
