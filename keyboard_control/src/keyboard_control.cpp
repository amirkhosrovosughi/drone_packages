// keyboard_control.cpp
#include "keyboard_control.hpp"
#include <termios.h>
#include <iostream>
// #include <rclcpp/executors.hpp>

static const float defaultStepDisplace = 0.2f;
static const float maxStepDisplace = 2.0f;
static const float minStepDisplace = 0.05f;
static const float changeStepDisplace = 0.05f;

static const float defaultStepRotate = 0.05f;
static const float maxStepRotate = 0.5f;
static const float minStepRotate = 0.01f;
static const float changeStepRotate = 0.01f;


KeyboardControl::KeyboardControl() : Node("keyboard_control")
{
    // Initialize publisher
    _moveCommandPublisher = this->create_publisher<drone_msgs::msg::DroneDirectionCommand>("/keyboard_control/movement_command", 10);
    _generalCommandClient = create_client<drone_msgs::srv::DroneMode>("/keyboard_control/general_command");

    _stepDisplace = defaultStepDisplace;
    _stepRotate = defaultStepRotate;
    // Start listening to the keyboard
    startKeyboardControl();
}

void KeyboardControl::startKeyboardControl()
{
    struct termios oldt, newt;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Disable canonical mode and echo
    newt.c_lflag &= ~(ICANON | ECHO);

    // Apply new terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char c;
    while (rclcpp::ok())
    {
        if (!_takeCLICommand)
        {
            // Read a character from the keyboard
            std::cin >> c;

            processKeyboardInput(c);
        }
        else
        {
            startCliCommandReceive();
        }
    }

    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}


void KeyboardControl::startCliCommandReceive()
{
    std::string cli_input = getClicommand();
    std::string command_value;
    int cli_command = parseCliCommand(cli_input, command_value);
    processMovementCommand(MovementCommand::CLI_COMMAND, cli_command, command_value);
    _takeCLICommand = false;
}

int KeyboardControl::parseCliCommand(const std::string &cli_input, std::string &command_value)
{
    // Split the input string at the first space
    std::istringstream iss(cli_input);
    std::string command;
    iss >> command;

    // Extract the rest of the input as command_value
    std::getline(iss, command_value);

    // Remove leading spaces from command_value
    if (!command_value.empty() && command_value[0] == ' ') {
        command_value.erase(0, 1);
    }

    // Look up the command in the map
    auto it = _cliCommandMap.find(command);
    if (it != _cliCommandMap.end()) {
        std::cout << "command: " << it->second << " command_value: " << command_value << "\n";
        return it->second;
    } else {
        command_value = "";  // If command not found, set command_value to an empty string
        std::cout << "command: " << it->second << " command_value: " << command_value << "\n";
        return 0;  // Return 0 if the command is not found
    }
}

void KeyboardControl::processKeyboardInput(char key)
{
    // Convert the character to uppercase for case-insensitive comparison
    key = std::toupper(key);
    

    // Process the key based on the enums
    switch (key) {
        case 'C':  // space key
            // processMovementCommand(MovementCommand::STOP);
            getCommand();
            break;
        case 'W':
            processMovementCommand(MovementCommand::GO_UP);
            break;
        case 'S':
            processMovementCommand(MovementCommand::GO_DOWN);
            break;
        case 'A':
            processMovementCommand(MovementCommand::ROTATE_ACW);
            RCLCPP_INFO(get_logger(), "pressed A");
            break;
        case 'D':
            processMovementCommand(MovementCommand::ROTATE_CW);
            RCLCPP_INFO(get_logger(), "pressed D");
            break;
        case 'U':
            processMovementCommand(MovementCommand::GO_FORWARD);
            RCLCPP_INFO(get_logger(), "pressed U");
            break;
        case 'J':
            processMovementCommand(MovementCommand::GO_BACKWARD);
            RCLCPP_INFO(get_logger(), "pressed J");
            break;
        case 'H':
            processMovementCommand(MovementCommand::GO_LEFT);
            RCLCPP_INFO(get_logger(), "pressed H");
            break;
        case 'K':
            processMovementCommand(MovementCommand::GO_RIGHT);
            RCLCPP_INFO(get_logger(), "pressed K");
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

void KeyboardControl::getCommand()
{
    _takeCLICommand = true;
    RCLCPP_INFO(get_logger(), "Enter your command:");
    std::cout << "Enter your command:";
}

std::string KeyboardControl::getClicommand()
{
    struct termios oldt, newt;
    std::string command;

    // Save current terminal settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Enable canonical mode and echo
    newt.c_lflag |= (ICANON | ECHO);

    // Apply new terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read input from user
    std::getline(std::cin, command);

    // Restore original terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    // Check for escape key (ASCII 27)
    if (!command.empty() && command[0] == 27) {
        return "";
    }

    return command;
}

void KeyboardControl::processMovementCommand(MovementCommand command)
{
    processMovementCommand(command, 0, "");
}

void KeyboardControl::processMovementCommand(MovementCommand command, uint8_t cli_command, std::string cli_command_value)
{
    auto directionCommand = drone_msgs::msg::DroneDirectionCommand();

    switch (command) {
        case MovementCommand::CLI_COMMAND:
            directionCommand.cli_command = cli_command;
            directionCommand.cli_command_value = cli_command_value;
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

void KeyboardControl::processGeneralCommand(GeneralCommand command)
{
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
