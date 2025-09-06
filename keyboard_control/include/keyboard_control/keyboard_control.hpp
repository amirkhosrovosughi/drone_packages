// keyboard_control.hpp
#ifndef KEYBOARD_CONTROL_HPP_
#define KEYBOARD_CONTROL_HPP_

/**
 * @file keyboard_control.hpp
 * @brief ROS 2 node for sending manual keyboard commands to a drone.
 *
 * Publishes @ref drone_msgs::msg::DroneDirectionCommand and calls
 * @ref drone_msgs::srv::DroneMode based on key presses or CLI commands.
 */

#include <cstdint>
#include <map>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <drone_msgs/msg/drone_direction_command.hpp>
#include <drone_msgs/srv/drone_mode.hpp>

#ifdef BUILD_TESTING
  #include <gtest/gtest_prod.h>  // for FRIEND_TEST
#endif

/**
 * @brief Movement-related commands driven by keyboard/CLI.
 */
enum class MovementCommand
{
    CLI_COMMAND = 0,   //!< A structured CLI command (see _cliCommandMap)
    GO_UP = 1,         //!< Increase altitude
    GO_DOWN = 2,       //!< Decrease altitude
    GO_FORWARD = 3,    //!< Move forward (+x)
    GO_BACKWARD = 4,   //!< Move backward (-x)
    GO_RIGHT = 5,      //!< Move right (+y)
    GO_LEFT = 6,       //!< Move left (-y)
    ROTATE_CW = 7,     //!< Rotate clockwise (-yaw in ENU)
    ROTATE_ACW = 8     //!< Rotate anti-clockwise (+yaw in ENU)
};

/**
 * @brief General (non-motion) commands and speed adjustments.
 */
enum class GeneralCommand
{
    SLEEP_MODE = 0,      //!< Set controller to sleep/idle mode
    MANUAL_MODE = 1,     //!< Manual/teleop mode
    AUTOMATIC_MODE = 2,  //!< Automatic/mission mode
    INCREASE_SPEED = 3,  //!< Increase displacement & rotation step sizes
    DECREASE_SPEED = 4   //!< Decrease displacement & rotation step sizes
};

/**
 * @brief Keyboard teleop and CLI command node.
 *
 * By default, the constructor starts reading from the terminal immediately.
 * For tests, pass @p autostart = false to avoid the interactive loop.
 */
class KeyboardControl : public rclcpp::Node {
public:
    /**
     * @brief Construct the node.
     * @param autostart If true, immediately start reading keyboard input.
     */
    explicit KeyboardControl(bool autostart = true);

private:
    /// Begin interactive terminal loop (blocking until shutdown).
    void startKeyboardControl();

    /// Handle a single key press (already uppercased).
    void processKeyboardInput(char key);

    /// Publish a movement command with no CLI payload.
    void processMovementCommand(MovementCommand command);

    /// Publish a movement command with CLI payload.
    void processMovementCommand(MovementCommand command, uint8_t cliCommand, std::string cliCommandValue);

    /// Handle general (non-motion) commands.
    void processGeneralCommand(GeneralCommand command);

    /// Increase step sizes (bounded).
    void increaseSpeed();

    /// Decrease step sizes (bounded).
    void decreaseSpeed();

    /// Send mode change via service request.
    void sendRequest(uint16_t mode);

    /// Ask user to enter a CLI command (enables canonical mode temporarily).
    void startCliCommandReceive();

    /// Print prompt and mark the node to receive CLI input.
    void getCommand();

    /// Read one full line as CLI command (blocking).
    std::string getCliCommand();

    /**
     * @brief Parse a CLI input string.
     * @param cliInput Raw line (e.g., "goto 1.0,2.0,3.0").
     * @param commandValue Receives the value part after the first space.
     * @return Integer code of the command (see _cliCommandMap). Returns 0 if unknown.
     */
    int parseCliCommand(const std::string &cliInput, std::string &commandValue);

private:
    rclcpp::Publisher<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandPublisher;
    rclcpp::Client<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandClient;

    float _stepDisplace;
    float _stepRotate;
    bool _takeCliCommand = false;

    /// Known CLI commands mapped to integer codes.
    const std::map<std::string, int> _cliCommandMap = {
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

#ifdef BUILD_TESTING
    FRIEND_TEST(KeyboardControlTest, ParseCliCommand);
    FRIEND_TEST(KeyboardControlTest, SpeedClamping);
    FRIEND_TEST(KeyboardControlTest, PublishesMovement);
#endif
};

#endif  // KEYBOARD_CONTROL_HPP_