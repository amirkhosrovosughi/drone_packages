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
    /**
     * @brief Begin interactive terminal loop.
     *
     * Blocking until shutdown. Continuously polls keyboard input and dispatches commands.
     */
    void startKeyboardControl();

    /**
     * @brief Handle a single key press.
     *
     * @param key The uppercase key character pressed.
     */
    void processKeyboardInput(char key);

    /**
     * @brief Publish a movement command without CLI payload.
     *
     * @param command Movement command enum.
     */
    void processMovementCommand(MovementCommand command);

    /**
     * @brief Publish a movement command with CLI payload.
     *
     * @param command Movement command enum.
     * @param cliCommand CLI command type (e.g., go to position).
     * @param cliCommandValue String payload associated with CLI command.
     */
    void processMovementCommand(MovementCommand command, uint8_t cliCommand, std::string cliCommandValue);

    /**
     * @brief Handle a general (non-motion) command.
     *
     * @param command General command enum.
     */
    void processGeneralCommand(GeneralCommand command);

    /**
     * @brief Increase translational and rotational step sizes (bounded).
     */
    void increaseSpeed();

    /**
     * @brief Decrease translational and rotational step sizes (bounded).
     */
    void decreaseSpeed();

    /**
     * @brief Send a mode change request via service.
     *
     * @param mode Drone mode (PX4 command or custom enum).
     */
    void sendRequest(uint16_t mode);

    /**
     * @brief Ask user to enter a CLI command.
     *
     * Temporarily enables canonical terminal mode to capture full-line input.
     */
    void startCliCommandReceive();

    /**
     * @brief Print CLI prompt and mark the node to receive CLI input.
     */
    void getCommand();

    /**
     * @brief Read a full line as CLI command.
     *
     * Blocking until newline is entered.
     * @return String containing the CLI command.
     */
    std::string getCliCommand();

     /**
     * @brief Parse a CLI input string.
     * @param cliInput Raw line (e.g., "goto 1.0,2.0,3.0").
     * @param commandValue Receives the value part after the first space.
     * @return Integer code of the command (see _cliCommandMap). Returns 0 if unknown.
     */
    int parseCliCommand(const std::string &cliInput, std::string &commandValue);

private:
    rclcpp::Publisher<drone_msgs::msg::DroneDirectionCommand>::SharedPtr _moveCommandPublisher; ///< Publisher for drone direction commands.
    rclcpp::Client<drone_msgs::srv::DroneMode>::SharedPtr _generalCommandClient;               ///< Client for sending general drone mode commands (e.g., arm, disarm).

    float _stepDisplace; ///< Step size for translational movements (meters).
    float _stepRotate;   ///< Step size for rotational movements (radians).
    bool _takeCliCommand = false; ///< Flag indicating whether CLI commands are enabled.

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