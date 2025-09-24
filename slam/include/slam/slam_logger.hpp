#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sstream>

// Logging levels
enum class LogLevel
{
    DEBUG = 0,
    INFO,
    WARN,
    ERROR
};

/**
 * @class SlamLogger
 * @brief Lightweight logging wrapper for ROS 2 logger with support for multiple log levels.
 */
class SlamLogger
{
public:
    /**
     * @brief Construct a new SlamLogger object.
     * @param logger ROS 2 logger to use.
     */
    explicit SlamLogger(const rclcpp::Logger& logger) : _logger(logger) {}

    /**
     * @brief Template method to accept stream-like inputs to log information message.
     */
    template <typename... Args>
    void logInfo(Args&&... args)
    {
        RCLCPP_INFO(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    /**
     * @brief Template method to accept stream-like inputs to log warning message.
     */
    template <typename... Args>
    void logWarn(Args&&... args)
    {
        RCLCPP_WARN(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    /**
     * @brief Template method to accept stream-like inputs to log error message.
     */
    template <typename... Args>
    void logError(Args&&... args)
    {
        RCLCPP_ERROR(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    /**
     * @brief Template method to accept stream-like inputs to log debug message.
     */
    template <typename... Args>
    void logDebug(Args&&... args)
    {
        RCLCPP_DEBUG(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    /**
     * @brief Generic log method with specified log level.
     * @param level Log level.
     * @param args Message components.
     */
    template <typename... Args>
    void log(LogLevel level, Args&&... args)
    {
        switch (level)
        {
        case LogLevel::DEBUG:
            logDebug(std::forward<Args>(args)...);
            break;
        case LogLevel::INFO:
            logInfo(std::forward<Args>(args)...);
            break;
        case LogLevel::WARN:
            logWarn(std::forward<Args>(args)...);
            break;
        case LogLevel::ERROR:
            logError(std::forward<Args>(args)...);
            break;
        default:
            logInfo(std::forward<Args>(args)...);
            break;
        }
    }

private:
    rclcpp::Logger _logger;  ///< Underlying ROS 2 logger instance.

    /**
     * @brief Helper function to build a concatenated string message from multiple arguments.
     */
    template <typename... Args>
    std::string buildMessage(Args&&... args)
    {
        std::ostringstream stream;
        (stream << ... << args); // Fold expression to concatenate inputs
        return stream.str();
    }
};

using LoggerPtr = std::shared_ptr<SlamLogger>;
