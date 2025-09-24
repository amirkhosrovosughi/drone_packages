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

class SlamLogger
{
public:
    explicit SlamLogger(const rclcpp::Logger& logger) : _logger(logger) {}

    // Template method to accept stream-like inputs
    template <typename... Args>
    void logInfo(Args&&... args)
    {
        RCLCPP_INFO(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    template <typename... Args>
    void logWarn(Args&&... args)
    {
        RCLCPP_WARN(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    template <typename... Args>
    void logError(Args&&... args)
    {
        RCLCPP_ERROR(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    template <typename... Args>
    void logDebug(Args&&... args)
    {
        RCLCPP_DEBUG(_logger, "%s", buildMessage(std::forward<Args>(args)...).c_str());
    }

    // Unified log method with level support
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
    rclcpp::Logger _logger;

    // Helper function to build a string message from stream-like arguments
    template <typename... Args>
    std::string buildMessage(Args&&... args)
    {
        std::ostringstream stream;
        (stream << ... << args); // Fold expression to concatenate inputs
        return stream.str();
    }
};

using LoggerPtr = std::shared_ptr<SlamLogger>;
