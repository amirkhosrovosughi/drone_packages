#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "common/slam_logger.hpp"

class RosSlamLogger : public SlamLogger
{
public:
    explicit RosSlamLogger(const rclcpp::Logger& logger)
    : _logger(logger)
    {}

protected:
    void write(LogLevel level, const std::string& message) override
    {
        switch (level)
        {
            case LogLevel::DEBUG:
                RCLCPP_DEBUG(_logger, "%s", message.c_str());
                break;
            case LogLevel::INFO:
                RCLCPP_INFO(_logger, "%s", message.c_str());
                break;
            case LogLevel::WARN:
                RCLCPP_WARN(_logger, "%s", message.c_str());
                break;
            case LogLevel::ERROR:
                RCLCPP_ERROR(_logger, "%s", message.c_str());
                break;
            default:
                RCLCPP_INFO(_logger, "%s", message.c_str());
                break;
        }
    }

private:
    rclcpp::Logger _logger;
};
