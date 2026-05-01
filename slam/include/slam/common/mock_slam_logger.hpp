#ifndef SLAM__COMMON__MOCK_SLAM_LOGGER_HPP_
#define SLAM__COMMON__MOCK_SLAM_LOGGER_HPP_

#include <string>
#include "common/slam_logger.hpp"

class MockSlamLogger : public SlamLogger
{
public:
    MockSlamLogger() = default;

protected:
    void write(LogLevel /*level*/, const std::string& /*message*/) override
    {
        // Intentionally no-op for unit tests and non-ROS builds
    }
};

#endif  // SLAM__COMMON__MOCK_SLAM_LOGGER_HPP_
