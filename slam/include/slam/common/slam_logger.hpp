#pragma once

#include <memory>
#include <sstream>
#include <string>

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
 * @brief Abstract, ROS-agnostic logger interface.
 *
 * This base class provides a templated, stream-style `log(...)` API that
 * forwards the formatted message to the pure-virtual `write(...)` method.
 * Implementations (ROS or mock) only need to implement `write`.
 */
class SlamLogger
{
public:
    virtual ~SlamLogger() = default;

    template <typename... Args>
    void logInfo(Args&&... args)
    {
        write(LogLevel::INFO, buildMessage(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void logWarn(Args&&... args)
    {
        write(LogLevel::WARN, buildMessage(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void logError(Args&&... args)
    {
        write(LogLevel::ERROR, buildMessage(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void logDebug(Args&&... args)
    {
        write(LogLevel::DEBUG, buildMessage(std::forward<Args>(args)...));
    }

    template <typename... Args>
    void log(LogLevel level, Args&&... args)
    {
        write(level, buildMessage(std::forward<Args>(args)...));
    }

protected:
    virtual void write(LogLevel level, const std::string& message) = 0;

private:
    template <typename... Args>
    std::string buildMessage(Args&&... args)
    {
        std::ostringstream stream;
        (stream << ... << args);
        return stream.str();
    }
};

using LoggerPtr = std::shared_ptr<SlamLogger>;
