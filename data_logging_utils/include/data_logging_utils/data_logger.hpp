#ifndef DATA_LOGGING_UTILS__DATA_LOGGER_HPP_
#define DATA_LOGGING_UTILS__DATA_LOGGER_HPP_

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <mutex>
#include <memory>
#include <filesystem>
#include <atomic>

namespace data_logging_utils
{

class DataLogger
{
public:
  // Initialize logging folder with session timestamp
  static void initialize();

  // Log a single variable
  static void log(const std::string& key, double value);

  // Log multiple variables at the same time
  static void log(const std::map<std::string, double>& values);

private:
  // Private constructor for static class
  DataLogger() = default;

  // Helper to create a new file for a key if it doesn't exist
  static void createFileIfNeeded(const std::string & key);

  // Log a single variable
  static void log(const std::string& key, double value, const std::string& timestamp);

  // Log multiple variables at the same time
  static void log(const std::map<std::string, double>& values, const std::string& timestamp);

  // Generate ISO8601 timestamp
  static std::string getCurrentTimestamp();

  // Get the session folder
  static std::filesystem::path getSessionFolder();

  // Static members
  static std::map<std::string, std::ofstream> _files;
  static std::mutex _mutex;
  static std::filesystem::path _sessionFolder;
  static std::atomic<bool> _initialized;
};

}  // namespace data_logging_utils

#endif  // DATA_LOGGING_UTILS__DATA_LOGGER_HPP_