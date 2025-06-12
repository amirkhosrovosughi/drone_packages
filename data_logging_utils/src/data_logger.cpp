#include "data_logging_utils/data_logger.hpp"
#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace data_logging_utils
{

// Static member initialization
std::map<std::string, std::ofstream> DataLogger::_files;
std::mutex DataLogger::_mutex;
std::filesystem::path DataLogger::_sessionFolder;
std::atomic<bool> DataLogger::_initialized = false;

void DataLogger::initialize()
{
  std::cout.flush();

  if (!_initialized)
  {
    // Session folder: ~/ros_data_logging/YYYY-MM-DD_HH-MM-SS
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");
    std::string folder_name = ss.str();

    std::filesystem::path base_folder = std::filesystem::path(std::getenv("HOME")) / "ros_data_logging";
    _sessionFolder = base_folder / folder_name;

    std::cout << "[DataLogger] session folder: " << _sessionFolder << std::endl;

    try
    {
      std::filesystem::create_directories(_sessionFolder);
      _initialized = true;
    }
    catch (const std::exception & e)
    {
      std::cerr << "[DataLogger] Failed to create log folder: " << e.what() << std::endl;
    }
  }
}

void DataLogger::log(const std::string &key, double value)
{
  if (!_initialized)
  {
    initialize();
  }
  auto timestamp = getCurrentTimestamp();
  log(key, value, timestamp);
}

void DataLogger::log(const std::string &key, double value, const std::string& timestamp)
{
  std::lock_guard<std::mutex> lock(_mutex);

  createFileIfNeeded(key);

  try
  {
    _files[key] << timestamp << "," << value << std::endl;
  }
  catch (const std::exception & e)
  {
    std::cerr << "[DataLogger] Failed to write to file for key '" << key << "': " << e.what() << std::endl;
  }
}

void DataLogger::log(const std::map<std::string, double>& values)
{
  if (!_initialized)
  {
    initialize();
  }
  auto timestamp = getCurrentTimestamp();
  log(values, timestamp);
}

void DataLogger::log(const std::map<std::string, double>& values, const std::string& timestamp)
{
  std::lock_guard<std::mutex> lock(_mutex);

  for (const auto & [key, value] : values)
  {
    createFileIfNeeded(key);

    try
    {
        _files[key] << timestamp << "," << value << std::endl;
    }
    catch (const std::exception & e)
    {
      std::cerr << "[DataLogger] Failed to write to file for key '" << key << "': " << e.what() << std::endl;
    }
  }
}

void DataLogger::createFileIfNeeded(const std::string & key)
{
  if (_files.find(key) == _files.end())
  {
    // New file name: key_YYYY-MM-DD_HH-MM-SS.csv
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << key << "_";
    ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S") << ".csv";

    std::filesystem::path file_path = _sessionFolder / ss.str();

    try
    {
        _files[key].open(file_path, std::ios::out);
        _files[key] << "Timestamp,Value" << std::endl;  // CSV header
    }
    catch (const std::exception & e)
    {
      std::cerr << "[DataLogger] Failed to create file for key '" << key << "': " << e.what() << std::endl;
    }
  }
}

std::string DataLogger::getCurrentTimestamp()
{
  auto now = std::chrono::system_clock::now();
  auto now_time = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S");
  ss << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

std::filesystem::path DataLogger::getSessionFolder()
{
  return _sessionFolder;
}

}  // namespace data_logging_utils
