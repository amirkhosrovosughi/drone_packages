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
    // Use a launch-provided run id when available so all nodes share one log folder.
    std::string run_id;
    const char * run_id_env = std::getenv("DATA_LOGGER_RUN_ID");

    if (run_id_env != nullptr && std::string(run_id_env).size() > 0)
    {
      run_id = run_id_env;
    }
    else
    {
      // Fallback: one folder per process initialization.
      auto now = std::chrono::system_clock::now();
      auto now_time = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");
      run_id = ss.str();
    }

    std::filesystem::path base_folder = std::filesystem::path(std::getenv("HOME")) / "ros_data_logging";

#if DATA_LOGGER_KEEP_HISTORY
    _sessionFolder = base_folder / run_id;
#else
    _sessionFolder = base_folder / "latest";
#endif

    std::cout << "[DataLogger] session folder: " << _sessionFolder << std::endl;

    try
    {
      std::filesystem::create_directories(_sessionFolder);

#if !DATA_LOGGER_KEEP_HISTORY
      // Clear stale files only once per run id to avoid deleting files from peers in the same launch.
      std::filesystem::path marker_path = _sessionFolder / ".run_id";
      std::string previous_run_id;

      if (std::filesystem::exists(marker_path))
      {
        std::ifstream marker_in(marker_path);
        std::getline(marker_in, previous_run_id);
      }

      if (previous_run_id != run_id)
      {
        for (const auto & entry : std::filesystem::directory_iterator(_sessionFolder))
        {
          if (entry.path().filename() == ".run_id")
          {
            continue;
          }
          std::filesystem::remove_all(entry.path());
        }

        std::ofstream marker_out(marker_path, std::ios::trunc);
        marker_out << run_id;
      }
#endif

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
    // Use stable filenames so each key always maps to one CSV per run folder.
    std::filesystem::path file_path = _sessionFolder / (key + ".csv");

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
