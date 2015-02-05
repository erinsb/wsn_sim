#pragma once
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>

#define LOG_ENABLE        (true)

#define LOG_ENABLE_ERROR  (true)
#define LOG_ENABLE_DEBUG  (true)
#define LOG_ENABLE_VERBOSE (true)

class Logger
{
public:
  Logger(std::string prefix, bool enabled) : mPrefix(prefix), mEnabled(enabled) {};

  Logger& operator<<(const Logger& log)
  {
    mStream << log.mStream.str();
    return *this;
  }

  template <class T>
  Logger& operator<<(const T &data)
  {
    if (mEnabled && LOG_ENABLE)
      mStream << data;
    return *this;
  }


  friend std::ostream& operator<<(std::ostream& os, const Logger& log);
private:
  std::stringstream mStream;
  std::string mPrefix;
  bool mEnabled;
  static std::mutex mCoutMutex;
};

#define LOG_ERROR   std::cout << Logger("[ERROR][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_DEBUG   std::cout << Logger("[DEBUG][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_VERBOSE std::cout << Logger("[VERBOSE][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)

#define LOGGER(prefix) std::cout << Logger(prefix, true)