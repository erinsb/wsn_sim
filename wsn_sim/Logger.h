#pragma once
#include <string>
#include <iostream>
#include <sstream>
#include <mutex>
#include <stdarg.h>

#define LOG_ENABLE        (0)

#define LOG_ENABLE_ERROR  (1)
#define LOG_ENABLE_DEBUG  (1)
#define LOG_ENABLE_VERBOSE (1)

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

#define LOG_ERROR   std::cout << Logger("[ERROR][" + std::string(__FUNCTION__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_DEBUG   std::cout << Logger("[DEBUG][" + std::string(__FUNCTION__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_VERBOSE std::cout << Logger("[VERBOSE][" + std::string(__FUNCTION__) + ":L" + std::to_string(__LINE__) + "]", true)

#define LOGGER(prefix) std::cout << Logger(prefix, true)

#if LOG_ENABLE
  #define _LOG(str, ...) printf("[%s]: " str "\n", __FUNCTION__, __VA_ARGS__)
#else
  #define _LOG(str, ...) 
#endif