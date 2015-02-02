#pragma once
#include <string>
#include <ostream>

#define LOG_ENABLE        (true)

#define LOG_ENABLE_ERROR  (true)
#define LOG_ENABLE_DEBUG  (true)
#define LOG_ENABLE_VERBOSE (true)

class Logger
{
public:
  Logger(std::string prefix, bool enabled) : mPrefix(prefix), mEnabled(enabled) {};

  template <class T>
  Logger& operator<<(const T &data)
  {
    if (mEnabled && LOG_ENABLE)
      cout << mPrefix << ": " << data << endl;
    std::string(__FUNCTION__);
    return *this;
  }
private:
  std::string mPrefix;
  bool mEnabled;
};

#define LOG_ERROR   Logger("[ERROR][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_DEBUG   Logger("[DEBUG][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)
#define LOG_VERBOSE Logger("[VERBOSE][" + std::string(__FILE__) + ":L" + std::to_string(__LINE__) + "]", true)