#include "Logger.h"
#include <mutex>

std::mutex Logger::mCoutMutex;

std::ostream& operator<<(std::ostream& os, const Logger& log)
{
  Logger::mCoutMutex.lock();
  os << std::endl << log.mPrefix << ": " << log.mStream.str();
  Logger::mCoutMutex.unlock();
  return os;
}
