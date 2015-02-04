#include "Logger.h"


std::ostream& operator<<(std::ostream& os, const Logger& log)
{
  os << std::endl << log.mPrefix << ": " << log.mStream.str();
  return os;
}
