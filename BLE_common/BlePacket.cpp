#include "BlePacket.h"
#include <string>
#include <sstream>
#include <iomanip>

bool ble_adv_addr_t::isNull(void)
{
  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    if (arr[i] != 0)
      return false;
  }
  return true;
}

std::string ble_adv_addr_t::toString(void)
{
  std::string str;
  std::stringstream stream;
  stream << "[";
  
  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    if (i > 0)
      stream << ":";
    stream << std::uppercase << std::hex << std::setfill('0') << std::setw(2);
    stream << (int)arr[i];
  }
  stream << "]";
  return stream.str();
}

bool operator==(ble_adv_addr_t const& left, ble_adv_addr_t const& right)
{
  return (memcmp(left.arr, right.arr, BLE_ADV_ADDR_LEN) == 0);
}

bool operator !=(ble_adv_addr_t const& left, ble_adv_addr_t const& right)
{
  return !(left == right);
}
