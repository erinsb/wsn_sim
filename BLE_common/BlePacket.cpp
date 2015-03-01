#include "BlePacket.h"

bool operator==(ble_adv_addr_t const& left, ble_adv_addr_t const& right)
{
  return (memcmp(left.arr, right.arr, BLE_ADV_ADDR_LEN) == 0);
}

bool operator !=(ble_adv_addr_t const& left, ble_adv_addr_t const& right)
{
  return !(left == right);
}

#if 0
BlePacket::BlePacket()
{
}


BlePacket::~BlePacket()
{
}

#endif