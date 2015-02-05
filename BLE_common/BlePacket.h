#pragma once
#include "RadioPacket.h"

typedef enum
{
  BLE_PACKET_TYPE_ADV_IND,
  BLE_PACKET_TYPE_ADV_DIRECT_IND,
  BLE_PACKET_TYPE_ADV_NONCONN_IND,
  BLE_PACKET_TYPE_SCAN_REQ,
  BLE_PACKET_TYPE_SCAN_RSP,
  BLE_PACKET_TYPE_CONN_REQ,
  BLE_PACKET_TYPE_ADV_DISCOVER_IND
} ble_packet_type_t;


class BlePacket : public RadioPacket
{
public:
  BlePacket();
  ~BlePacket();

  
};

