#pragma once
#include "RadioPacket.h"
#include <stdint.h>

#define BLE_PACKET_OVERHEAD_LENGTH (6)

#pragma pack(push, 1)
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

typedef union
{
  uint64_t raw : 48;
  uint8_t arr[6];
} ble_adv_addr_t;

typedef struct
{
  uint32_t access_addr;
  ble_packet_type_t type : 4;
  uint8_t _rfu1 : 2;
  uint8_t addr_type : 2;
  uint8_t length;
  ble_adv_addr_t adv_addr;
  union
  {
    struct
    {
      uint8_t raw[31];
    } adv;
    struct
    {
      uint8_t raw[31];
    } scan_rsp;
    struct
    {
      ble_adv_addr_t scan_addr;
    } scan_req;
    struct
    {
      ble_adv_addr_t initiator_addr;
    }adv_direct;
    struct
    {
      uint32_t access_addr;
      uint8_t crc_init[3];
      uint8_t win_size;
      uint16_t win_offset;
      uint16_t interval;
      uint16_t latency;
      uint16_t timeout;
      uint8_t channel_map[5];
      uint8_t hop : 5;
      uint8_t sca : 3;
    }conn_req;
  } payload;

  void setAdvAddr(uint8_t* addr) { memcpy(adv_addr.arr, addr, 6); }
  void setAdvAddr(uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4, uint8_t a5) 
  {
    uint8_t temp[] = { a0, a1, a2, a3, a4, a5 };
    memcpy(adv_addr.arr, temp, 6); 
  }
  void setAdvAddr(uint64_t addr) 
  { 
    adv_addr.raw = addr & 0xFFFFFFFFFFFF0000;
  }
}ble_adv_packet_t;
#pragma pack(pop)

#if 0
class BlePacket
{
public:
  BlePacket(uint32_t accessAddr, uint8_t payloadLength = 0, uint8_t* rawPayload = NULL);
  ~BlePacket();

  virtual uint8_t* raw(void);
  virtual void parseFrom(uint8_t* data, uint16_t length);
  uint8_t length;
  
private:
  uint32_t accessAddr;
  ble_packet_type_t type;
  uint8_t payload[255];  
};

typedef struct
{
  uint8_t len;
  uint16_t type;
  uint8_t data[25];
}ble_adv_data_t;

class AdvPacket : BlePacket
{
public:
  AdvPacket(uint32_t accessAddr, ble_adv_addr_t adv_addr) : BlePacket(accessAddr) { memcpy(mAdvAddr, adv_addr, 6); }

  void advDataAdd(ble_adv_data_t& adv_data);
  void advDataRemove(uint16_t type);
  bool advDataInPacket(uint16_t type);

private:
  ble_adv_addr_t mAdvAddr;
  std::vector<ble_adv_data_t> mAdvDataEntries;
};
#endif