#pragma once
#include "RadioPacket.h"
#include <stdint.h>

#define BLE_PACKET_OVERHEAD_LENGTH  (6)
#define BLE_ADV_ADDR_LEN            (6)

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

typedef struct
{
  uint8_t type : 4;
  uint8_t _rfu1 : 2;
  uint8_t addr_type : 2;
  uint8_t length;
} ble_packet_header_t;

typedef union ble_adv_addr_t
{
  //uint64_t raw : 48;
  uint8_t arr[6];
  bool isNull(void);
  void set(union ble_adv_addr_t& adv_addr) { memcpy(arr, adv_addr.arr, BLE_ADV_ADDR_LEN); }
  void clear(void) { memset(arr, 0, BLE_ADV_ADDR_LEN); }
  std::string toString(void);
} ble_adv_addr_t;
#pragma pack(pop)

bool operator ==(ble_adv_addr_t const& left, ble_adv_addr_t const& right);
bool operator !=(ble_adv_addr_t const& left, ble_adv_addr_t const& right);

#pragma pack(push, 1)
typedef struct
{
  uint32_t access_addr;
  ble_packet_header_t header;
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
}ble_adv_packet_t;
#pragma pack(pop)
