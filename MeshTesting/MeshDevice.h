#pragma once
#include "Device.h"
#include "RadioPacket.h"
#include "BlePacket.h"
#include <vector>
#include <queue>
#include <stdint.h>

#define MESH_INTERVAL           (100*MS)
#define MESH_MAX_SUBSCRIPTIONS  (5)
#define MESH_MAX_CLOCK_DRIFT    (250.0 * PPM)
#define MESH_CH_OFFSET_US(offs) (625 * ((uint32_t) offs)
#define MESH_MAX_ROUTES         (10)
#define MESH_UNKNOWN_DIST       (UINT32_MAX)

#define MESH_ACCESS_ADDR        (0xC0221E55)

class MeshDevice;

//mesh packet, based on ble adv
#pragma pack(push, 1)
typedef enum
{
  MESH_ADV_TYPE_DEFAULT = 0x50,
  MESH_ADV_TYPE_NEIGHBOR_NOTIFICATION,
  MESH_ADV_TYPE_ACK,
  MESH_ADV_TYPE_UNICAST,
  MESH_ADV_TYPE_BROADCAST
}mesh_adv_type_t;

typedef struct
{
  uint32_t access_addr;
  ble_packet_header_t header;
  ble_adv_addr_t adv_addr;
  union
  {
    uint8_t raw[250];
    struct
    {
      uint8_t adv_len;    // BLE compatibility
      mesh_adv_type_t adv_type;  // BLE compatibility
      union 
      {
        uint8_t raw[248];
        struct
        {
          ble_adv_addr_t clusterAddr; // address of cluster head
          uint16_t offsetFromCH;      // number of 625us slots offset
          uint8_t nbCount;            // number of other nodes this one is subscribed to
          uint8_t powerScore;         // based on power supply and stability
        } default;
        struct
        {
          ble_adv_addr_t neighborsCH; // address of neighbor's clusterhead
          ble_adv_addr_t neighborAddr;// address of neighbor
          uint32_t nextExpectedBeacon;// timeoffset from this msg to the next expected nb beacon
          uint8_t neighborScore;      // neighbor's powerScore
        } neighborNotification;
        union
        {
          struct
          {
            ble_adv_addr_t sender;    // multihop source
            ble_adv_addr_t dest;      // multihop destination
            uint32_t sender_dist;     // distance from sender
            uint32_t dest_dist;       // distance to destination
            uint8_t payload[228];     // rest of packet is payload
          } unicast;
          struct
          {
            ble_adv_addr_t sender;
            uint8_t payload[242];
          } broadcast;
        }data;
      } payload;
    } str;
  }payload;
} mesh_packet_t;
#pragma pack(pop)

typedef struct
{
  ble_adv_addr_t destination;
  uint32_t timeStamp;
  uint32_t distance;
} route_entry_t;

class MeshNeighbor
{
  friend MeshDevice;
public:
  MeshNeighbor(ble_adv_addr_t* addr);

  void receivedBeacon(uint32_t rxTime, mesh_packet_t* beacon);

  bool isClusterHead(void);

  uint8_t getPowerScore(void) { return mPowerScore; }
  uint32_t getLastSyncTime(void) { return mLastSyncTime; }
  uint32_t getNextBeaconTime(uint32_t timeNow);
    
  ble_adv_addr_t mAdvAddr;
  bool mFollowing;
private:
  ble_adv_addr_t mClusterHead;

  uint32_t mLastSyncTime;
  uint32_t mNbCount;
  uint8_t mPowerScore;
  uint32_t mBeaconCount;
  uint32_t mBeaconInterval;
};

class MeshDevice : public Device
{
public:
  MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x = 0.0, double y = 0.0);
  ~MeshDevice();
  void setAdvAddress(uint64_t addr);

  void startSearch(void);
  void stopSearch(void);

  void registerNeighbor(ble_adv_addr_t* advAddr, uint32_t rxTime);
  MeshNeighbor* getNeighbor(ble_adv_addr_t* advAddr);

  void transmit(uint8_t* data, uint32_t length);
  
  void setClusterHead(MeshNeighbor* nb);
  void setPowerScore(uint8_t score);
  void addRoute(ble_adv_addr_t* addr, uint32_t dist);
  
  uint32_t getDistanceTo(ble_adv_addr_t* addr);

private:
  route_entry_t mRoutes[MESH_MAX_ROUTES];
  std::vector<MeshNeighbor> mNeighbors;
  std::queue<ble_adv_packet_t*> mPacketQueue;
  std::vector<MeshNeighbor*> mSubscriptions;
  mesh_packet_t mDefaultPacket;
  bool mSearching;
  ble_adv_addr_t mMyAddr;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);

  route_entry_t* getWeakestRoute(void);
  MeshNeighbor* getStrongestSubscription(void);
};

  