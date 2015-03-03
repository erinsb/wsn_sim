#pragma once
#include "Device.h"
#include "RadioPacket.h"
#include "BlePacket.h"
#include <vector>
#include <queue>
#include <stdint.h>

#define MESH_INTERVAL               (100*MS)
#define MESH_MAX_SUBSCRIPTIONS      (5)
#define MESH_MAX_CLOCK_DRIFT        (250.0 * PPM)
#define MESH_CH_OFFSET_US(offs)     (625 * ((uint32_t) offs)
#define MESH_CH_BEACON_MARGIN       (300)
#define MESH_MAX_ROUTES             (10)
#define MESH_UNKNOWN_DIST           (UINT32_MAX)
#define MESH_RX_RU_TIME             (180)       // realistic, considering xtal startup
#define MESH_RX_LISTEN_TIME         (400)
#define MESH_ROUTE_TIMEOUT          (1*MINUTES) //time before a route is deemed invalid
#define MESH_ACCESS_ADDR            (0xC0221E55)
#define MESH_MESSAGE_ID_CACHE_SIZE  (16)
#define MESH_CH_SWITCH_THRESHOLD    (20)        // required difference in ch weight before we change ch

#define MESH_PACKET_OVERHEAD           (sizeof(uint32_t) + sizeof(ble_packet_header_t) + BLE_ADV_ADDR_LEN + sizeof(uint8_t) + sizeof(mesh_adv_type_t) + sizeof(msgID_t))
#define MESH_PACKET_OVERHEAD_UNICAST   (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint32_t))
#define MESH_PACKET_OVERHEAD_BROADCAST (BLE_ADV_ADDR_LEN + sizeof(uint16_t))
#define MESH_PACKET_OVERHEAD_NBNOT     (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint8_t))
#define MESH_PACKET_OVERHEAD_DEFAULT   (BLE_ADV_ADDR_LEN + sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t))
#define MESH_PACKET_OVERHEAD_DIST_REQ  (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint16_t))
#define MESH_PACKET_OVERHEAD_DIST_RSP  (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint32_t))

class MeshDevice;

//mesh packet, based on ble adv
#pragma pack(push, 1)
typedef enum
{
  MESH_ADV_TYPE_RAW = 0x50,
  MESH_ADV_TYPE_DEFAULT,                // fallback packet for when packet queue is empty
  MESH_ADV_TYPE_NEIGHBOR_NOTIFICATION,  // emitted when a new neighbor is discovered
  MESH_ADV_TYPE_NEIGHBOR_REQUEST,       // asking for information about a neighbor's neighbor. Triggers neighbor notifications
  MESH_ADV_TYPE_ACK,                    // confirming reception of value
  MESH_ADV_TYPE_UNICAST,                // multihop directed message
  MESH_ADV_TYPE_BROADCAST,              // multihop broadcast
  MESH_ADV_TYPE_DIST_REQ,               // request for node distance response
  MESH_ADV_TYPE_DIST_RSP,               // response to dist req
  MESH_ADV_TYPE_ADDRESS_ADVERTISEMENT   // self-initiated advertisement
}mesh_adv_type_t;

typedef uint16_t msgID_t;

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
      uint8_t adv_len;                // BLE compatibility
      mesh_adv_type_t adv_type;       // BLE compatibility
      msgID_t msgID;                  // message identifier, to avoid loops
      union 
      {
        uint8_t raw[248];
        struct
        {
          ble_adv_addr_t clusterAddr; // address of cluster head
          uint16_t offsetFromCH;      // number of 625us slots offset
          uint8_t nbCount;            // number of other nodes this one is subscribed to
          uint8_t nodeWeight;         // based on power supply and stability. Lower is better
        } default;
        struct
        {
          ble_adv_addr_t neighbor;
        } neighborRequest;
        struct
        {
          ble_adv_addr_t neighborsCH; // address of neighbor's clusterhead
          ble_adv_addr_t neighborAddr;// address of neighbor
          uint32_t nextExpectedBeacon;// timeoffset from this msg to the next expected nb beacon
          uint8_t neighborWeight;     // neighbor's nodeWeight
        } neighborNotification;
        struct
        {
          ble_adv_addr_t source;      // source of request
          ble_adv_addr_t dest;        // destination to request distance to
          uint32_t source_dist;       // aggregate distance from source
          uint16_t ttl;               // max hop count for request
        } distReq;
        struct
        {
          ble_adv_addr_t source;      // source of request
          ble_adv_addr_t dest;        // destination to request distance to
          uint32_t source_dist;       // highest distance allowed for retransmit
          uint32_t dest_dist;         // aggregate distance from destination (the total source->dest distance)
        } distRsp;
        struct
        {
          ble_adv_addr_t source;      // advertised source
          uint32_t source_dist;       // distance to advertised source
          uint16_t ttl;               // remaining hops before retransmits stop
        } addressAdvertisement;
        union
        {
          struct
          {
            ble_adv_addr_t source;    // multihop source
            ble_adv_addr_t dest;      // multihop destination
            uint32_t source_dist;     // distance from source
            uint32_t dest_dist;       // distance to destination
            uint8_t payload[248 - MESH_PACKET_OVERHEAD_UNICAST];
          } unicast;
          struct
          {
            ble_adv_addr_t source;    // message originator
            uint16_t ttl;             // remaining hops before retransmits stop
            uint8_t payload[248 - MESH_PACKET_OVERHEAD_BROADCAST];
          } broadcast;
        }data;
      } payload;
    } str;
  }payload;

  uint32_t getPayloadLength(void);

} mesh_packet_t;
#pragma pack(pop)


typedef struct
{
  ble_adv_addr_t destination;
  uint32_t timeStamp;
  uint32_t distance;
} route_entry_t;

typedef struct
{
  msgID_t entries[MESH_MESSAGE_ID_CACHE_SIZE];
  uint32_t nextEntry;

  bool hasID(msgID_t msgID);
  void registerID(msgID_t msgID);
}msgIDcache_t;

class MeshNeighbor
{
  friend MeshDevice;
public:
  MeshNeighbor(ble_adv_addr_t* addr);

  void receivedBeacon(uint32_t rxTime, mesh_packet_t* beacon);

  bool isClusterHead(void);

  uint8_t getNodeWeight(void) { return mNodeWeight; }
  uint32_t getLastSyncTime(void) { return mLastSyncTime; }
  uint32_t getNextBeaconTime(uint32_t timeNow);
    
  ble_adv_addr_t mAdvAddr;
  bool mFollowing;
private:
  ble_adv_addr_t mClusterHead;

  uint32_t mLastSyncTime;
  uint32_t mLastBeaconTime;
  uint32_t mNbCount;
  uint8_t mNodeWeight;
  uint32_t mBeaconCount;
  uint32_t mBeaconInterval;
  timer_t mRxTimer;
};


class MeshDevice : public Device
{
public:
  MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x = 0.0, double y = 0.0);
  ~MeshDevice();
  void setAdvAddress(uint64_t addr);

  void startSearch(void);
  void stopSearch(void);

  void registerNeighbor(ble_adv_addr_t* advAddr, uint32_t rxTime, mesh_packet_t* pPacket = NULL);
  MeshNeighbor* getNeighbor(ble_adv_addr_t* advAddr);
  void abortSubscription(MeshNeighbor* pSub);

  void transmitUnicast(
    ble_adv_addr_t* source, 
    ble_adv_addr_t* dest,  
    uint8_t* data, uint32_t length, 
    uint32_t sender_dist = MESH_UNKNOWN_DIST, 
    uint32_t receiver_dist = MESH_UNKNOWN_DIST
    );
  void transmitBroadcast(
    ble_adv_addr_t* source,
    uint16_t ttl,
    uint8_t* data, uint32_t length
    );
  void transmitNeighborNotification(MeshNeighbor* pNb);
  void transmitRepeat(mesh_packet_t* pPacket, bool overWriteAdvAddr = true);

  bool hasMessageID(msgID_t messageID) { return mMsgIDcache.hasID(messageID); }
    
  void setClusterHead(MeshNeighbor* pNb);
  void setNodeWeight(uint8_t weight){ mNodeWeight = weight; }
  void addRoute(ble_adv_addr_t* addr, uint32_t dist);
  bool hasRouteTo(ble_adv_addr_t* addr);
  uint32_t getDistanceTo(ble_adv_addr_t* addr);
  
  void setCHBeaconOffset(uint32_t beaconOffset);

  virtual void LLdataRX(ble_adv_addr_t* sender, uint8_t* data, uint32_t length) {};

private:
  route_entry_t mRoutes[MESH_MAX_ROUTES];
  std::vector<MeshNeighbor> mNeighbors;
  std::queue<mesh_packet_t*> mPacketQueue;
  std::vector<MeshNeighbor*> mSubscriptions;
  mesh_packet_t mDefaultPacket;
  MeshNeighbor* mClusterHead;
  bool mSearching;
  ble_adv_addr_t mMyAddr;
  uint8_t mNodeWeight;
  uint32_t mCHBeaconOffset;
  timer_t mBeaconTimerID;
  msgIDcache_t mMsgIDcache;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);

  route_entry_t* getWeakestRoute(void);
  MeshNeighbor* getStrongestSubscription(void);
  void orderNextSubscriptionRx(MeshNeighbor* pSub);
  void subscriptionTimeout(uint32_t timestamp, void* context);
  void rxStop(uint32_t timestamp, void* context);
  void beaconTimeout(uint32_t timestamp, void* context);
  void electClusterHead(void);
  void subscribe(MeshNeighbor* pNb);
  void processPacket(mesh_packet_t* pMeshPacket);
};

  