#pragma once
#include "Device.h"
#include "RadioPacket.h"
#include "BlePacket.h"
#include "MeshWSN.h"
#include <vector>
#include <queue>
#include <stdint.h>
#include <stdarg.h>

#define MESH_INTERVAL               (100L*MS)
#define MESH_MAX_SUBSCRIPTIONS      (8)
#define MESH_OPTIMAL_SUBSCRIPTIONS  (4)
#define MESH_MAX_LOSS_COUNT         (6)
#define MESH_MAX_CLOCK_DRIFT        (500.0 * PPM)
#define MESH_MAX_CLOCK_DRIFT_TWO_SIDED  (MESH_MAX_CLOCK_DRIFT * 2.1)
#define MESH_CH_OFFSET_US(offs)     (625 * ((timestamp_t) offs))
#define MESH_CH_OFFSET_SLOTS(offs)  ((timestamp_t) offs / 625)
#define MESH_CH_BEACON_MARGIN       (500)
#define MESH_MAX_ROUTES             (10)
#define MESH_UNKNOWN_DIST           (UINT32_MAX)
#define MESH_RX_RU_TIME             (150L)       
#define MESH_TX_RU_TIME             (RADIO_DEFAULT_TURNAROUND)       
#define MESH_RX_LISTEN_TIME         (380 + MESH_RX_RU_TIME)
#define MESH_ROUTE_TIMEOUT          (1*MINUTES) //time before a route is deemed invalid
#define MESH_ACCESS_ADDR            (0xC0221E55)
#define MESH_MESSAGE_ID_CACHE_SIZE  (16)
#define MESH_CH_SWITCH_THRESHOLD    (20)        // required difference in ch weight before we change ch
#define MESH_BLANK_MSGID            (0)

#define MESH_PACKET_OVERHEAD              (sizeof(uint32_t) + sizeof(ble_packet_header_t) + BLE_ADV_ADDR_LEN + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(msgID_t))
#define MESH_PACKET_OVERHEAD_UNICAST      (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint32_t))
#define MESH_PACKET_OVERHEAD_BROADCAST    (BLE_ADV_ADDR_LEN + sizeof(uint16_t))
#define MESH_PACKET_OVERHEAD_NBNOT        (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint8_t))
#define MESH_PACKET_OVERHEAD_DEFAULT      (BLE_ADV_ADDR_LEN + sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t))
#define MESH_PACKET_OVERHEAD_JOIN_CLUSTER (1 + 4 * BLE_ADV_ADDR_LEN)
#define MESH_PACKET_OVERHEAD_SLEEPING     (BLE_ADV_ADDR_LEN + sizeof(uint16_t))
#define MESH_PACKET_OVERHEAD_DIST_REQ     (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint16_t))
#define MESH_PACKET_OVERHEAD_DIST_RSP     (2 * BLE_ADV_ADDR_LEN + sizeof(uint32_t) + sizeof(uint32_t))

#define _MESHLOG(name, str, ...) _LOG("[%s] " str, name.c_str(), __VA_ARGS__)
#define _MESHWARN(name, str, ...) _WARN("[%s] " str, name.c_str(), __VA_ARGS__)
#define _MESHERROR(name, str, ...) _ERROR("[%s] " str, name.c_str(), __VA_ARGS__)

class MeshDevice;
class MeshWSN;

//mesh packet, based on ble adv
#pragma pack(push, 1)
typedef enum
{
  MESH_ADV_TYPE_RAW = 0x50,
  MESH_ADV_TYPE_DEFAULT,                // fallback packet for when packet queue is empty
  MESH_ADV_TYPE_CLUSTER_REQ,
  MESH_ADV_TYPE_JOIN_CLUSTER,
  MESH_ADV_TYPE_SLEEPING,               // node is falling asleep
  MESH_ADV_TYPE_WAKEUP,                 // request to wake up nodes
  MESH_ADV_TYPE_CORRUPTION_NOTIFICATION,// a corrupted package was registered in the network
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
      uint8_t adv_type;               // BLE compatibility
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
          ble_adv_addr_t clusterAddr;
        } cluster_req;
        struct
        {
          uint8_t first_slot;
          ble_adv_addr_t node[4];
        } join_cluster;
        struct
        {
          ble_adv_addr_t clusterAddr; // address of cluster head
          uint16_t offsetFromCH;      // number of 625us slots offset
        } sleeping;
        struct
        {
          ble_adv_addr_t clusterAddr; // address of cluster head of cluster to wake up
        } wakeup;
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
  timestamp_t timeStamp;
  uint32_t distance;
} route_entry_t;

typedef struct
{
  msgID_t entries[MESH_MESSAGE_ID_CACHE_SIZE];
  uint32_t nextEntry;

  bool hasID(msgID_t msgID);
  void registerID(msgID_t msgID);
} msgIDcache_t;



class MeshNeighbor
{
  friend MeshDevice;
public:
  MeshNeighbor(ble_adv_addr_t* addr);

  void receivedBeacon(timestamp_t rxTime, mesh_packet_t* beacon, uint32_t channel, uint8_t signalStrength);

  bool isClusterHead(void);

  uint8_t getNodeWeight(void) { return mNodeWeight; }
  timestamp_t getNextBeaconTime(timestamp_t timeNow);
  uint32_t getSubscriptionScore(void);
  uint32_t getChannel(timestamp_t timestamp);
  uint32_t getLostPacketCount(timestamp_t timestamp);
    
  ble_adv_addr_t mAdvAddr;
  bool mFollowing;
private:
  ble_adv_addr_t mClusterHead;

  //uint32_t mLastSyncTime;
  timestamp_t mLastBeaconTime;
  uint32_t mNbCount;
  uint8_t mNodeWeight;
  uint32_t mBeaconCount;
  timestamp_t mBeaconInterval;
  timer_t mRxTimer;
  uint8_t mSignalStrength;
  uint32_t mLastChannel;
  uint8_t mLazyInterval;
  timestamp_t mBeaconOffset;
  //timestamp_t mInterval;

  MeshDevice* mDev; // for debugging
  bool mDebugTag;
};


class MeshDevice : public Device
{
  friend MeshWSN;
public:
  MeshDevice(std::string name = "mesh", double x = 0.0, double y = 0.0);
  ~MeshDevice();
  void setAdvAddress(uint64_t addr);
  ble_adv_addr_t* getAdvAddress(void) { return &mMyAddr; }

  void start(void);
  void kill(void);

  void startSearch(void);
  void stopSearch(void);

  void startBeaconing(void);
  void stopBeaconing(void);

  MeshNeighbor* registerNeighbor(ble_adv_addr_t* advAddr, timestamp_t rxTime, mesh_packet_t* pPacket = NULL, uint8_t signalStrength = 100, Device* pDev = NULL);
  MeshNeighbor* getNeighbor(ble_adv_addr_t* advAddr);
  void abortSubscription(MeshNeighbor* pSub);
  bool isSubscribedTo(ble_adv_addr_t* addr);

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
  void transmitSleep(void);
  void transmitClusterjoin(void);

  bool hasMessageID(msgID_t messageID) { return mMsgIDcache.hasID(messageID); }
    
  void setNodeWeight(uint8_t weight){ mNodeWeight = weight; }
  void setClusterHead(MeshNeighbor* pNb);
  bool hasClusterHead(void) const { return (mClusterHead != NULL || mIsCH); }
  void optimizeSubscriptions(void);

  void addRoute(ble_adv_addr_t* addr, uint32_t dist);
  bool hasRouteTo(ble_adv_addr_t* addr);
  uint32_t getDistanceTo(ble_adv_addr_t* addr);
  
  void setCHBeaconOffset(timestamp_t beaconOffset);
  void subscribe(MeshNeighbor* pNb);
  void resubscribe(MeshNeighbor* pNb);
  
  void print(void);

  virtual void LLdataRX(ble_adv_addr_t* sender, uint8_t* data, uint32_t length) {};

private:
  route_entry_t mRoutes[MESH_MAX_ROUTES];
  std::vector<MeshNeighbor*> mNeighbors;
  std::queue<mesh_packet_t*> mPacketQueue;
  std::vector<MeshNeighbor*> mSubscriptions;
  std::queue<MeshNeighbor*> mClusterRequesters;
  mesh_packet_t mDefaultPacket;
  MeshNeighbor* mCurrentSub;
  MeshNeighbor* mClusterHead;
  bool mSearching;
  bool mInSubscriptionRX;
  bool mInBeaconTX;
  bool mIsCH;
  bool mClusterSleeps;
  bool mBeaconing;
  bool mClusterSync;
  ble_adv_addr_t mMyAddr;
  uint8_t mNodeWeight;
  timestamp_t mCHBeaconOffset;
  timestamp_t mLastBeaconTime;
  uint32_t mClusterLeafCount;
  uint32_t mLastBeaconChannel;
  timestamp_t mFirstBeaconTX;
  uint32_t mBeaconCount;
  volatile timer_t mBeaconTimerID;
  volatile timer_t mCurrentRXTimer;
  msgIDcache_t mMsgIDcache;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);

  route_entry_t* getWeakestRoute(void);
  MeshNeighbor* getMostRedundantSubscription(void);
  MeshNeighbor* getLightestNeighbor(void);
  void resync(MeshNeighbor* pNb);
  void subscriptionTimeout(timestamp_t timestamp, void* context);
  void rxStop(timestamp_t timestamp, void* context);
  void beaconTimeout(timestamp_t timestamp, void* context);
  bool electClusterHead(void);
  void processPacket(mesh_packet_t* pMeshPacket, timestamp_t start_time);
  void becomeCH(void);
  void lostPacket(MeshNeighbor* pNb);
  void sleep(void);
  void prepareBeacon(void);
};

  