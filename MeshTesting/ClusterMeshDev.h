#pragma once
#include "MeshDevice.h"
#include "MeshWSN.h"
#include "Device.h"
#include "Radio.h"
#include <stack>

#define MESH_MAX_CLUSTER_SIZE (32)
#define MESH_CLUSTER_SLOT_US  (625)
#define MESH_LEAF_SCAN_WAIT   (MESH_INTERVAL * 10 + MESH_INTERVAL / 2)

typedef enum
{
  CM_STATE_RECON,
  CM_STATE_MAKE_CLUSTER,
  CM_STATE_REQ_CLUSTER,
  CM_STATE_CH,
  CM_STATE_CH_SCAN,
  CM_STATE_LEAF,
  CM_STATE_LEAF_SCAN
} cluster_mesh_state_t;

bool isScanningState(cluster_mesh_state_t state);
std::string getStateString(cluster_mesh_state_t state);

class MeshCluster
{
public:
  MeshCluster(ble_adv_addr_t* pCHaddr) 
  { 
    mCHaddr.set(*pCHaddr); 
    memset(mDevices, 0, MESH_MAX_CLUSTER_SIZE * sizeof(MeshNeighbor*)); 
    mOffsetFromOwnClusterTrain = 0;
  }
  ble_adv_addr_t mCHaddr;
  MeshNeighbor* mDevices[MESH_MAX_CLUSTER_SIZE];
  timestamp_t mOffsetFromOwnClusterTrain;
  uint8_t mClusterMax;

  bool contains(MeshNeighbor* pNb)
  {
    for (auto pDev : mDevices)
    {
      if (pDev == pNb)
        return true;
    }
    return false;
  }

  uint32_t getFirstDeviceIndex(void)
  {
    for (uint32_t i = 0; i < MESH_MAX_CLUSTER_SIZE; ++i)
    {
      if (mDevices[i] != NULL)
        return i;
    }
    return MESH_MAX_CLUSTER_SIZE;
  }
  uint32_t getLastDeviceIndex(void)
  {
    for (uint32_t i = MESH_MAX_CLUSTER_SIZE - 1; i > 0; --i)
    {
      if (mDevices[i] != NULL)
        return i;
    }
    return 0;
  }

  
};

class ClusterMeshDev : public Device
{
  friend MeshWSN;
public:
  ClusterMeshDev(std::string name = "mesh", double x = 0.0, double y = 0.0);
  ~ClusterMeshDev();

  void start();

  void cleanupPrevState(void);
  void doRecon(void);
  void doMakeCluster(void);
  void setCH(MeshNeighbor* pCH);
  void becomeCH(void);
  void subscribe(MeshNeighbor* pNb);
  void subscriptionAbort(MeshNeighbor* pNb);
  void makeClusterCheck(void); // check whether we have a decision on cluster creation yet, and act on it

  bool isInCluster(void) { return (mState == CM_STATE_LEAF || CM_STATE_CH); }
  bool isSubscribedTo(MeshNeighbor* pNb);
  bool isCH(void) { return mState == CM_STATE_CH || mState == CM_STATE_CH_SCAN; }

  MeshCluster* getCluster(ble_adv_addr_t* pCHaddr);
  void setScore(uint8_t score) { mScore = score; }
  
private:
  std::vector<MeshNeighbor*> mNeighbors;
  std::vector<MeshNeighbor*> mSubscriptions;
  std::vector<MeshCluster> mClusters;
  std::queue<MeshNeighbor*> mClusterReqs;
  MeshNeighbor* mClusterHead;
  MeshCluster* mMyCluster;
  cluster_mesh_state_t mState;
  timer_t mBeaconTimer;
  timer_t mChTimer;
  timer_t mCurrentSubAbortTimer;
  bool mLeafScanTriggered;
  bool mJustFinishedLeafScan;
  timestamp_t mLastStateChange;
  std::queue<mesh_packet_t*> mPacketQueue;
  std::stack<mesh_packet_t> mDefaultPacket;
  uint8_t mScore;
  uint32_t mMyClusterOffset;
  ble_adv_addr_t mAdvAddr;

  MeshNeighbor* getBestNb(std::function<bool(MeshNeighbor*)> filterFunc = NULL);
  MeshNeighbor* getNb(ble_adv_addr_t* pAdvAddr);

  virtual void radioCallbackTx(RadioPacket* pPacket);
  virtual void radioCallbackRx(RadioPacket* pPacket, uint8_t rx_strength, bool corrupted);

  void processPacket(mesh_packet_t* pMeshPacket);
  void radioBeaconTX(void);
  void radioSubRX(MeshNeighbor* pNb, bool noDrift = false);
  void orderRestOfCluster(MeshCluster* pCluster, uint32_t anchorIndex, timestamp_t anchorTimestamp);
  bool isSubscribedToCluster(MeshCluster* pCluster);

  void setState(cluster_mesh_state_t state);
  

  void transmitClusterjoin(void);
  void transmitClusterReq(void);
};

