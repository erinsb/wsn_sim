#include "MeshDevice.h"
#include "BlePacket.h"
#include "Radio.h"
#include "RadioPacket.h"
#include <random>

#define SEARCHLIGHT_SLOT (10000)


MeshNeighbor::MeshNeighbor(ble_adv_addr_t* adv_addr) : 
  mLastSyncTime(0), 
  mBeaconCount(0), 
  mFollowing(0),
  mNbCount(0),
  mPowerScore(0)
{
  memcpy(&mAdvAddr, adv_addr, 6);
}

void MeshNeighbor::receivedBeacon(uint32_t rxTime, mesh_packet_t* beacon)
{
  ++mBeaconCount;
  mLastSyncTime = rxTime;

  // extract any state information 
  switch (beacon->payload.str.adv_type)
  {
  case MESH_ADV_TYPE_DEFAULT:
    if (beacon->payload.str.payload.default.clusterAddr != mClusterHead)
    {
      memcpy(mClusterHead.arr, beacon->payload.str.payload.default.clusterAddr.arr, BLE_ADV_ADDR_LEN);
    }
    mNbCount = beacon->payload.str.payload.default.nbCount;
    mPowerScore = beacon->payload.str.payload.default.powerScore;
    break;
  default:
    // don't care
  }
}

bool MeshNeighbor::isClusterHead(void)
{
  return (mAdvAddr == mClusterHead);
}

uint32_t MeshNeighbor::getNextBeaconTime(uint32_t timeNow)
{
  if (timeNow <= mLastSyncTime)
    return mLastSyncTime + MESH_INTERVAL;
  else
    return (1 + (timeNow - mLastSyncTime) / MESH_INTERVAL) * MESH_INTERVAL + mLastSyncTime;
}

MeshDevice::MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x, double y) : Device(x, y), mSearching(false)
{
  double driftFactor = (MESH_MAX_CLOCK_DRIFT) * (2.0 * ((double)rand() / RAND_MAX) - 1.0) + 1.0; // 1.0 +- <250/1mill 
  mTimer->setDriftFactor(driftFactor);

  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    mMyAddr.arr[i] = rand() & 0xFF;
  }

  for (uint8_t i = 0; i < MESH_MAX_ROUTES; ++i)
  {
    mRoutes[i].timeStamp = 0;
    mRoutes[i].distance = MESH_UNKNOWN_DIST;
  }
}


MeshDevice::~MeshDevice()
{
}



void MeshDevice::startSearch(void)
{
  if (mSearching)
    return;
  mSearching = true;
  if (mRadio->getState() == Radio::RADIO_STATE_IDLE)
  {
    mRadio->receive();
  }
}

void MeshDevice::stopSearch(void)
{
  mSearching = false;
}

void MeshDevice::registerNeighbor(ble_adv_addr_t* advAddr, uint32_t rxTime)
{
  mNeighbors.push_back(MeshNeighbor(advAddr, rxTime));
  if (mNeighbors.size() == )

}

MeshNeighbor* MeshDevice::getNeighbor(ble_adv_addr_t* advAddr)
{
  for (MeshNeighbor& nb : mNeighbors)
  {
    if (nb.mAdvAddr == *advAddr)
    {
      return &nb;
    }
  }

  return NULL;
}

void MeshDevice::transmit(uint8_t* data, uint32_t length)
{
  ble_adv_packet_t* pPacket = new ble_adv_packet_t();
  memcpy(pPacket->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);

  mPacketQueue.push(pPacket);
}

void MeshDevice::addRoute(ble_adv_addr_t* addr, uint32_t dist)
{

}


void MeshDevice::radioCallbackTx(RadioPacket* packet)
{

}

void MeshDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (corrupted)
    return;

  ble_adv_packet_t blePacket;
  memcpy(&blePacket, packet->getContents(), packet->getLength());

  MeshNeighbor* pNb = getNeighbor(&blePacket.adv_addr);
  if (pNb == NULL)
  {
    registerNeighbor(&blePacket.adv_addr, packet->mStartTime);
    if ()
  }
  else
  {
    pNb->receivedBeacon(packet->mStartTime);
  }

  
}

route_entry_t* MeshDevice::getWeakestRoute(void)
{
  route_entry_t* pOldest = &mRoutes[0];

  for (uint32_t i = 1; i < MESH_MAX_ROUTES; ++i)
  {
    if (pOldest->timeStamp == 0) // quick exit before route table is filled
      return pOldest;

    if (mRoutes[i].timeStamp < pOldest->timeStamp)
      pOldest = &mRoutes[i];
  }

  return pOldest;
}

MeshNeighbor* MeshDevice::getStrongestSubscription(void)
{
  if (mSubscriptions.size() == 0)
    return NULL;

  MeshNeighbor* pStrongest = mSubscriptions[0];

  for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
  {
    (*it)->
  }
}