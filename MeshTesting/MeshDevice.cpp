#include "MeshDevice.h"
#include "BlePacket.h"
#include "Radio.h"
#include "RadioPacket.h"
#include <random>

#define SEARCHLIGHT_SLOT (10000)

using namespace std::placeholders;

uint32_t mesh_packet_t::getPayloadLength(void)
{
  switch (payload.str.adv_type)
  {
  case MESH_ADV_TYPE_RAW:
    return payload.str.adv_len;
  case MESH_ADV_TYPE_DEFAULT:
    return 0;
  case MESH_ADV_TYPE_BROADCAST:
    return payload.str.adv_len - MESH_PACKET_OVERHEAD_BROADCAST;
  case MESH_ADV_TYPE_NEIGHBOR_NOTIFICATION:
    return 0;
  case MESH_ADV_TYPE_UNICAST:
    return payload.str.adv_len - MESH_PACKET_OVERHEAD_UNICAST;
  default:
    return 0;
  }
}


bool msgIDcache_t::hasID(msgID_t msgID)
{
  for (msgID_t& entry : entries)
  {
    if (entry == msgID)
      return true;
  }
  return false;
}

void msgIDcache_t::registerID(msgID_t msgID)
{
  entries[(nextEntry++) % MESH_MESSAGE_ID_CACHE_SIZE];
}

MeshNeighbor::MeshNeighbor(ble_adv_addr_t* adv_addr) : 
  mLastSyncTime(0), 
  mLastBeaconTime(0),
  mBeaconCount(0), 
  mFollowing(0),
  mNbCount(0),
  mNodeWeight(0)
{
  memcpy(&mAdvAddr, adv_addr, 6);
}

void MeshNeighbor::receivedBeacon(uint32_t rxTime, mesh_packet_t* beacon)
{
  ++mBeaconCount;
  mLastBeaconTime = rxTime;
  mLastSyncTime = rxTime;

  // extract any state information 
  if (beacon != NULL)
  {
    switch (beacon->payload.str.adv_type)
    {
    case MESH_ADV_TYPE_DEFAULT:
      if (beacon->payload.str.payload.default.clusterAddr != mClusterHead)
      {
        memcpy(mClusterHead.arr, beacon->payload.str.payload.default.clusterAddr.arr, BLE_ADV_ADDR_LEN);
      }
      mNbCount = beacon->payload.str.payload.default.nbCount;
      mNodeWeight = beacon->payload.str.payload.default.nodeWeight;
      break;
    default:
      // don't care
      break;
    }
  }
}

bool MeshNeighbor::isClusterHead(void)
{
  return (mAdvAddr == mClusterHead);
}

uint32_t MeshNeighbor::getNextBeaconTime(uint32_t timeNow)
{
  if (timeNow <= mLastBeaconTime)
    return mLastBeaconTime + MESH_INTERVAL;
  else
    return (1 + (timeNow - mLastBeaconTime) / MESH_INTERVAL) * MESH_INTERVAL + mLastBeaconTime;
}






MeshDevice::MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x, double y) : 
  Device(x, y), 
  mSearching(false), 
  mCHBeaconOffset(0), 
  mClusterHead(NULL)
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
  mMsgIDcache.nextEntry = 0;

  // must use some time at the beginning to find devices
  startSearch();
  mTimer->orderRelative(MESH_INTERVAL + MESH_CH_BEACON_MARGIN, [](uint32_t timeout, void* context) { ((MeshDevice*)context)->electClusterHead(); }, this);
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

void MeshDevice::registerNeighbor(ble_adv_addr_t* advAddr, uint32_t rxTime, mesh_packet_t* pPacket)
{
  mNeighbors.push_back(MeshNeighbor(advAddr));
  MeshNeighbor* pNb = &mNeighbors.back();
  mSubscriptions.push_back(pNb);
  
  pNb->mFollowing = true;
  pNb->receivedBeacon(rxTime, pPacket);

  orderNextSubscriptionRx(pNb);

  if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS) // need to reduce set
  {
    MeshNeighbor* pStrongestSub = getStrongestSubscription();
    abortSubscription(pStrongestSub);    
  }
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

void MeshDevice::abortSubscription(MeshNeighbor* pSub)
{
  for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
  {
    if (*it == pSub)
    {
      MeshNeighbor* pNb = *it;
      pNb->mFollowing = false;
      mTimer->abort(pNb->mRxTimer);
      mSubscriptions.erase(it);

      break;
    }
  }
}

void MeshDevice::transmitUnicast(
  ble_adv_addr_t* source,
  ble_adv_addr_t* dest,
  uint8_t* data, uint32_t length,
  uint32_t source_dist,
  uint32_t dest_dist
  )
{
  mesh_packet_t* pPacket = new mesh_packet_t();
  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_UNICAST + length;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_UNICAST + length;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_UNICAST;
  pPacket->payload.str.payload.data.unicast.source_dist = source_dist;
  pPacket->payload.str.payload.data.unicast.dest_dist = dest_dist;
  memcpy(pPacket->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.data.unicast.source.arr, source->arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.data.unicast.dest.arr, dest->arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.data.unicast.payload, data, length);

  mPacketQueue.push(pPacket);
}

void MeshDevice::transmitBroadcast(
  ble_adv_addr_t* source,
  uint16_t ttl,
  uint8_t* data, uint32_t length
  )
{
  mesh_packet_t* pPacket = new mesh_packet_t();
  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_BROADCAST + length;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_BROADCAST + length;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_BROADCAST;
  memcpy(pPacket->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.data.broadcast.source.arr, source->arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.data.broadcast.payload, data, length);

  mPacketQueue.push(pPacket);
}

void MeshDevice::transmitNeighborNotification(MeshNeighbor* pNb)
{
  mesh_packet_t* pPacket = new mesh_packet_t();
  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_NBNOT;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_NBNOT;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_NEIGHBOR_NOTIFICATION;
  memcpy(pPacket->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.neighborNotification.neighborAddr.arr, pNb->mAdvAddr.arr, BLE_ADV_ADDR_LEN);
  memcpy(pPacket->payload.str.payload.neighborNotification.neighborsCH.arr, pNb->mClusterHead.arr, BLE_ADV_ADDR_LEN);
  pPacket->payload.str.payload.neighborNotification.nextExpectedBeacon = pNb->getNextBeaconTime(mTimer->getTimestamp());
  pPacket->payload.str.payload.neighborNotification.neighborWeight = pNb->mNodeWeight;

  mPacketQueue.push(pPacket);
}

void MeshDevice::transmitRepeat(mesh_packet_t* pPacket, bool overWriteAdvAddr)
{
  mesh_packet_t* pNewPacket = new mesh_packet_t();
  memcpy(pNewPacket, pPacket, sizeof(mesh_packet_t));

  if (overWriteAdvAddr)
  {
    memcpy(pNewPacket->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
  }

  mPacketQueue.push(pNewPacket);
}

void MeshDevice::setClusterHead(MeshNeighbor* nb)
{
  if (mClusterHead != NULL)
  {
    // keep our timeouts, but adjust offset
    uint32_t nextCHbeacon = nb->getNextBeaconTime(mTimer->getTimestamp());
    
    mCHBeaconOffset = (MESH_INTERVAL + mTimer->getExpiration(mBeaconTimerID) - nb->mLastBeaconTime) % MESH_INTERVAL;

    if (!nb->mFollowing)
    {
      if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS)
      {
        abortSubscription(getStrongestSubscription());
      }
      subscribe(nb);
    }


    mClusterHead = nb;
  }
  else
  {
    if (mCHBeaconOffset == 0)
      mCHBeaconOffset = rand() % (MESH_INTERVAL - 2 * MESH_CH_BEACON_MARGIN) + MESH_CH_BEACON_MARGIN;
    mBeaconTimerID = mTimer->orderAt(nb->mLastBeaconTime + mCHBeaconOffset, std::bind(&MeshDevice::beaconTimeout, this, _1, _2), NULL);
  }
  mClusterHead = nb;
}

void MeshDevice::addRoute(ble_adv_addr_t* addr, uint32_t dist)
{
  uint32_t timeNow = mTimer->getTimestamp();
  for (route_entry_t& route : mRoutes)
  {
    if (route.destination == *addr)
    {
      if (dist < route.distance || (timeNow - route.timeStamp) > MESH_ROUTE_TIMEOUT) 
      {
        route.distance = dist + mNodeWeight;
        route.timeStamp = timeNow;
      }
      return;
    }
  }
  // route does not exist in our table, create one. 
  route_entry_t* entry = getWeakestRoute();
  entry->destination = *addr;
  entry->distance = dist + mNodeWeight;
  entry->timeStamp = timeNow;
}

bool MeshDevice::hasRouteTo(ble_adv_addr_t* addr)
{
  for (route_entry_t& route : mRoutes)
  {
    if (route.destination == *addr)
      return true;
  }
  return false;
}

uint32_t MeshDevice::getDistanceTo(ble_adv_addr_t* addr)
{
  // search one hop neighbors
  for (MeshNeighbor* pNb : mSubscriptions)
  {
    if (pNb->mAdvAddr == *addr)
      return mNodeWeight + pNb->getNodeWeight();
  }

  // search routing table
  for (route_entry_t& route : mRoutes)
  {
    if (route.destination == *addr)
      return route.distance;
  }

  return MESH_UNKNOWN_DIST;
}

void MeshDevice::setCHBeaconOffset(uint32_t beaconOffset)
{

}

void MeshDevice::radioCallbackTx(RadioPacket* packet)
{
  // don't care
}

void MeshDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (corrupted || packet == NULL)
    return;

  mesh_packet_t* pMeshPacket = (mesh_packet_t*) packet->getContents();

  MeshNeighbor* pNb = getNeighbor(&pMeshPacket->adv_addr);
  if (pNb == NULL)
  {
    registerNeighbor(&pMeshPacket->adv_addr, packet->mStartTime);
  }
  else
  {
    pNb->receivedBeacon(packet->mStartTime, pMeshPacket); // let neighbor structure update itself
  }  
  if (pNb->isClusterHead())
  {
    // sync all leaf node drifts
    for (MeshNeighbor& chLeaf : mNeighbors)
    {
      if (chLeaf.mClusterHead == pNb->mAdvAddr)
      {
        chLeaf.mLastSyncTime = packet->mStartTime;

        // alter timeout
        if (chLeaf.mFollowing)
        {
          mTimer->abort(chLeaf.mRxTimer);
          orderNextSubscriptionRx(&chLeaf);
        }
      }
    }
  }

  processPacket(pMeshPacket);
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
    if (pStrongest->mNbCount < (*it)->mNbCount)
      pStrongest = *it;
  }

  return pStrongest;
}

void MeshDevice::orderNextSubscriptionRx(MeshNeighbor* pSub)
{
  uint32_t timeNow = getEnvironment()->getTimestamp();
  uint32_t nextBeacon = pSub->getNextBeaconTime(timeNow);
  uint32_t deltaTime = (nextBeacon - pSub->mLastSyncTime);

  mTimer->orderAt(nextBeacon - deltaTime * MESH_MAX_CLOCK_DRIFT * 2.0 - MESH_RX_RU_TIME, 
    std::bind(&MeshDevice::subscriptionTimeout, this, _1, _2), (void*) pSub);
}

void MeshDevice::subscriptionTimeout(uint32_t timestamp, void* context)
{
  MeshNeighbor* pSub = (MeshNeighbor*) context;
  
  if (mRadio->getState() == Radio::RADIO_STATE_IDLE)
  {
    mRadio->receive();
    mTimer->orderRelative((timestamp - pSub->getLastSyncTime()) * MESH_MAX_CLOCK_DRIFT * 2.0 + MESH_RX_LISTEN_TIME, 
      std::bind(&MeshDevice::rxStop, this, _1, _2), (void*) pSub);
  }
}

void MeshDevice::rxStop(uint32_t timestamp, void* context)
{
  MeshNeighbor* pSub = (MeshNeighbor*)context;
  mRadio->disable();

  radioCallbackRx(NULL, 0, true); // want to keep all packet logic in one place
}

void MeshDevice::beaconTimeout(uint32_t timestamp, void* context)
{
  // send packet from queue, if any. 
  if (mPacketQueue.size() > 0)
  {
    mesh_packet_t* pPacket = mPacketQueue.front();
    mRadio->setPacket((uint8_t*)pPacket, pPacket->header.length + MESH_PACKET_OVERHEAD);
    mPacketQueue.pop();
  }
  else
  { 
    // update values in default packet
    if (mClusterHead != NULL)
    {
      memcpy(mDefaultPacket.payload.str.payload.default.clusterAddr.arr, mClusterHead->mAdvAddr.arr, BLE_ADV_ADDR_LEN);
    }
    else
    {
      memcpy(mDefaultPacket.payload.str.payload.default.clusterAddr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
    }
    mDefaultPacket.payload.str.payload.default.nbCount = mSubscriptions.size();
    mDefaultPacket.payload.str.payload.default.nodeWeight = mNodeWeight;
    mDefaultPacket.payload.str.payload.default.offsetFromCH = mCHBeaconOffset;
    mRadio->setPacket((uint8_t*)&mDefaultPacket, mDefaultPacket.header.length + MESH_PACKET_OVERHEAD);
  }
  mRadio->transmit();
}

void MeshDevice::electClusterHead(void)
{
  if (mNeighbors.size() == 0)
  {
    mClusterHead = NULL;
    return;
  }
  MeshNeighbor* pCH = &mNeighbors[0];

  for (MeshNeighbor& nb : mNeighbors)
  {
    if (nb.mNodeWeight < mClusterHead->mNodeWeight && nb.isClusterHead())
      pCH = &nb;
  }

  if (pCH->mNodeWeight > mNodeWeight)
  {
    mClusterHead = NULL;
    return;
  }
  setClusterHead(pCH);
}

void MeshDevice::processPacket(mesh_packet_t* pMeshPacket)
{
  if (hasMessageID(pMeshPacket->payload.str.msgID))
    return;

  switch (pMeshPacket->payload.str.adv_type)
  {
  case MESH_ADV_TYPE_BROADCAST:
    if (pMeshPacket->payload.str.payload.data.broadcast.source != mMyAddr)
    {
      LLdataRX(&pMeshPacket->payload.str.payload.data.unicast.source,
        pMeshPacket->payload.str.payload.data.broadcast.payload,
        pMeshPacket->getPayloadLength());
      if (pMeshPacket->payload.str.payload.data.broadcast.ttl-- > 0)
      {
        transmitRepeat(pMeshPacket);
      }
    }
    break;
  case MESH_ADV_TYPE_UNICAST:

    if (pMeshPacket->payload.str.payload.data.unicast.source_dist != MESH_UNKNOWN_DIST) 
      addRoute(&pMeshPacket->payload.str.payload.data.unicast.source, pMeshPacket->payload.str.payload.data.unicast.source_dist);

    if (pMeshPacket->payload.str.payload.data.unicast.dest == mMyAddr)
    {
      LLdataRX(&pMeshPacket->payload.str.payload.data.unicast.source,
        pMeshPacket->payload.str.payload.data.unicast.payload,
        pMeshPacket->getPayloadLength());
    }
    else // not for me. Retransmit?
    {
      uint32_t dest_dist = getDistanceTo(&pMeshPacket->payload.str.payload.data.unicast.dest);
      uint32_t source_dist = getDistanceTo(&pMeshPacket->payload.str.payload.data.unicast.source);

      if (dest_dist < pMeshPacket->payload.str.payload.data.unicast.dest_dist) // moving towards known target
      {
        pMeshPacket->payload.str.payload.data.unicast.source_dist += mNodeWeight;
        pMeshPacket->payload.str.payload.data.unicast.dest_dist = dest_dist;
        transmitRepeat(pMeshPacket);
      }
      else if (pMeshPacket->payload.str.payload.data.unicast.source_dist < source_dist &&
        pMeshPacket->payload.str.payload.data.unicast.dest_dist == MESH_UNKNOWN_DIST) // moving away from a source that doesn't know the target
      {
        pMeshPacket->payload.str.payload.data.unicast.source_dist += mNodeWeight;
        pMeshPacket->payload.str.payload.data.unicast.dest_dist = dest_dist;
        transmitRepeat(pMeshPacket);
      }
    }
    break;
  case MESH_ADV_TYPE_DIST_REQ:
  {
    uint32_t distance = getDistanceTo(&pMeshPacket->payload.str.payload.distReq.dest);

    if (pMeshPacket->payload.str.payload.distReq.source_dist != MESH_UNKNOWN_DIST)
      addRoute(&pMeshPacket->payload.str.payload.distReq.source, pMeshPacket->payload.str.payload.distReq.source_dist);

    if (distance != MESH_UNKNOWN_DIST) // we have a path, respond to request
    {
      mesh_packet_t* pRsp = new mesh_packet_t();
      pRsp->access_addr = MESH_ACCESS_ADDR;
      pRsp->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_NBNOT;
      pRsp->header.type = BLE_PACKET_TYPE_ADV_IND;
      pRsp->payload.str.adv_len = MESH_PACKET_OVERHEAD_NBNOT;
      pRsp->payload.str.adv_type = MESH_ADV_TYPE_NEIGHBOR_NOTIFICATION;
      pRsp->payload.str.msgID = pMeshPacket->payload.str.msgID + 1;
      memcpy(pRsp->adv_addr.arr, mMyAddr.arr, BLE_ADV_ADDR_LEN);
      memcpy(pRsp->payload.str.payload.distRsp.dest.arr, pMeshPacket->payload.str.payload.distReq.dest.arr, BLE_ADV_ADDR_LEN);
      pRsp->payload.str.payload.distRsp.dest_dist = distance + mNodeWeight;
      memcpy(pRsp->payload.str.payload.distRsp.source.arr, pMeshPacket->payload.str.payload.distReq.source.arr, BLE_ADV_ADDR_LEN);
      pRsp->payload.str.payload.distRsp.source_dist = pMeshPacket->payload.str.payload.distReq.source_dist + mNodeWeight;
    }
    else // just relay, don't answer
    {
      if (pMeshPacket->payload.str.payload.distReq.ttl-- > 0)
      {
        pMeshPacket->payload.str.payload.distReq.source_dist += mNodeWeight;
        transmitRepeat(pMeshPacket);
      }
    }
    break; 
  }

  case MESH_ADV_TYPE_DIST_RSP:
  {
    uint32_t distance = getDistanceTo(&pMeshPacket->payload.str.payload.distReq.dest);

    if (pMeshPacket->payload.str.payload.distRsp.source_dist > distance) // we're further from the source than the one we receive from, don't repeat
      break;

    if (pMeshPacket->payload.str.payload.distRsp.dest_dist != MESH_UNKNOWN_DIST)
      addRoute(&pMeshPacket->payload.str.payload.distRsp.dest, pMeshPacket->payload.str.payload.distRsp.dest_dist);

    pMeshPacket->payload.str.payload.distRsp.source_dist = distance;

    transmitRepeat(pMeshPacket);

    break;
  }

  case MESH_ADV_TYPE_ADDRESS_ADVERTISEMENT:
    addRoute(&pMeshPacket->payload.str.payload.addressAdvertisement.source, pMeshPacket->payload.str.payload.addressAdvertisement.source_dist);

    if (pMeshPacket->payload.str.payload.addressAdvertisement.ttl-- > 0) // relay
      transmitRepeat(pMeshPacket);
    break;

  case MESH_ADV_TYPE_NEIGHBOR_REQUEST:
    for (neighbor)
  }

  mMsgIDcache.registerID(pMeshPacket->payload.str.msgID);
}
