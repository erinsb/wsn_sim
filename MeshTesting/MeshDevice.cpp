#include "MeshDevice.h"
#include "BlePacket.h"
#include "Radio.h"
#include "RadioPacket.h"
#include "Logger.h"

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
  if (msgID != MESH_BLANK_MSGID)
    entries[(nextEntry++) % MESH_MESSAGE_ID_CACHE_SIZE];
}

MeshNeighbor::MeshNeighbor(ble_adv_addr_t* adv_addr) : 
  mLastSyncTime(0), 
  mLastBeaconTime(0),
  mBeaconCount(0), 
  mFollowing(0),
  mNbCount(0),
  mNodeWeight(0),
  mLostPackets(0),
  mBeaconInterval(0),
  mRxTimer(0)
{
  mClusterHead.clear();
  mAdvAddr.set(*adv_addr);
}

void MeshNeighbor::receivedBeacon(uint32_t rxTime, mesh_packet_t* beacon, uint32_t channel)
{
  ++mBeaconCount;
  mLastBeaconTime = rxTime;
  mLastSyncTime = rxTime;
  mLastChannel = channel;
  mLostPackets = 0;

  // extract any state information 
  if (beacon != NULL)
  {
    switch (beacon->payload.str.adv_type)
    {
    case MESH_ADV_TYPE_DEFAULT:
      if (beacon->payload.str.payload.default.clusterAddr != mClusterHead)
      {
        if (mClusterHead == mAdvAddr)
          _LOG("%s Became Cluster head!", mAdvAddr.toString().c_str());
      }
      mClusterHead.set(beacon->payload.str.payload.default.clusterAddr);
      mNbCount = beacon->payload.str.payload.default.nbCount;
      mNodeWeight = beacon->payload.str.payload.default.nodeWeight;
      break;
    default:
      // don't care
      _WARN("Unhandled nb packet");
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

uint32_t MeshNeighbor::getSubscriptionScore(void)
{
  return (MESH_MAX_SUBSCRIPTIONS - mNbCount);
}

uint32_t MeshNeighbor::getNextChannel(void)
{
  return (mLastChannel + ((mLostPackets + 1) * (1 + mAdvAddr.arr[0]) % 3) - 37) % 3 + 37;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

MeshDevice::MeshDevice(std::string name, double x, double y) :
  Device(x, y), 
  mSearching(false), 
  mCHBeaconOffset(mRand(MESH_INTERVAL - 2 * MESH_CH_BEACON_MARGIN) + MESH_CH_BEACON_MARGIN), 
  mClusterHead(NULL),
  mNodeWeight(mRand() % 0xFF),
  mIsCH(false),
  mBeaconCount(0)
{
  mName = name;
  double driftFactor = ((MESH_MAX_CLOCK_DRIFT)* (2.0 * mRand.Float() - 1.0)) + 1.0; // 1.0 +- <250/1mill 
  mTimer->setDriftFactor(driftFactor);

  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    mMyAddr.arr[i] = mRand() & 0xFF;
  }

  for (uint8_t i = 0; i < MESH_MAX_ROUTES; ++i)
  {
    mRoutes[i].timeStamp = 0;
    mRoutes[i].distance = MESH_UNKNOWN_DIST;
  }
  mMsgIDcache.nextEntry = 0;

  memset(&mDefaultPacket, 0, sizeof(mesh_packet_t));
  mDefaultPacket.payload.str.adv_type = MESH_ADV_TYPE_DEFAULT;
  mDefaultPacket.access_addr = MESH_ACCESS_ADDR;
  mDefaultPacket.adv_addr.set(mMyAddr);
  mDefaultPacket.payload.str.adv_len = MESH_PACKET_OVERHEAD_DEFAULT;
  mDefaultPacket.payload.str.msgID = 0;
  mDefaultPacket.payload.str.payload.default.nodeWeight = mNodeWeight;
  mDefaultPacket.payload.str.payload.default.clusterAddr.clear();
  mDefaultPacket.payload.str.payload.default.nbCount = 0;
  mDefaultPacket.payload.str.payload.default.nbCount = 0;
  mDefaultPacket.header.length = 4 + MESH_PACKET_OVERHEAD_DEFAULT;
}


MeshDevice::~MeshDevice()
{
}

void MeshDevice::start(void)
{
  // must use some time at the beginning to find devices
  startSearch();
  mTimer->orderRelative(MESH_INTERVAL + MESH_CH_BEACON_MARGIN, [](uint32_t timeout, void* context)
  {
    MeshDevice* pMD = ((MeshDevice*)context);
    _MESHLOG(pMD->mName, "Stopped Searching");
    pMD->stopSearch();
    pMD->electClusterHead();
  },
    this);
  
  startBeaconing();
}


void MeshDevice::startSearch(void)
{
  if (mSearching)
    return;

  mSearching = true;

  mRadio->setChannel(37);
  mRadio->shortToRx();
  mRadio->receive();

}

void MeshDevice::stopSearch(void)
{
  mSearching = false;
  mRadio->shortDisable();
  mRadio->disable();
}

void MeshDevice::startBeaconing(void)
{
  // randomized offset
  mBeaconTimerID = mTimer->orderPeriodic(mCHBeaconOffset, MESH_INTERVAL, MEMBER_TIMEOUT(MeshDevice::beaconTimeout));
}

void MeshDevice::stopBeaconing(void)
{
  mTimer->abort(mBeaconTimerID);
}

MeshNeighbor* MeshDevice::registerNeighbor(ble_adv_addr_t* advAddr, uint32_t rxTime, mesh_packet_t* pPacket, Device* pDev)
{
  MeshNeighbor* pNb = new MeshNeighbor(advAddr);
  mNeighbors.push_back(pNb);
  pNb->mDev = (MeshDevice*) pDev;

  pNb->receivedBeacon(rxTime, pPacket, mRadio->getChannel());
  _MESHLOG(mName, "Registered neighbor %s", advAddr->toString().c_str());
  subscribe(pNb);
  return pNb;
}

MeshNeighbor* MeshDevice::getNeighbor(ble_adv_addr_t* advAddr)
{
  for (MeshNeighbor* pNb : mNeighbors)
  {
    if (pNb->mAdvAddr == *advAddr)
    {
      return pNb;
    }
  }

  return NULL;
}

void MeshDevice::abortSubscription(MeshNeighbor* pSub)
{
  _MESHWARN(mName, "Aborted sub to %s", pSub->mAdvAddr.toString().c_str());
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
  mCHBeaconOffset = (MESH_INTERVAL + mTimer->getExpiration(mBeaconTimerID) - nb->mLastBeaconTime) % MESH_INTERVAL;
  if (mClusterHead != NULL)
  {
    // keep our timeouts, but adjust offset
    uint32_t nextCHbeacon = nb->getNextBeaconTime(mTimer->getTimestamp());
    

    if (!nb->mFollowing)
    {
      if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS)
      {
        abortSubscription(getMostRedundantSubscription());
      }
      subscribe(nb);
    }
  }
  else
  {
    //mBeaconTimerID = mTimer->orderAt(nb->mLastBeaconTime + mCHBeaconOffset, MEMBER_TIMEOUT(MeshDevice::beaconTimeout), NULL);
  }
  mClusterHead = nb;
  mWSN->addConnection(this, nb->mDev);
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

bool MeshDevice::isSubscribedTo(ble_adv_addr_t* addr)
{
  for (MeshNeighbor* pNb : mSubscriptions)
  {
    if (pNb->mAdvAddr == *addr)
      return true;
  }

  return false;
}

void MeshDevice::setCHBeaconOffset(uint32_t beaconOffset)
{
  if (mIsCH)
    _MESHWARN(mName, "Setting beacon offset in cluster head");

  if (mClusterHead)
  {
    mTimer->abort(mBeaconTimerID);
  }
  mCHBeaconOffset = beaconOffset;

  mBeaconTimerID = mTimer->orderAt(mClusterHead->mLastBeaconTime + mCHBeaconOffset, MEMBER_TIMEOUT(MeshDevice::beaconTimeout), NULL);
}

void MeshDevice::print(void)
{
  if (!LOG_ENABLE)
    return;
  printf("DEVICE: %s %s\n", mName.c_str(), mMyAddr.toString().c_str());
  printf("\tWeight: %d\n", mNodeWeight);
  printf("\tDrift factor: %f\n", mTimer->getDriftFactor());
  if (mIsCH)
    printf("\tCLUSTER HEAD\n");
  else if (mClusterHead != NULL)
    printf("\tCH:%s\n", mClusterHead->mAdvAddr.toString().c_str());
  printf("\tNeighbors: %d\n", mNeighbors.size());
  for (uint32_t i = 0; i < mNeighbors.size(); ++i)
  {
    MeshNeighbor* pNb = mNeighbors[i];
    printf("\t%s ", pNb->mAdvAddr.toString().c_str());
    if (mClusterHead == pNb)
      printf("Cluster head ");
    else if (pNb->mFollowing)
      printf("Following ");
    printf("\n");
  }
  printf("\n");
}

void MeshDevice::radioCallbackTx(RadioPacket* packet)
{
  mLastBeaconTime = mTimer->getTimerTime(packet->mStartTime);
  mInBeaconTX = false;
  if (mSearching)
    mRadio->setChannel(37);
  if (mIsCH)
  {
    for (MeshNeighbor* pChLeaf : mNeighbors)
    {
      if (pChLeaf->mClusterHead == mMyAddr)
      {
        pChLeaf->mLastSyncTime = mTimer->getTimerTime(packet->mStartTime);

        // alter timeout
        if (pChLeaf->mFollowing)
        {
          resync(pChLeaf);
        }
      }
    }
  }
  //_MESHLOG(mName, "Beacon!");
}

void MeshDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (mInSubscriptionRX && mCurrentRXTimer != 0)
  {
    mTimer->abort(mCurrentRXTimer);
    mCurrentRXTimer = 0;
    mInSubscriptionRX = false;
  }

  if (packet == NULL)
  {
    _MESHWARN(mName, "Packet is NULL");
    return;
  }
  if (corrupted)
  {
    _MESHWARN(mName, "Corrupted packet");
    return;
  }

  //_MESHLOG(mName, "RX!");

  mesh_packet_t* pMeshPacket = (mesh_packet_t*) packet->getContents();

  MeshNeighbor* pNb = getNeighbor(&pMeshPacket->adv_addr);
  if (pNb == NULL)
  {
    pNb = registerNeighbor(&pMeshPacket->adv_addr, mTimer->getTimerTime(packet->mStartTime), pMeshPacket, packet->getSender()->getDevice());
  }
  else
  {
    pNb->receivedBeacon(mTimer->getTimerTime(packet->mStartTime), pMeshPacket, packet->mChannel); // let neighbor structure update itself
  }

  // timing stuff:
  if (pNb->mClusterHead != mMyAddr)
    resync(pNb);

  if (pNb->isClusterHead())
  {
    // use as clusterhead if none exists
    if (mClusterHead == NULL && !mIsCH)
    {
      setClusterHead(pNb);
      _MESHLOG(mName, "Elected clusterhead: %s", pNb->mAdvAddr.toString().c_str());
    }
    else if (mClusterHead == pNb)
    {
      mTimer->reschedule(mBeaconTimerID, mTimer->getTimerTime(packet->mStartTime) + mCHBeaconOffset - MESH_TX_RU_TIME);
    }

    // sync all leaf node drifts
    for (MeshNeighbor* pChLeaf : mNeighbors)
    {
      if (pChLeaf->mClusterHead == pNb->mAdvAddr)
      {
        pChLeaf->mLastSyncTime = mTimer->getTimerTime(packet->mStartTime);

        // alter timeout
        if (pChLeaf->mFollowing)
        {
          //resync(&chLeaf);          
        }
      }
    }
  }
  else if (mClusterHead == NULL && !mIsCH && !mSearching)
  {
    electClusterHead();
  }
  if (pNb == mClusterHead)
  {
    mTimer->reschedule(mBeaconTimerID, mTimer->getTimerTime(packet->mStartTime) + mCHBeaconOffset - MESH_TX_RU_TIME);
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

MeshNeighbor* MeshDevice::getMostRedundantSubscription(void)
{
  if (mSubscriptions.size() == 0)
    return NULL;

  MeshNeighbor* pStrongest = mSubscriptions[0];

  if (mSearching)
  {
    for(auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
    {
      if (pStrongest->mNodeWeight > (*it)->mNodeWeight && mClusterHead != *it) // Want to keep track of best nodes during search
        pStrongest = *it;
    }
  }
  else
  {
    for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
    {
      if (pStrongest->getSubscriptionScore() < (*it)->getSubscriptionScore() && mClusterHead != *it) // Judged by mesh neighbor scoring function. Own clusterheads are never redundant
        pStrongest = *it;
    }
  }

  return pStrongest;
}

MeshNeighbor* MeshDevice::getLightestNeighbor(void)
{
  MeshNeighbor* pLightest = NULL;
  for (MeshNeighbor* pNb : mSubscriptions)
  {
    if ((pLightest == NULL || pNb->mNodeWeight < pLightest->mNodeWeight))
      pLightest = pNb;
  }

  return pLightest;
}

void MeshDevice::resync(MeshNeighbor* pNb)
{
  uint32_t nextBeaconTime = pNb->getNextBeaconTime(mTimer->getTimestamp());
  mTimer->reschedule(pNb->mRxTimer, nextBeaconTime - MESH_RX_RU_TIME - (nextBeaconTime - pNb->mLastSyncTime) * MESH_MAX_CLOCK_DRIFT_TWO_SIDED);
}

void MeshDevice::subscriptionTimeout(uint32_t timestamp, void* context)
{
  MeshNeighbor* pSub = (MeshNeighbor*) context;
  
  if (mInBeaconTX || mInSubscriptionRX || mSearching)
  {
    // yield

    return;
  }
  if (mRadio->getState() != Radio::RADIO_STATE_IDLE)
  {
    _MESHWARN(mName, "Radio wasn't in idle (state: %d)", mRadio->getState());
    mRadio->shortDisable();
    mRadio->disable();
  }
  mInSubscriptionRX = true;
  mRadio->setChannel(pSub->getNextChannel());
  mRadio->receive();
  //multiply drift by 2.0 to accomodate for the adjustment at the beginning and the end
  mCurrentRXTimer = mTimer->orderRelative(MESH_MAX_CLOCK_DRIFT_TWO_SIDED * 2.0 * (timestamp - pSub->mLastSyncTime) + MESH_RX_LISTEN_TIME,
    MEMBER_TIMEOUT(MeshDevice::rxStop), (void*)pSub);
  
}

void MeshDevice::rxStop(uint32_t timestamp, void* context)
{
  MeshNeighbor* pSub = (MeshNeighbor*)context;
  mRadio->shortDisable();
  mRadio->disable();
  _MESHWARN(mName, "RX timed out :(");
  pSub->mLostPackets++;
  // check if we should abandon the sub
  if (pSub->mLostPackets++ > MESH_MAX_LOSS_COUNT)
  {
    abortSubscription(pSub);
    bool lostClusterHead = (mClusterHead == pSub);
    if (mClusterHead == pSub)
    {
      mClusterHead = NULL;
      electClusterHead();
    }
  }

  mCurrentRXTimer = 0;
  mInSubscriptionRX = false;
}

void MeshDevice::beaconTimeout(uint32_t timestamp, void* context)
{
  mInBeaconTX = true;
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
    if (mIsCH)
    {
      mDefaultPacket.payload.str.payload.default.clusterAddr.set(mMyAddr);
    }
    else if (mClusterHead == NULL)
    {
      mDefaultPacket.payload.str.payload.default.clusterAddr.clear();
    }
    else
    {
      mDefaultPacket.payload.str.payload.default.clusterAddr.set(mClusterHead->mAdvAddr);
    }
    mDefaultPacket.payload.str.payload.default.nbCount = mSubscriptions.size();
    mDefaultPacket.payload.str.payload.default.nodeWeight = mNodeWeight;
    mDefaultPacket.payload.str.payload.default.offsetFromCH = mCHBeaconOffset;
    mDefaultPacket.payload.str.adv_len = MESH_PACKET_OVERHEAD_DEFAULT;

    mRadio->setPacket((uint8_t*)&mDefaultPacket, mDefaultPacket.header.length + MESH_PACKET_OVERHEAD - 4);
  }

  mRadio->setChannel(37 + ((1 + (mMyAddr.arr[0] % 3)) * mBeaconCount) % 3);

  if (mSearching)
  {
    mRadio->shortDisable();
    mRadio->disable(); 
    mRadio->transmit();
    mRadio->shortToRx();
  }
  else
  {
    if (mInSubscriptionRX) // abort it, own beacon is more important
    {
      mRadio->shortDisable();
      mRadio->disable();
    }
    mRadio->transmit();
  }
  ++mBeaconCount;
}

bool MeshDevice::electClusterHead(void)
{
#if 0
  // self elect?
  MeshNeighbor* pNb = getLightestNeighbor();
  if (pNb == NULL || pNb->mNodeWeight > mNodeWeight)
  {
    _MESHLOG(mName, "Becoming CH because my weight=%d, nb weight=%d", mNodeWeight, pNb->mNodeWeight);
    becomeCH();
    return true;
  }

  MeshNeighbor* pBestCH = NULL;
  for (MeshNeighbor* pNb : mSubscriptions)
  {
    if (pNb->isClusterHead() && (pBestCH == NULL || pNb->mNodeWeight < pBestCH->mNodeWeight))
    {
      pBestCH = pNb;
    }
  }
  if (pBestCH != NULL)
  {
    setClusterHead(pBestCH);
    return true;
  }
  
  return false;
}

void MeshDevice::subscribe(MeshNeighbor* pNb)
{
  mSubscriptions.push_back(pNb);

  pNb->mFollowing = true;
  mWSN->addConnection(this, pNb->mDev, false);

  uint32_t timeNow = mTimer->getTimestamp();
  uint32_t nextBeacon = pNb->getNextBeaconTime(timeNow);
  uint32_t deltaTime = (nextBeacon - pNb->mLastSyncTime);

  pNb->mRxTimer = mTimer->orderPeriodic(nextBeacon - deltaTime * MESH_MAX_CLOCK_DRIFT_TWO_SIDED - MESH_RX_RU_TIME,
    (int32_t)MESH_INTERVAL - (int32_t)(MESH_INTERVAL * MESH_MAX_CLOCK_DRIFT_TWO_SIDED),
    MEMBER_TIMEOUT(MeshDevice::subscriptionTimeout), pNb);

  if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS) // need to reduce set
  {
    MeshNeighbor* pStrongestSub = getMostRedundantSubscription();
    abortSubscription(pStrongestSub);
  }
}

void MeshDevice::processPacket(mesh_packet_t* pMeshPacket)
{
  if (hasMessageID(pMeshPacket->payload.str.msgID))
    return;

  switch (pMeshPacket->payload.str.adv_type)
  {
  case MESH_ADV_TYPE_DEFAULT:
    if (!pMeshPacket->payload.str.payload.default.clusterAddr.isNull())
    {
      MeshNeighbor* pCH = getNeighbor(&pMeshPacket->payload.str.payload.default.clusterAddr);
      if (pCH != NULL)
      {
        // set the CH neighbor to be its own CH, notified through proxy.
        pCH->mClusterHead.set(pCH->mAdvAddr);
      }
    }
    break;

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
    if (isSubscribedTo(&pMeshPacket->payload.str.payload.neighborRequest.neighbor))
    {
      transmitNeighborNotification(getNeighbor(&pMeshPacket->payload.str.payload.neighborRequest.neighbor));
    }
    break;
  }

  mMsgIDcache.registerID(pMeshPacket->payload.str.msgID);
}

void MeshDevice::becomeCH(void)
{
  mIsCH = true;
  mClusterHead = NULL;
  mCHBeaconOffset = 0;
  _MESHLOG(mName, "Became CH");
  mDevTag = DEVICE_TAG_MEDIUM;
  //TODO: establish appropriate connections and so on
}