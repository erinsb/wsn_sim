#include "MeshDevice.h"
#include "Timer.h"
#include "BlePacket.h"
#include "Radio.h"
#include "RadioPacket.h"
#include "Logger.h"
#include "MeshWSN.h"

#define SEARCHLIGHT_SLOT  (10000)
#define FIRST_CHANNEL     (37)
#define LAST_CHANNEL      (37)

using namespace std::placeholders;


static uint32_t getNextChannel(uint32_t lastChannel, uint8_t seed, uint8_t lostBeacons = 0)
{
  if (FIRST_CHANNEL == LAST_CHANNEL)
  {
    return FIRST_CHANNEL;
  }
  
  while (lostBeacons-- > 0)
  {
    lastChannel = getNextChannel(lastChannel, seed, 0);
  }
  if (seed & 0x01)
  {
    int32_t ch = lastChannel + 1;
    while (ch > LAST_CHANNEL)
      ch -= (LAST_CHANNEL - FIRST_CHANNEL);
    return ch;
  }
  else 
  {
    int32_t ch = lastChannel - 1;
    while (ch < FIRST_CHANNEL)
      ch += (LAST_CHANNEL - FIRST_CHANNEL);
    return ch;
  }
  //uint32_t add = (seed % (LAST_CHANNEL - FIRST_CHANNEL)) + 1;
  //return (lastChannel + (lostBeacons + 1) * add - FIRST_CHANNEL) % (LAST_CHANNEL - FIRST_CHANNEL + 1) + FIRST_CHANNEL;
}

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
  if (msgID == 0)
    return false;
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
  mLastBeaconTime(0),
  mBeaconCount(0), 
  mFollowing(0),
  mNbCount(0),
  mNodeWeight(0),
  mBeaconInterval(0),
  mRxTimer(0),
  mDebugTag(false)
{
  mClusterHead.clear();
  mAdvAddr.set(*adv_addr);
}

void MeshNeighbor::receivedBeacon(timestamp_t rxTime, mesh_packet_t* beacon, uint32_t channel, uint8_t signalStrength)
{
  // extract any state information 
  if (beacon != NULL)
  {
    ++mBeaconCount;
    mLastBeaconTime = rxTime;
    mLastChannel = channel;
    mSignalStrength = signalStrength;

    switch (beacon->payload.str.adv_type)
    {
    case MESH_ADV_TYPE_DEFAULT:
      mClusterHead.set(beacon->payload.str.payload.default.clusterAddr);
      mNbCount = beacon->payload.str.payload.default.nbCount;
      mNodeWeight = beacon->payload.str.payload.default.nodeWeight;
      mBeaconOffset = MESH_CH_OFFSET_US(beacon->payload.str.payload.default.offsetFromCH);
      break;
    case MESH_ADV_TYPE_SLEEPING:
      mClusterHead.set(beacon->payload.str.payload.sleeping.clusterAddr);
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

timestamp_t MeshNeighbor::getNextBeaconTime(timestamp_t timeNow)
{
  if (timeNow <= mLastBeaconTime)
    return mLastBeaconTime + MESH_INTERVAL;
  else
    return (1 + (timeNow - mLastBeaconTime) / MESH_INTERVAL) * MESH_INTERVAL + mLastBeaconTime;
}

uint32_t MeshNeighbor::getSubscriptionScore(void)
{
  return (255 - mSignalStrength);
}

uint32_t MeshNeighbor::getChannel(timestamp_t timestamp)
{
  return getNextChannel(mLastChannel, mAdvAddr.arr[0], getLostPacketCount(timestamp));
}

uint32_t MeshNeighbor::getLostPacketCount(timestamp_t timestamp)
{
  return (timestamp - mLastBeaconTime - MESH_INTERVAL / 2) / MESH_INTERVAL;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////

MeshDevice::MeshDevice(std::string name, double x, double y) :
  Device(x, y), 
  mSearching(false), 
  mCHBeaconOffset(((mRand(MESH_INTERVAL - 2 * MESH_CH_BEACON_MARGIN) + MESH_CH_BEACON_MARGIN) / 625) * 625), 
  mClusterHead(NULL),
  mNodeWeight(mRand() % 0xFF),
  mIsCH(false),
  mBeaconCount(0),
  mBeaconing(false), 
  mLastBeaconChannel(FIRST_CHANNEL),
  mClusterSync(false)
{
  mBackgroundPowerUsage = 0.007; // 7uA all the time
  mExtraPowerUsagePart = 1.05; // 105% more power than assumed
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
  mDefaultPacket.header.length = MESH_PACKET_OVERHEAD_DEFAULT;
}


MeshDevice::~MeshDevice()
{
}

void MeshDevice::start(void)
{
  // must use some time at the beginning to find devices
  startSearch();
  mTimer->orderRelative(MESH_INTERVAL + MESH_CH_BEACON_MARGIN, [](timestamp_t timeout, void* context)
  {
    MeshDevice* pMD = ((MeshDevice*)context);
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

  mRadio->setChannel(FIRST_CHANNEL);
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
  if (mBeaconCount == 0)
    mLastBeaconChannel = FIRST_CHANNEL;
  mBeaconCount = 0;
  if (mClusterHead != NULL)
    mBeaconTimerID = mTimer->orderPeriodic(mClusterHead->getNextBeaconTime(mTimer->getTimestamp()) + mCHBeaconOffset, MESH_INTERVAL, MEMBER_TIMEOUT(MeshDevice::beaconTimeout));
  else
    mBeaconTimerID = mTimer->orderPeriodic(mTimer->getTimestamp() + mCHBeaconOffset, MESH_INTERVAL, MEMBER_TIMEOUT(MeshDevice::beaconTimeout));
  mBeaconing = true;
}

void MeshDevice::stopBeaconing(void)
{
  mBeaconing = false;
  mTimer->abort(mBeaconTimerID);
}

MeshNeighbor* MeshDevice::registerNeighbor(ble_adv_addr_t* advAddr, timestamp_t rxTime, mesh_packet_t* pPacket, uint8_t signalStrength, Device* pDev)
{
  MeshNeighbor* pNb = new MeshNeighbor(advAddr);
  mNeighbors.push_back(pNb);
  pNb->mDev = (MeshDevice*) pDev;

  pNb->receivedBeacon(rxTime, pPacket, mRadio->getChannel(), signalStrength);
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
  //_MESHWARN(mName, "Aborted sub to %s", pSub->mDev->mName.c_str());
  for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
  {
    if (*it == pSub)
    {
      MeshNeighbor* pNb = *it;
      mWSN->removeConnection(this, pNb->mDev);
      pNb->mFollowing = false;
      mTimer->abort(pNb->mRxTimer);
      mSubscriptions.erase(it);
      break;
    }
  }

  if (pSub == mClusterHead && mClusterHead != NULL)
  {
    mClusterHead = NULL;
    electClusterHead();
  }

  if (mClusterSleeps)
    return;

  while (mSubscriptions.size() < MESH_OPTIMAL_SUBSCRIPTIONS && mNeighbors.size() > 6)
  {
    MeshNeighbor* pBestNb = NULL;
    for (MeshNeighbor* pNb : mNeighbors)
    {
      if (pNb == pSub || pNb->mFollowing == true) // don't resubscribe to the recently lost node
        continue;
      if (pBestNb == NULL || pNb->mNodeWeight < pBestNb->mNodeWeight)
      {
        pBestNb = pNb;
      }
    }
    if (pBestNb == NULL)
      break;

    resubscribe(pBestNb);
  }

#if 0
  // fall asleep if I am a connector node and last external CH is lost
  if (!mIsCH && mClusterSleeps && mBeaconing) // is connector node
  {
    uint8_t chCount = 0;
    for (MeshNeighbor* pNb : mSubscriptions)
    {
      if (pNb->isClusterHead())
        chCount++;
    }
    if (chCount <= 1)
    {
      transmitSleep();
    }
  }
#endif
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

void MeshDevice::transmitSleep(void)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_SLEEPING;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_SLEEPING;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_SLEEPING;
  pPacket->adv_addr.set(mMyAddr);
  if (mClusterHead != NULL)
    pPacket->payload.str.payload.sleeping.clusterAddr.set(mClusterHead->mAdvAddr);
  else if (mIsCH)
    pPacket->payload.str.payload.sleeping.clusterAddr.set(mMyAddr);
  else
    pPacket->payload.str.payload.sleeping.clusterAddr.clear();
  pPacket->payload.str.payload.sleeping.offsetFromCH = mCHBeaconOffset;

  mPacketQueue.push(pPacket);
}

void MeshDevice::transmitClusterjoin(void)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_JOIN_CLUSTER;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_JOIN_CLUSTER;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_JOIN_CLUSTER;
  for (uint8_t i = 0; i < 4; ++i)
    pPacket->payload.str.payload.join_cluster.node[i].clear();
  pPacket->adv_addr.set(mMyAddr);

  mPacketQueue.push(pPacket);
}

void MeshDevice::setClusterHead(MeshNeighbor* nb)
{
  mCHBeaconOffset = (MESH_INTERVAL + mTimer->getExpiration(mBeaconTimerID) + MESH_TX_RU_TIME - nb->mLastBeaconTime) % MESH_INTERVAL;
  mClusterSync = false;
  /* start requesting cluster position */
  mDefaultPacket.payload.str.adv_type = MESH_ADV_TYPE_CLUSTER_REQ;

  if (mClusterHead != NULL)
  {
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

void MeshDevice::optimizeSubscriptions(void)
{
  while (mSubscriptions.size() > MESH_OPTIMAL_SUBSCRIPTIONS)
  {
    MeshNeighbor* pSubToDestroy = NULL;
    for (MeshNeighbor* pNb : mSubscriptions)
    {
      if (pNb->isClusterHead()) // clusterheads are always valuable subs
        continue;
      if (pSubToDestroy == NULL || pNb->mNodeWeight < pSubToDestroy->mNodeWeight) // kill strongest
      {
        pSubToDestroy = pNb;
      }
    }
    abortSubscription(pSubToDestroy);
  }
}

void MeshDevice::addRoute(ble_adv_addr_t* addr, uint32_t dist)
{
  timestamp_t timeNow = mTimer->getTimestamp();
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

void MeshDevice::setCHBeaconOffset(timestamp_t beaconOffset)
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

  mesh_packet_t* pMeshPacket = (mesh_packet_t*)packet->getContents();

  mLastBeaconTime = mTimer->getTimerTime(packet->mStartTime);
  mInBeaconTX = false;
  if (mSearching)
    mRadio->setChannel(FIRST_CHANNEL);
  if (mIsCH)
  {
    for (MeshNeighbor* pChLeaf : mNeighbors)
    {
      if (pChLeaf->mClusterHead == mMyAddr)
      {
        //pChLeaf->mLastSyncTime = mTimer->getTimerTime(packet->mStartTime);

        // alter timeout
        if (pChLeaf->mFollowing)
        {
          //resync(pChLeaf);
        }
      }
    }
  }
  if (mIsCH && (mBeaconCount - mFirstBeaconTX) == 10)
  {
    //transmitSleep(); // send the entire cluster to sleep
  }
  if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_SLEEPING)
  {
    if (!mIsCH)
      sleep();
    else
    {
      mClusterSleeps = true;
    }
  }

}

void MeshDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (!mSearching)
    mRadio->shortDisable();
  if (mInSubscriptionRX && mCurrentRXTimer != 0)
  {
    mTimer->abort(mCurrentRXTimer);
    mCurrentRXTimer = 0;
    mInSubscriptionRX = false;
  }

  if (!hasClusterHead() && !mSearching)
  {
    electClusterHead();
  }

  ((MeshWSN*)mWSN)->logReceive();


  if (packet == NULL)
  {
    _MESHERROR(mName, "Packet is NULL");
    return;
  }
  if (corrupted)
  {
    if (!mSearching)
      lostPacket(mCurrentSub);
    ((MeshWSN*)mWSN)->logCorruption();
    return;
  }

  mesh_packet_t* pMeshPacket = (mesh_packet_t*) packet->getContents();
  MeshNeighbor* pNb = getNeighbor(&pMeshPacket->adv_addr);

  int64_t drift = 0; 

  if (pNb == NULL)
  {
    pNb = registerNeighbor(&pMeshPacket->adv_addr, mTimer->getTimerTime(packet->mStartTime), pMeshPacket, rx_strength, packet->getSender()->getDevice());
  }
  else
  {
    drift = mTimer->getTimerTime(packet->mStartTime) - pNb->mLastBeaconTime - MESH_INTERVAL;
    //if (pNb == mClusterHead)
    //  _MESHLOG(mName, "Drift: %d", drift);
    pNb->receivedBeacon(mTimer->getTimerTime(packet->mStartTime), pMeshPacket, packet->mChannel, rx_strength); // let neighbor structure update itself
  }

  // bump timer drift
  resync(pNb);

  processPacket(pMeshPacket, mTimer->getTimerTime(packet->mStartTime));

  if (pNb->isClusterHead())
  {
    // use as clusterhead if none exists
    if (mClusterHead == NULL && !mIsCH)
    {
      setClusterHead(pNb);
      _MESHLOG(mName, "Elected clusterhead: %s", pNb->mDev->mName.c_str());
    }
    else if (mClusterHead == pNb && mBeaconing)
    {
      //mTimer->reschedule(mBeaconTimerID, mLastBeaconTime + MESH_INTERVAL + drift - MESH_TX_RU_TIME); // mTimer->getTimerTime(packet->mStartTime) + mCHBeaconOffset
    }
  }

  mCurrentSub = NULL;

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
      if (pStrongest->mNodeWeight < (*it)->mNodeWeight && mClusterHead != *it) // Want to keep track of best nodes during search
        pStrongest = *it;
    }
  }
  else
  {
    for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
    {
      if (pStrongest->getSubscriptionScore() > (*it)->getSubscriptionScore() && mClusterHead != *it) // Judged by mesh neighbor scoring function. Own clusterheads are never redundant
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
    if (pLightest == NULL || pNb->mNodeWeight < pLightest->mNodeWeight)
      pLightest = pNb;
  }

  return pLightest;
}

void MeshDevice::resync(MeshNeighbor* pNb)
{
  if (pNb->mFollowing) // safety for when a neighbor sub has been cancelled
  {
    timestamp_t nextBeaconTime = pNb->getNextBeaconTime(mTimer->getTimestamp());
    mTimer->reschedule(pNb->mRxTimer, nextBeaconTime - MESH_RX_RU_TIME - (nextBeaconTime - pNb->mLastBeaconTime) * MESH_MAX_CLOCK_DRIFT_TWO_SIDED);
  }
}

void MeshDevice::subscriptionTimeout(timestamp_t timestamp, void* context)
{
  MeshNeighbor* pSub = (MeshNeighbor*) context;
  if (mSearching) // are already listening
  {
    mCurrentSub = pSub;
    return;
  }

  if (mInBeaconTX)
  {
    lostPacket(pSub); // beacon is more important
    return;
  }

  if (mInSubscriptionRX)
  {
    if (mCurrentSub == mClusterHead || (mCurrentSub->getLostPacketCount(timestamp) >= pSub->getLostPacketCount(timestamp) && !mCurrentSub->isClusterHead()))
    {
      // yield
      lostPacket(pSub);
      return;
    }
    else
    {
      mCurrentSub->mDebugTag = true;
      mTimer->abort(mCurrentRXTimer);
      lostPacket(mCurrentSub);
      mRadio->shortDisable();
      mRadio->disable();
      _MESHWARN(mName, "Yielded %s for %s", mCurrentSub->mDev->mName.c_str(), pSub->mDev->mName.c_str());
    }
  }

  mInSubscriptionRX = true;
  mCurrentSub = pSub;
  // must ensure that we don't count upcoming packet as lost
  mRadio->setChannel(pSub->getChannel(timestamp - 1000));
  mRadio->receive();
  //multiply drift by 2.0 to accomodate for the adjustment at the beginning and the end
  //MeshNeighbor* pCH = getNeighbor(&pSub->mClusterHead);
  timestamp_t driftOffset = (timestamp - pSub->mLastBeaconTime);// (pSub->mBeaconCount > 3 && !(pCH != NULL && pCH->mFollowing)) ? (timestamp - pSub->mLastBeaconTime) : 3 * MESH_INTERVAL;

  mCurrentRXTimer = mTimer->orderRelative(MESH_MAX_CLOCK_DRIFT_TWO_SIDED * 2.0 * driftOffset + MESH_RX_LISTEN_TIME,
    MEMBER_TIMEOUT(MeshDevice::rxStop), (void*)pSub);
  
}

void MeshDevice::rxStop(timestamp_t timestamp, void* context)
{
  if (mRadio->rxInProgress())
    return; // cancel the abort and wait for RX

  MeshNeighbor* pSub = (MeshNeighbor*)context;
  mRadio->shortDisable();
  mRadio->disable();

  if (pSub->mDev->mBeaconing)
    _MESHWARN(mName, "RX timed out\tmissing: %s (lost: %d). \tMy ch: %d, tx ch: %d", pSub->mDev->mName.c_str(), pSub->getLostPacketCount(timestamp), mRadio->getChannel(), pSub->mDev->mRadio->getChannel());
  else
    _MESHWARN(mName, "RX timed out\tmissing: %s (lost: %d). It's asleep.", pSub->mDev->mName.c_str(), pSub->getLostPacketCount(timestamp));

  lostPacket(pSub);

  mCurrentRXTimer = 0;
  mInSubscriptionRX = false;
  mCurrentSub = NULL;
}

void MeshDevice::beaconTimeout(timestamp_t timestamp, void* context)
{
  mInBeaconTX = true;
  bool updateDefaultPacket = (mPacketQueue.size() == 0 || mPacketQueue.front() == &mDefaultPacket);
  mesh_packet_t* pPacket = NULL;
  // send packet from queue, if any. 
  if (mPacketQueue.size() > 0)
  {
    pPacket = mPacketQueue.front();
    mPacketQueue.pop();
    string nodestr = "";

    if (pPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER)
    {
      if (mClusterRequesters.size() == 0)
      {
        updateDefaultPacket = true;
      }
      else
      {
        pPacket->payload.str.payload.join_cluster.first_slot = mClusterLeafCount;
        for (uint32_t i = 0; i < 4; ++i)
        {
          if (mClusterRequesters.size() == 0)
            break;
        
          pPacket->payload.str.payload.join_cluster.node[i].set(mClusterRequesters.front()->mAdvAddr);
          mClusterRequesters.pop();
          mClusterLeafCount++;
          nodestr += "\n\t" + pPacket->payload.str.payload.join_cluster.node[i].toString();
        }
        _LOG("CH sending join packet to %s", nodestr.c_str());
      }
    }
  }
  if (updateDefaultPacket)
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
    mDefaultPacket.payload.str.payload.default.offsetFromCH = MESH_CH_OFFSET_SLOTS(mCHBeaconOffset);
    mDefaultPacket.payload.str.adv_len = MESH_PACKET_OVERHEAD_DEFAULT;
    pPacket = &mDefaultPacket;
  }
  mRadio->setPacket((uint8_t*)pPacket, pPacket->header.length + MESH_PACKET_OVERHEAD);
  
  mLastBeaconChannel = (mLastBeaconTime > 0)? getNextChannel(mLastBeaconChannel, mMyAddr.arr[0]) : FIRST_CHANNEL;
  mRadio->setChannel(mLastBeaconChannel);

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
      _MESHLOG(mName, "delayed beacon to avoid sub");
      //lostPacket(mCurrentSub);
      //mTimer->abort(mCurrentRXTimer);
      mRadio->shortToTx();
      //mRadio->disable();
      //mInSubscriptionRX = false;
      //mCurrentSub = NULL;
    }
    else
    {
      mRadio->transmit();
    }
  }

  ((MeshWSN*)mWSN)->logTransmit();

  ++mBeaconCount;
}

bool MeshDevice::electClusterHead(void)
{
  MeshNeighbor* pStrongestCH = NULL;
  MeshNeighbor* pStrongestNeighbor = NULL;
  bool hasAvailableNeighbor = false;
  for (MeshNeighbor* pNb : mSubscriptions)
  {
    if (pNb->isClusterHead() && (pStrongestCH == NULL || pNb->mNodeWeight < pStrongestCH->mNodeWeight))
    {
      pStrongestCH = pNb;
    }
    else if (pNb->mClusterHead.isNull() && (pStrongestNeighbor == NULL || pNb->mNodeWeight < pStrongestNeighbor->mNodeWeight))
    {
      pStrongestNeighbor = pNb;
    }
    if (pNb->mClusterHead.isNull())
      hasAvailableNeighbor = true;
  }

  if (pStrongestCH != NULL)
  {
    mClusterHead = pStrongestCH;

    return true;
  }
  else if (pStrongestNeighbor == NULL || pStrongestNeighbor->mNodeWeight > mNodeWeight)
  {
    becomeCH();
    return true;
  }
  else if (!hasAvailableNeighbor)
  {
    becomeCH();
    return true;
  }

  return false;
}

void MeshDevice::subscribe(MeshNeighbor* pNb)
{
  mSubscriptions.push_back(pNb);

  pNb->mFollowing = true;
  pNb->mBeaconCount = 0;
  mWSN->addConnection(this, pNb->mDev, false);

  timestamp_t timeNow = mTimer->getTimestamp();
  timestamp_t nextBeacon = pNb->getNextBeaconTime(timeNow);
  timestamp_t deltaTime = (nextBeacon - pNb->mLastBeaconTime);

  pNb->mRxTimer = mTimer->orderPeriodic(nextBeacon - deltaTime * MESH_MAX_CLOCK_DRIFT_TWO_SIDED - MESH_RX_RU_TIME,
    (int32_t)MESH_INTERVAL - (int32_t)(MESH_INTERVAL * MESH_MAX_CLOCK_DRIFT_TWO_SIDED),
    MEMBER_TIMEOUT(MeshDevice::subscriptionTimeout), pNb);

  if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS) // need to reduce set
  {
    MeshNeighbor* pStrongestSub = getMostRedundantSubscription();
    abortSubscription(pStrongestSub);
  }
}

void MeshDevice::resubscribe(MeshNeighbor* pNb)
{
  if (pNb->mFollowing)
    _MESHERROR(mName, "Already has subscription to %s", pNb->mDev->mName.c_str());

  mSubscriptions.push_back(pNb);

  pNb->mFollowing = true;
  pNb->mBeaconCount = 0;
  mWSN->addConnection(this, pNb->mDev, false);

  MeshNeighbor* pCH = getNeighbor(&pNb->mClusterHead);

  timestamp_t timeNow = mTimer->getTimestamp();
  timestamp_t nextBeacon = pNb->getNextBeaconTime(timeNow);
  timestamp_t deltaTime = (nextBeacon - pNb->mLastBeaconTime);
  if (pCH != NULL && pCH->mFollowing)
  {
    nextBeacon = pCH->getNextBeaconTime(timeNow) + pNb->mBeaconOffset - 625;
    deltaTime = pNb->mBeaconOffset;
    //pNb->mLastBeaconTime = nextBeacon - MESH_INTERVAL;
  }

  pNb->mRxTimer = mTimer->orderPeriodic(nextBeacon - deltaTime * MESH_MAX_CLOCK_DRIFT_TWO_SIDED - MESH_RX_RU_TIME,
    (int32_t)MESH_INTERVAL - (int32_t)(MESH_INTERVAL * MESH_MAX_CLOCK_DRIFT_TWO_SIDED),
    MEMBER_TIMEOUT(MeshDevice::subscriptionTimeout), pNb);

  if (mSubscriptions.size() >= MESH_MAX_SUBSCRIPTIONS) // need to reduce set
  {
    MeshNeighbor* pStrongestSub = getMostRedundantSubscription();
    abortSubscription(pStrongestSub);
  }
}

void MeshDevice::processPacket(mesh_packet_t* pMeshPacket, timestamp_t start_time)
{
  if (hasMessageID(pMeshPacket->payload.str.msgID))
  {
    _MESHWARN(mName, "Got message I don't care about: %d", pMeshPacket->payload.str.msgID);
    return;
  }

  MeshNeighbor* pSender = getNeighbor(&pMeshPacket->adv_addr);

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
  case MESH_ADV_TYPE_CLUSTER_REQ:
    if (pMeshPacket->payload.str.payload.cluster_req.clusterAddr == mMyAddr && mIsCH)
    {
      mClusterRequesters.push(getNeighbor(&pMeshPacket->adv_addr));
      transmitClusterjoin();
    }
    break;
  case MESH_ADV_TYPE_JOIN_CLUSTER:
    for (uint8_t i = 0; i < 4; ++i)
    {
      if (pMeshPacket->payload.str.payload.join_cluster.node[i] == mMyAddr)
      {
        // move beacon to the number in the slot
        mClusterSync = true;
        mTimer->reschedule(mBeaconTimerID, start_time + MESH_INTERVAL + MESH_CH_OFFSET_US(pMeshPacket->payload.str.payload.join_cluster.first_slot + i));
        mDefaultPacket.payload.str.adv_type = MESH_ADV_TYPE_DEFAULT;
        break;
      }
    }
    break;
  case MESH_ADV_TYPE_SLEEPING:
    if (pSender == mClusterHead && mClusterHead != NULL) // clusterhead commanded sleep. Do so.
    {
      mClusterSleeps = true;
      uint8_t chCount = 0;
      std::vector<MeshNeighbor*> abortList;
      for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
      {
        if ((*it)->isClusterHead())
          chCount++;
        else
          abortList.push_back(*it);
      }

      // abort all non-essential subscriptions
      for (auto it = abortList.begin(); it != abortList.end(); it++)
        abortSubscription(*it);

      if (chCount <= 1)
      {
        transmitSleep();
        mDevTag = DEVICE_TAG_STRONG; // sleep tag
      }
    }
    else if (!pSender->isClusterHead())
    {
      abortSubscription(pSender);
    }
    else if (mIsCH)
    {
      _MESHLOG(mName, "CH aborting sub to slave");
      abortSubscription(pSender);
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
  mClusterLeafCount = 0;
  _MESHLOG(mName, "Became CH");
  ((MeshWSN*)mWSN)->logClusterHead(this);
  mDevTag = DEVICE_TAG_MEDIUM;
  mFirstBeaconTX = mBeaconCount;
}

void MeshDevice::lostPacket(MeshNeighbor* pNb)
{
  pNb->mBeaconCount++;
  // check if we should abandon the sub
  if (pNb->mBeaconCount > 3 && pNb->getLostPacketCount(mTimer->getTimestamp()) > MESH_MAX_LOSS_COUNT && pNb->mFollowing)
  {
    abortSubscription(pNb);
    bool lostClusterHead = (mClusterHead == pNb);
    if (mClusterHead == pNb)
    {
      mClusterHead = NULL;
      electClusterHead();
    }
  }
}

void MeshDevice::sleep(void)
{
  for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
  {
    if (!(*it)->isClusterHead())
    {
      abortSubscription(*it);
      it = mSubscriptions.begin();
    }
  }
  if (mSubscriptions.size() <= 1) // isn't connector node
    stopBeaconing();

  mClusterSleeps = true;
}

void MeshDevice::prepareBeacon(void)
{

}