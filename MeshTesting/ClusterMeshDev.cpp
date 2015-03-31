#include "ClusterMeshDev.h"
#include "Logger.h"

#define MESH_CHANNEL  (37)

ClusterMeshDev::ClusterMeshDev(std::string name, double x, double y)
{
  double driftFactor = ((MESH_MAX_CLOCK_DRIFT)* (2.0 * mRand.Float() - 1.0)) + 1.0; // 1.0 +- <250/1mill 
  mTimer->setDriftFactor(driftFactor);
  mName = name;
  pos.x = x;
  pos.y = y;

  mMyCluster = NULL;
  mMyClusterOffset = 0;

  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    mAdvAddr.arr[i] = mRand() & 0xFF;
  }
  mDefaultPacket.push(mesh_packet_t());
  memset(&mDefaultPacket.top(), 0, sizeof(mesh_packet_t));
  mDefaultPacket.top().payload.str.adv_type = MESH_ADV_TYPE_DEFAULT;
  mDefaultPacket.top().access_addr = MESH_ACCESS_ADDR;
  mDefaultPacket.top().adv_addr.set(mAdvAddr);
  mDefaultPacket.top().payload.str.adv_len = MESH_PACKET_OVERHEAD_DEFAULT;
  mDefaultPacket.top().payload.str.msgID = 0;
  mDefaultPacket.top().payload.str.payload.default.nodeWeight = mScore;
  mDefaultPacket.top().payload.str.payload.default.clusterAddr.clear();
  mDefaultPacket.top().payload.str.payload.default.nbCount = 0;
  mDefaultPacket.top().header.length = MESH_PACKET_OVERHEAD_DEFAULT;
}


ClusterMeshDev::~ClusterMeshDev()
{
}

void ClusterMeshDev::start(void)
{
  doRecon();
  mTimer->orderRelative(MESH_INTERVAL * 2, [this](timestamp_t, void*){ doMakeCluster(); }); // don't cleanup recon, we want to continue searching
}

void ClusterMeshDev::cleanupPrevState(void)
{
  switch (mState)
  {
    case CM_STATE_RECON:
      mRadio->shortDisable();
      mRadio->disable();
      break;
    case CM_STATE_MAKE_CLUSTER:
      mRadio->shortDisable();
      mRadio->disable();
      break;
    case CM_STATE_REQ_CLUSTER:
      mRadio->shortDisable();
      mRadio->disable();
      break;
    case CM_STATE_CH_SCAN:
      mRadio->shortDisable();
      if (mRadio->getState() == Radio::RADIO_STATE_RX)
      {
        mRadio->disable();
      }
      break;
    case CM_STATE_CH:
      mTimer->abort(mBeaconTimer);
      break;
    case CM_STATE_LEAF:
      mTimer->abort(mChTimer);
      break;
  }
}

void ClusterMeshDev::doRecon(void)
{
  if (mRadio->getState() != Radio::RADIO_STATE_RX)
  {
    mRadio->setChannel(MESH_CHANNEL);
    mRadio->shortToRx(); // rx forever
    mRadio->receive();
  }

  mBeaconTimer = mTimer->orderRelative(mRand.Float() * (MESH_INTERVAL - 1 * MS) + 1 * MS, [this](timestamp_t, void*) { radioBeaconTX(); });
  setState(CM_STATE_RECON);
}

void ClusterMeshDev::doMakeCluster(void)
{
  setState(CM_STATE_MAKE_CLUSTER);

  if (mRadio->getState() != Radio::RADIO_STATE_RX)
  {
    mRadio->setChannel(MESH_CHANNEL);
    mRadio->shortToRx(); // rx forever
    mRadio->receive();
  }

  makeClusterCheck();
}

void ClusterMeshDev::setCH(MeshNeighbor* pCH)
{
  setState(CM_STATE_REQ_CLUSTER);
  mClusterHead = pCH;

  if (!isSubscribedTo(pCH))
  {
    subscribe(pCH);
  }

  // send request packet to cluster
  transmitClusterReq();
}

void ClusterMeshDev::becomeCH(void)
{
  timestamp_t timeNow = mTimer->getTimestamp();
  setState(CM_STATE_CH_SCAN);

  mTimer->abort(mBeaconTimer);
  mBeaconTimer = mTimer->orderPeriodic(timeNow + mRand.Float() * MESH_INTERVAL + MESH_CLUSTER_SLOT_US, MESH_INTERVAL, [=](timestamp_t, void*){ radioBeaconTX(); });
  mTimer->orderRelative(MESH_INTERVAL * 4, [=](timestamp_t, void*){ cleanupPrevState(); setState(CM_STATE_CH); }); // Stop scanning for participants after a while

  mRadio->shortToRx(); // scan "forever"
  if (mRadio->getState() != Radio::RADIO_STATE_RX)
  {
    mRadio->disable();
  }

  mClusters.push_back(MeshCluster(&mAdvAddr));
  mMyCluster = &mClusters.back();
  mMyClusterOffset = 0;
}

void ClusterMeshDev::subscribe(MeshNeighbor* pNb)
{
  timestamp_t nextBeacon = pNb->getNextBeaconTime(mTimer->getTimestamp());

  mSubscriptions.push_back(pNb);

  MeshCluster* pCluster = getCluster(&pNb->mClusterHead);
  if (pCluster != NULL)
  {
    // only schedule timer if it's the first in its cluster. 
    bool foundFirst = false;
    for (auto pClusterDev : pCluster->mDevices)
    {
      if (pClusterDev != NULL)
      {
        if (pClusterDev == pNb)
        {
          if (!foundFirst)
          {
            pNb->mRxTimer = mTimer->orderPeriodic((timestamp_t)(nextBeacon - MESH_RX_RU_TIME - MESH_MAX_CLOCK_DRIFT_TWO_SIDED * (nextBeacon - pNb->mLastBeaconTime)), 
              MESH_INTERVAL * (1.0 - MESH_MAX_CLOCK_DRIFT_TWO_SIDED),
              [=](timestamp_t, void*){ radioSubRX(pNb); });
          }
        }
        else
        {
          if (foundFirst) // the new sub is before this one in the cluster, we don't need this timer anymore 
          {
            mTimer->abort(pClusterDev->mRxTimer);
          }
        }
        if (isSubscribedTo(pClusterDev))
          foundFirst = true;
      }
    }
  }

  mWSN->addConnection(this, pNb->mDev);
}

void ClusterMeshDev::makeClusterCheck(void)
{
  MeshNeighbor* pBestNb = getBestNb([](MeshNeighbor* pNb) -> bool{ return (pNb->mClusterHead.isNull()); }); // unbound devices
  MeshNeighbor* pBestCH = getBestNb([](MeshNeighbor* pNb) -> bool{ return (pNb->isClusterHead()); });

  if (pBestCH != NULL)
  {
    setCH(pBestCH);
    return;
  }

  if (pBestNb == NULL || pBestNb->mNodeWeight < mScore)
  {
    if (pBestNb == NULL)
      _MESHLOG(mName, "No available neighbors");
    else
      _MESHLOG(mName, "Best NB: %d, me: %d", pBestNb->mNodeWeight, mScore);
    becomeCH();
    return;
  }
  _MESHLOG(mName, "No nearby CH's, and I suck");
  //keep waiting
}

bool ClusterMeshDev::isSubscribedTo(MeshNeighbor* pNb)
{
  for (auto pSub : mSubscriptions)
  {
    if (pSub == pNb)
      return true;
  }
  return false;
}

MeshCluster* ClusterMeshDev::getCluster(ble_adv_addr_t* pCHaddr)
{
  for (auto& cluster : mClusters)
  {
    if (cluster.mCHaddr == *pCHaddr)
      return &cluster;
  }
  return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MeshNeighbor* ClusterMeshDev::getBestNb(std::function<bool(MeshNeighbor*)> filterFunc)
{
  MeshNeighbor* pBestNb = NULL;
  for (auto pNb : mNeighbors)
  {
    if ((filterFunc == NULL || filterFunc(pNb)) && (pBestNb == NULL || pNb->mNodeWeight > pBestNb->mNodeWeight))
    {
      pBestNb = pNb;
    }
  }

  return pBestNb;
}

MeshNeighbor* ClusterMeshDev::getNb(ble_adv_addr_t* pAdvAddr)
{
  for (auto pNb : mNeighbors)
  {
    if (pNb->mAdvAddr == *pAdvAddr)
    {
      return pNb;
    }
  }
  return NULL;
}

void ClusterMeshDev::radioCallbackTx(RadioPacket* pPacket)
{
  timestamp_t txTime = mTimer->getTimerTime(pPacket->mStartTime);
  if (mMyCluster != NULL && isCH())
    orderRestOfCluster(mMyCluster, mMyClusterOffset, txTime);
}

void ClusterMeshDev::radioCallbackRx(RadioPacket* pPacket, uint8_t rx_strength, bool corrupted)
{
  mRadio->removeFilter();
  mTimer->abort(mCurrentSubAbortTimer);

  if (corrupted)
  {
    return;
  }
  mesh_packet_t* pMeshPacket = (mesh_packet_t*) pPacket->getContents();
  MeshNeighbor* pSender = getNb(&pMeshPacket->adv_addr);
  timestamp_t rxTime = mTimer->getTimerTime(pPacket->mStartTime);

  // register newfound neighbor
  if (pSender == NULL)
  {
    pSender = new MeshNeighbor(&pMeshPacket->adv_addr);
    pSender->mDev = pPacket->getSender()->getDevice();
    mNeighbors.push_back(pSender);
  }

  // update neighbor structure
  pSender->receivedBeacon(rxTime, pMeshPacket, pPacket->mChannel, rx_strength);

  if (isSubscribedTo(pSender))
  {
    if (mTimer->isValidTimer(pSender->mRxTimer)) // may have been ordered into cluster train
    {
      mTimer->reschedule(pSender->mRxTimer, rxTime + (MESH_INTERVAL - MESH_RX_RU_TIME) * (1.0 - MESH_MAX_CLOCK_DRIFT_TWO_SIDED));
    }
  }
  
  // setup any chained events
  if (!pSender->mClusterHead.isNull())
  {
    MeshCluster* pCluster = getCluster(&pSender->mClusterHead);
    if (pCluster == NULL && pSender->mBeaconOffset < MESH_MAX_CLUSTER_SIZE)
    {
      mClusters.push_back(MeshCluster(&pSender->mClusterHead));
      pCluster = &mClusters.back();
      pCluster->mDevices[pSender->mBeaconOffset] = pSender; // may have to update this other places when introducing cluster reorganization
    } 

    
    if (pCluster->getFirstDeviceIndex() == pSender->mBeaconOffset &&
      (pCluster != mMyCluster || pSender->mBeaconOffset < mMyClusterOffset)) // must be first beacon in cluster (including own)
    {
      orderRestOfCluster(pCluster, pSender->mBeaconOffset, rxTime);
    }
  }

  // special behavior
  switch (mState)
  {
    case CM_STATE_MAKE_CLUSTER:
      makeClusterCheck();
      break;
    case CM_STATE_REQ_CLUSTER:
      if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER)
      {
        MeshCluster* pCluster = getCluster(&pSender->mAdvAddr);

        for (uint8_t i = 0; i < 4; ++i)
        {
          uint32_t clusterOffset = pMeshPacket->payload.str.payload.join_cluster.first_slot + i;

          if (pMeshPacket->payload.str.payload.join_cluster.node[i] == mAdvAddr)
          {
            // join cluster train
            mMyClusterOffset = clusterOffset;
            mClusterHead = pSender; // in case there was any doubt (naive?)
            mMyCluster = pCluster;
            mTimer->abort(mBeaconTimer); // beacon will now rely on cluster head
            cleanupPrevState();
            setState(CM_STATE_LEAF);
            for (uint32_t j = 0; j < mNeighbors.size(); ++j)
            {
              if (mNeighbors[j] != pSender)
              {
                subscribe(mNeighbors[j]);
                break;
              }
            }
            mDefaultPacket.pop(); // take away request packet
          }
          else /* register newfound info about neighbor */
          {
            MeshNeighbor* pNb = getNb(&pMeshPacket->payload.str.payload.join_cluster.node[i]);
            if (pNb != NULL)
            {
              if (isSubscribedTo(pNb))
              {
                mTimer->abort(pNb->mRxTimer);
              }
              pNb->mLastBeaconTime = rxTime + clusterOffset * MESH_CLUSTER_SLOT_US;
              pNb->mClusterHead = pSender->mAdvAddr;
              pCluster->mDevices[clusterOffset] = pNb;
            }
          }

        }
      }
      break;
    case CM_STATE_CH:
    case CM_STATE_CH_SCAN:
      if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_CLUSTER_REQ && pMeshPacket->payload.str.payload.cluster_req.clusterAddr == mAdvAddr)
      {
        mClusterReqs.push(pSender);
        transmitClusterjoin();
      }
      break;
    default: 
      break;
  }


}

void ClusterMeshDev::processPacket(mesh_packet_t* pMeshPacket)
{
  
}

void ClusterMeshDev::radioBeaconTX(void)
{
  if (mPacketQueue.empty())
  {
    if (mState == CM_STATE_CH || mState == CM_STATE_CH_SCAN)
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.set(mAdvAddr);
    else if (mClusterHead != NULL)
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.set(mClusterHead->mAdvAddr);
    else
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.clear();


    mDefaultPacket.top().payload.str.payload.default.clusterMax = mMyCluster->getLastDeviceIndex() + 1;
    mDefaultPacket.top().payload.str.payload.default.nodeWeight = mScore;
    mDefaultPacket.top().payload.str.payload.default.offsetFromCH = mMyClusterOffset;
    mDefaultPacket.top().header.length = MESH_PACKET_OVERHEAD_DEFAULT;
    mRadio->setPacket((uint8_t*)&mDefaultPacket.top(), MESH_PACKET_OVERHEAD + mDefaultPacket.top().header.length);
  }
  else
  {
    _MESHLOG(mName, "Packet from queue");
    mesh_packet_t* pPacket = mPacketQueue.front();
    mPacketQueue.pop();
    std::string nodestr;
    // clusterJoin
    if (pPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER)
    {
      uint32_t count = min(4, mClusterReqs.size());
      uint32_t first, freeCount = 0;

      for (first = 1; first + freeCount < MESH_MAX_CLUSTER_SIZE; )
      {
        if (mMyCluster->mDevices[first + freeCount] == NULL)
          freeCount++;
        else
        {
          first++;
          freeCount = 0;
        }

        if (freeCount == count)
        {
          break;
        }
      }

      if (first < MESH_MAX_CLUSTER_SIZE)
      {
        pPacket->payload.str.payload.join_cluster.first_slot = first;
        uint8_t send_count = 0;
        for (uint8_t i = 0; i < count; ++i)
        {
          MeshNeighbor* pNb = mClusterReqs.front();
          if (!mMyCluster->contains(pNb))
          {
            mClusterReqs.pop();
            pPacket->payload.str.payload.join_cluster.node[i].set(pNb->mAdvAddr);
            send_count++;
            mMyCluster->mDevices[i + first] = pNb;
            nodestr += "\n\t" + pNb->mDev->mName;
          }
          else if (i == 0) // can permit sending a single device message upon duplicate requests
          {
            for (uint8_t j = 0; j < MESH_MAX_CLUSTER_SIZE; ++j)
            {
              if (mMyCluster->mDevices[j] == pNb)
              {
                mClusterReqs.pop();
                pPacket->payload.str.payload.join_cluster.node[i].set(pNb->mAdvAddr);
                send_count++;
                mMyCluster->mDevices[i + first] = pNb;
                nodestr += "\n\t" + pNb->mDev->mName;
                i = count; // break outer loop
                break;
              }
            }
          }
        }
        if (send_count == 0)
        {
          pPacket = &mDefaultPacket.top();
        }
        else
        {
          _MESHLOG(mName, "CH sending join packet to %s", nodestr.c_str());
        }
      }
    }


    mRadio->setPacket((uint8_t*)pPacket, MESH_PACKET_OVERHEAD + pPacket->header.length);
  }

  switch (mState)
  {
    case CM_STATE_RECON:
    case CM_STATE_MAKE_CLUSTER:
    case CM_STATE_REQ_CLUSTER:
      mBeaconTimer = mTimer->orderRelative((MESH_INTERVAL - mLastStateChange % MESH_INTERVAL) + mRand.Float() * (MESH_INTERVAL - 1 * MS) + 1 * MS, [this](timestamp_t, void*) { radioBeaconTX(); });
      //deliberate fallthrough
    case CM_STATE_CH_SCAN:
      mRadio->shortDisable();
      mRadio->disable();
      mRadio->transmit();
      mRadio->shortToRx();
      break;
    default:
      mRadio->disable();
      mRadio->setChannel(MESH_CHANNEL);
      mRadio->shortDisable();
      mRadio->transmit();
      break;
  }
  
}

void ClusterMeshDev::radioSubRX(MeshNeighbor* pNb, bool noDrift)
{
  switch (mState)
  {
    case CM_STATE_RECON:
    case CM_STATE_MAKE_CLUSTER:
    case CM_STATE_CH_SCAN:
      // skip, this will be covered by ongoing scan
      break;
    case CM_STATE_CH: // subscription to node outside of own cluster
    case CM_STATE_LEAF:
    case CM_STATE_REQ_CLUSTER:
      mRadio->disable();
      mRadio->setChannel(MESH_CHANNEL);
      mRadio->shortDisable();
      mRadio->receive();
      break;
  }

  mCurrentSubAbortTimer = mTimer->orderRelative(MESH_RX_RU_TIME + MESH_ADDRESS_OFFSET_US + (!noDrift) * ((mTimer->getTimestamp() - pNb->mLastBeaconTime) * MESH_MAX_CLOCK_DRIFT_TWO_SIDED * 2), 
    [this](timestamp_t, void*){ if (!mRadio->rxInProgress()) mRadio->disable(); });

  // filter on adv addr
  mRadio->setFilter([=](RadioPacket* pPacket)-> bool{ return ((mesh_packet_t*)pPacket->getContents())->adv_addr == pNb->mAdvAddr; });
}

void ClusterMeshDev::orderRestOfCluster(MeshCluster* pCluster, uint32_t anchorIndex, timestamp_t anchorTimestamp)
{
  // order own beacon if it's the correct cluster
  if (pCluster == mMyCluster && mMyClusterOffset > anchorIndex)
  {
    mTimer->orderAt(anchorTimestamp + (mMyClusterOffset - anchorIndex) * MESH_CLUSTER_SLOT_US - MESH_TX_RU_TIME, [=](timestamp_t, void*){radioBeaconTX(); });
  }
  // order all subscriptions in the cluster
  MeshNeighbor* pNextNb = NULL;
  for (uint32_t i = anchorIndex + 1; i < MESH_MAX_CLUSTER_SIZE; ++i)
  {
    pNextNb = pCluster->mDevices[i];
    if (pNextNb != NULL)
    {
      if (isSubscribedTo(pNextNb))
      {
        mTimer->orderAt(anchorTimestamp + (i - anchorIndex) * MESH_CLUSTER_SLOT_US - MESH_RX_RU_TIME, [=](timestamp_t, void*){radioSubRX(pNextNb, true); });
      }
    }
  }
}

void ClusterMeshDev::setState(cluster_mesh_state_t state) 
{ 
  std::string stateText = "";
  switch (state)
  {
    case CM_STATE_CH:
      stateText = "CH";
      break;
    case CM_STATE_CH_SCAN:
      stateText = "CH scan";
      break;
    case CM_STATE_LEAF:
      stateText = "Leaf";
      break;
    case CM_STATE_MAKE_CLUSTER:
      stateText = "Make Cluster";
      break;
    case CM_STATE_RECON:
      stateText = "Recon";
      break;
    case CM_STATE_REQ_CLUSTER:
      stateText = "Request cluster";
      break;
    default:
      stateText = "Unknown state";
  }
  _MESHLOG(mName, "STATE: %s", stateText.c_str());
  mLastStateChange = mTimer->getTimestamp(); 
  mState = state; 
}

void ClusterMeshDev::transmitClusterjoin(void)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_JOIN_CLUSTER;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_JOIN_CLUSTER;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_JOIN_CLUSTER;
  for (uint8_t i = 0; i < 4; ++i)
    pPacket->payload.str.payload.join_cluster.node[i].clear();
  pPacket->adv_addr.set(mAdvAddr);

  mPacketQueue.push(pPacket);
}

void ClusterMeshDev::transmitClusterReq(void)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_CLUSTER_REQ;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_CLUSTER_REQ;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_CLUSTER_REQ;
  pPacket->payload.str.payload.cluster_req.clusterAddr.set(mClusterHead->mAdvAddr);
  pPacket->adv_addr.set(mAdvAddr);

  mDefaultPacket.push(*pPacket);
  delete pPacket;
}