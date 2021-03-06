#include "TreeMeshDev.h"
#include "Logger.h"

#define MESH_CHANNEL  (37)

bool isScanningState(cluster_mesh_state_t state)
{
  switch (state)
  {
    case CM_STATE_RECON:
    case CM_STATE_MAKE_CLUSTER:
    case CM_STATE_CH_SCAN:
		case CM_STATE_BRANCH_SCAN:
      return true;
    default:
      return false;
  }
}

std::string getStateString(cluster_mesh_state_t state)
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
		case CM_STATE_BRANCH:
			stateText = "Branch";
			break;
		case CM_STATE_BRANCH_SCAN:
			stateText = "Branch scan";
			break;
    case CM_STATE_LEAF:
      stateText = "Leaf";
      break;
    case CM_STATE_LEAF_SCAN:
      stateText = "Leaf scan";
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
  return stateText;
}


ClusterMeshDev::ClusterMeshDev(std::string name, double x, double y)
{
  mBackgroundPowerUsage = 0.007; // 7uA all the time
  mExtraPowerUsagePart = 0.2; // 20% more power than assumed
  mScanTriggered = false;
  mScanTimer = TIMER_INVALID;
  mBeaconTimer = TIMER_INVALID;
  mChTimer = TIMER_INVALID;
  mCurrentSubAbortTimer = TIMER_INVALID;
  double driftFactor = ((MESH_MAX_CLOCK_DRIFT)* (2.0 * mRand.Float() - 1.0)) + 1.0; // 1.0 +- <250/1mill 
  mTimer->setDriftFactor(driftFactor);
  mName = name;
  pos.x = x;
  pos.y = y;
  mInterval = MESH_INTERVAL;

  mMyCluster = NULL;
  mMyClusterOffset = 0;
	mMySubCluster = NULL;
	transmitSubTree = false; 

  for (uint8_t i = 0; i < BLE_ADV_ADDR_LEN; ++i)
  {
    mAdvAddr.arr[i] = mRand() & 0xFF;
  }
  mDefaultPacket.push(mesh_packet_t());
  mDefaultPacket.top().payload.str.adv_type = MESH_ADV_TYPE_DEFAULT;
  mDefaultPacket.top().access_addr = MESH_ACCESS_ADDR;
  mDefaultPacket.top().adv_addr.set(mAdvAddr);
  mDefaultPacket.top().payload.str.adv_len = MESH_PACKET_OVERHEAD_DEFAULT;
  mDefaultPacket.top().payload.str.msgID = 0;
  mDefaultPacket.top().payload.str.payload.default.nodeWeight = mScore;
  mDefaultPacket.top().payload.str.payload.default.clusterAddr.clear();
  mDefaultPacket.top().payload.str.payload.default.ch_fields = {0};

  mDefaultPacket.top().header.length = MESH_PACKET_OVERHEAD_DEFAULT;
}


ClusterMeshDev::~ClusterMeshDev()
{
}

void ClusterMeshDev::start(void)
{
  doRecon();
  mTimer->orderRelative(MESH_INTERVAL * 5, [this](timestamp_t, void*){ doMakeCluster(); }); // don't cleanup recon, we want to continue searching
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
		case CM_STATE_BRANCH_SCAN:
		case CM_STATE_CH_SCAN:
      mRadio->shortDisable();
      if (mRadio->getState() == Radio::RADIO_STATE_RX)
      {
        mRadio->disable();
      }
      break;
		case CM_STATE_BRANCH:
		case CM_STATE_CH:
      mTimer->abort(mBeaconTimer);
      _MESHLOG(mName, "Aborted beacon timer without restarting?");
      break;
    case CM_STATE_LEAF:
      //mTimer->abort(mChTimer);
      break;
  }
}

void ClusterMeshDev::doRecon(void)
{
  if (mRadio->getState() != Radio::RADIO_STATE_IDLE)
  {
    mRadio->shortDisable();
	mRadio->disable();
  }
  
  mDefaultPacket.top().payload.str.payload.default.clusterAddr.clear();
  mDefaultPacket.top().payload.str.payload.default.offsetFromCH = 0;

  mRadio->setChannel(MESH_CHANNEL);
  mRadio->shortToRx(); // rx forever
  mRadio->receive();

  mTimer->abort(mBeaconTimer);

  timestamp_t start = mTimer->getTimestamp() + mRand.Float() * (MESH_INTERVAL - 1 * MS) + 1 * MS;
	mBeaconTimer = mTimer->orderPeriodic(start, MESH_INTERVAL, [=](timestamp_t, void*) {transmitSubTree = false; radioBeaconTX(); });;
  setState(CM_STATE_RECON);
}

void ClusterMeshDev::doMakeCluster(void)
{
  setState(CM_STATE_MAKE_CLUSTER);

  if (mRadio->getState() != Radio::RADIO_STATE_IDLE)
  {
    mRadio->shortDisable();
    mRadio->disable();
  }

  mRadio->setChannel(MESH_CHANNEL);
  mRadio->shortToRx(); // rx forever
  mRadio->receive();

  makeClusterCheck();
}

void ClusterMeshDev::setCH(MeshNeighbor* pCH)
{
  setState(CM_STATE_REQ_CLUSTER);
  mRadio->shortDisable();
  mRadio->disable();
  mClusterHead = pCH;
	MeshCluster * pCluster = NULL;
	if(pCH->isClusterHead())
		pCluster = getCluster(&pCH->mAdvAddr);
	else
		pCluster = getCluster(&pCH->mClusterHead);
  //mTimer->abort(mBeaconTimer);

  if (pCluster == NULL)
  {
    pCluster = new MeshCluster(&pCH->mAdvAddr);
    mClusters.push_back(pCluster);
    pCluster->mDevices[0] = pCH;
  }

  setCluster(pCluster);

  if (!isSubscribedTo(pCH))
  {
    //pCH->mLastBeaconTime = mTimer->getTimestamp();
    subscribe(pCH);
  }

  transmitClusterReq();
}

void ClusterMeshDev::becomeCH(void)
{
	_MESHLOG(mName, "Become CH");
	timestamp_t timeNow = mTimer->getTimestamp();
  //mClusterHead = NULL;
  
  timestamp_t start = timeNow + MESH_CLUSTER_SLOT_US + mRand(MESH_INTERVAL);// getNonCollidingOffset(timeNow + MESH_CLUSTER_SLOT_US, timeNow + MESH_INTERVAL);

  mTimer->abort(mBeaconTimer);
	mBeaconTimer = mTimer->orderPeriodic(start, MESH_INTERVAL, [=](timestamp_t, void*) {transmitSubTree = true; radioBeaconTX(); });
	if (mClusterHead == NULL) {
		setState(CM_STATE_CH_SCAN); 
		mTimer->orderRelative(MESH_INTERVAL * 4, [=](timestamp_t, void*) { setState(CM_STATE_CH); }); // Stop scanning for participants after a while
	}
	else {
		setState(CM_STATE_BRANCH_SCAN);
		mTimer->orderRelative(MESH_INTERVAL * 4, [=](timestamp_t, void*) { setState(CM_STATE_BRANCH); }); // Stop scanning for participants after a while
	}
  mRadio->shortToRx(); // scan "forever"
  if (mRadio->getState() != Radio::RADIO_STATE_RX)
  {
    mRadio->disable();
  }
	if (!mTimer->isValidTimer(mChTimer)) {
		mTimer->abort(mChTimer); //abort switch back to leaf
	}

  startLeafScanTimer();

	//if (mClusterHead == NULL) {
		mMySubCluster = new MeshCluster(&mAdvAddr);
		mClusters.push_back(mMySubCluster);
		if(mClusterHead == NULL) mMyClusterOffset = 0; //Only for root
	//}
  mDevTag = DEVICE_TAG_MEDIUM;
  // order cluster time orientation with regular offsets
  mTimer->orderPeriodic(MESH_INTERVAL * (10 + mRand(10)), MESH_INTERVAL * 20, 
    [this](timestamp_t, void*)
  {
    setupTimeOrientation();
  });
}

void ClusterMeshDev::subscribe(MeshNeighbor* pNb)
{
  timestamp_t nextBeacon = pNb->getNextBeaconTime(mTimer->getTimestamp());

  mSubscriptions.push_back(pNb);
  pNb->mFollowing = true;
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
	

  // for topology map
  bool isCHconnection = 
    (mClusterHead == pNb || 
      (
        ((ClusterMeshDev*)pNb->mDev)->mClusterHead != NULL && 
        ((ClusterMeshDev*)pNb->mDev)->mClusterHead->mDev == this
      )
    );

  mWSN->addConnection(this, pNb->mDev, isCHconnection);
}

void ClusterMeshDev::subscriptionAbort(MeshNeighbor* pNb)
{
  if (mState != CM_STATE_CH && mState != CM_STATE_CH_SCAN) 
    resetLeafScanInterval();

	_MESHWARN(mName, "Aborted sub to %s. STATE: %s", pNb->mDev->mName.c_str(), getStateString(mState).c_str());
  mWSN->removeConnection(this, pNb->mDev);
  mTimer->abort(pNb->mRxTimer);
  
  if (pNb == mClusterHead /*&& !isCH()*/) //no longer necessarily true in tree
  {
    mClusterHead = NULL;
    std::vector<MeshNeighbor*> tempNbs;
    for (MeshNeighbor* pNb : mNeighbors)
    {
      if (pNb->mFollowing)
      {
        tempNbs.push_back(pNb);
      }
      else
      {
        for (MeshCluster* pCluster : mClusters)
        {
          uint32_t index = pCluster->getIndexOf(pNb);
          if (index < MESH_MAX_CLUSTER_SIZE)
          {
            pCluster->mDevices[index] = NULL;
          } 
        }
        delete pNb;
      }
    }
    mNeighbors = tempNbs;
    if (mState != CM_STATE_RECON && mState != CM_STATE_MAKE_CLUSTER)
    {
      doRecon();
      mTimer->orderRelative(MESH_INTERVAL * 2, [this](timestamp_t, void*){ doMakeCluster(); });
    }
  }
  if (mState == CM_STATE_REQ_CLUSTER && mTimer->getTimestamp() - mLastStateChange > MESH_INTERVAL * 50)
  {
    becomeCH();
  }
  for (auto it = mSubscriptions.begin(); it != mSubscriptions.end(); it++)
  {
    if (*it == pNb)
    {
      mSubscriptions.erase(it);
      break;
    }
  }
}

void ClusterMeshDev::makeClusterCheck(void)
{
	MeshNeighbor* pBestNb = getBestNb([=](MeshNeighbor* pNb) -> bool { return (pNb->mClusterHead.isNull()); }); // unbound devices
  MeshNeighbor* pBestCH = getBestNb([](MeshNeighbor* pNb) -> bool{ return (pNb->isClusterHead() || !pNb->mClusterHead.isNull()); });

  if (mClusterHead == NULL && pBestCH != NULL)
  {
		_MESHLOG(mName, "Set CH: %s",pBestCH->mDev->mName.c_str());
		setCH(pBestCH);
    return;
  }

  if (pBestNb == NULL || pBestNb->mNodeWeight <= mScore)
  {
    if (pBestNb == NULL)
      _MESHLOG(mName, "No available neighbors");
    else
      _MESHLOG(mName, "Best NB: %d, me: %d", pBestNb->mNodeWeight, mScore);
    becomeCH();
  }
  else if (mTimer->getTimestamp() - mLastStateChange > MESH_INTERVAL * 20)
    becomeCH();

  //else keep waiting
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
  for (auto it = mClusters.begin(); it != mClusters.end(); it++)
  {
    if ((*it)->mCHaddr == *pCHaddr)
      return (*it);
  }
  return NULL;
}

// Time synchronization
timestamp_t ClusterMeshDev::getClusterTime(void)
{
  if (isCH() && mTimer->isValidTimer(mBeaconTimer))
  {
    return mTimer->getExpiration(mBeaconTimer) + MESH_TX_RU_TIME;
  }
  else if (mClusterHead != NULL)
  {
    return mTimer->getExpiration(mClusterHead->mRxTimer) + MESH_RX_RU_TIME;
  }
  return 0;
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

MeshCluster* ClusterMeshDev::getLastClusterBefore(void)
{
  MeshCluster* pNearestCluster = NULL;
  for (MeshCluster* pCluster : mClusters)
  {
    if (pCluster != mMyCluster)
    {
      timestamp_t tempOffset = pCluster->getOffset(getClusterTime());
      timestamp_t bestOffset = pNearestCluster == NULL? 0 : pNearestCluster->getOffset(getClusterTime());
      if (pNearestCluster == NULL || tempOffset > bestOffset)
      {
        pNearestCluster = pCluster;
      }
    }
  }
  return pNearestCluster;
}


MeshCluster* ClusterMeshDev::getNearestCluster(timestamp_t time)
{
  MeshCluster* pNearestCluster = NULL;
  for (MeshCluster* pCluster : mClusters)
  {
    if (pCluster != mMyCluster &&
      (pNearestCluster == NULL || pCluster->absoluteOffset(time) < pNearestCluster->absoluteOffset(time)))
    {
      pNearestCluster = pCluster;
    }
  }

  return pNearestCluster;
}

MeshCluster* ClusterMeshDev::getNextClusterAfter(timestamp_t startTime)
{
  MeshCluster* pBestCluster = NULL;
  for (MeshCluster* pCluster : mClusters)
  {
    if (pBestCluster == NULL || pCluster->getOffset(startTime) < pBestCluster->getOffset(startTime))
    {
      pBestCluster = pCluster;
    }
  }
  return pBestCluster;
}

void ClusterMeshDev::radioCallbackTx(RadioPacket* pPacket)
{
  timestamp_t txTime = mTimer->getTimerTime(pPacket->mStartTime);
  if (isCH())
  {
    mesh_packet_t* pMeshPacket = (mesh_packet_t*)pPacket->getContents();

    /*if (pMeshPacket->indicatesClusterScan())
    {
      scanCluster(mMySubCluster, 0, txTime);
    }
    else
    {*/
      orderRestOfCluster(mMySubCluster, 0, txTime); //same as scanCluster mostly?
    //}
    // do as promised in packet
    if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_DEFAULT)
    {
      if (pMeshPacket->payload.str.payload.default.ch_fields.rxAtEnd)
      {
        mTimer->orderAt(txTime + (pMeshPacket->payload.str.payload.default.ch_fields.clusterMax) * MESH_CLUSTER_SLOT_US - MESH_RX_RU_TIME,
          [this](timestamp_t, void*)
        {
          mRadio->disable();
          //mRadio->shortDisable();
          mRadio->receive();
          mCurrentSubAbortTimer = mTimer->orderRelative(MESH_RX_RU_TIME + MESH_ADDRESS_OFFSET_US,
            [=](timestamp_t, void*)
          {
            if (!mRadio->rxInProgress())
            {
              mRadio->disable();
            }
          }
          );
        }
        );
      }
      if (pMeshPacket->payload.str.payload.default.ch_fields.isScanning)
      {
        mRadio->shortToRx();
      }
    }
    else if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_NUDGE_CLUSTER)
    {
      nudgeCluster(pMeshPacket->payload.str.payload.nudge_cluster.offset_us);
    }

  }
}

void ClusterMeshDev::radioCallbackRx(RadioPacket* pPacket, uint8_t rx_strength, bool corrupted)
{
	mRadio->removeFilter();
	mTimer->abort(mCurrentSubAbortTimer);
	((MeshWSN*)mWSN)->logReceive();

	if (corrupted)
	{
		((MeshWSN*)mWSN)->logCorruption();
		mJustFinishedLeafScan = false;
		return;
	}
	mesh_packet_t* pMeshPacket = (mesh_packet_t*)pPacket->getContents();
	MeshNeighbor* pSender = getNb(&pMeshPacket->adv_addr);
	timestamp_t rxTime = mTimer->getTimerTime(pPacket->mStartTime);
	/*if (pSender != NULL && pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER) {
		_MESHLOG(mName, "Received join cluster packet from %s", pSender->mDev->mName.c_str());
	}*/
	//else _MESHLOG(mName, "Received packet from unknown");
	// register newfound neighbor

	if (pSender == NULL)
	{
		pSender = new MeshNeighbor(&pMeshPacket->adv_addr);
		pSender->mDev = pPacket->getSender()->getDevice();
		// add pSender to mNeighbors
		mNeighbors.push_back(pSender);
	}

	// update neighbor structure & score
	pSender->receivedBeacon(rxTime, pMeshPacket, pPacket->mChannel, rx_strength);
	mScore = mNeighbors.size();
	//mDefaultPacket.top().payload.str.payload.default.nodeWeight = mScore;


	if (isSubscribedTo(pSender))
	{
		if (mTimer->isValidTimer(pSender->mRxTimer)) // may have been ordered into cluster train
		{
			mTimer->reschedule(pSender->mRxTimer, rxTime + (MESH_INTERVAL - MESH_RX_RU_TIME) * (1.0 - MESH_MAX_CLOCK_DRIFT_TWO_SIDED));
		}
	}

	// Get cluster of pSender
	MeshCluster* pCluster = NULL;
	// sender is part of a cluster
	if (!pSender->mClusterHead.isNull() || pSender == mClusterHead)
	{
		if (pSender->mBeaconOffset >= MESH_MAX_CLUSTER_SIZE)
			_MESHERROR(mName, "Beacon offset out of range for %s. Offset: %d", pSender->mDev->mName, pSender->mBeaconOffset);

		bool clusterIsNew = false;

		/* if (pSender == mClusterHead)
		 {
			 pCluster = mMyCluster;
		 }
		 else
		 {*/
		pCluster = getCluster(&pSender->mClusterHead);
		//}
		if (mMyCluster == NULL && pSender == mClusterHead && !isCH())
		{
			setCluster(pCluster);
			if (pCluster == NULL)
				_MESHERROR(mName, "Can't find my own cluster");
		}
		else if (pCluster == NULL)
		{
			_MESHLOG(mName, "New cluster!");
			pCluster = new MeshCluster(&pSender->mClusterHead);
			mClusters.push_back(pCluster);
			pCluster->mDevices[pSender->mBeaconOffset] = pSender; // may have to update this other places when introducing cluster reorganization
			clusterIsNew = true;
		}

		if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_DEFAULT)
		{
			pCluster->mClusterMax = pMeshPacket->payload.str.payload.default.ch_fields.clusterMax;
		}
		// cluster train collision avoidance
		if (mMyCluster != NULL && pCluster != mMyCluster)
		{
			// store cluster train offset from our own
			timestamp_t clusterTrainAnchor = rxTime - MESH_CLUSTER_SLOT_US * pSender->mBeaconOffset;

			timestamp_t oldOffset = pCluster->mLastCHBeacon;
			pCluster->mLastCHBeacon = clusterTrainAnchor;

			if (clusterTrainAnchor > oldOffset + MESH_INTERVAL / 2)
			{
				if (clusterIsNew)
				{
					pCluster->mInitialOffset = pCluster->getOffset(getClusterTime());
					pCluster->mApproachSpeed = 0.0;
				}
				else
					pCluster->mApproachSpeed = pCluster->getOffset(oldOffset);

				if (pCluster->mApproachSpeed > MESH_INTERVAL / 2)
					pCluster->mApproachSpeed = pCluster->mApproachSpeed - MESH_INTERVAL;

				// want approach speed, rather than offset speed:
				pCluster->mApproachSpeed *= -1;
			}
		}
		// setup any chained events
		//if (pCluster->getFirstDeviceIndex() >= pSender->mBeaconOffset &&
		//  (pCluster != mMyCluster || pSender->mBeaconOffset < mMyClusterOffset)) // must be first beacon in cluster (including own)
		if (pCluster->mCHaddr == pSender->mAdvAddr) //head of its cluster
		{
			if (pCluster == mMyCluster)
				orderRestOfCluster(mMyCluster, 0, rxTime);
			else orderRestOfCluster(pCluster, pSender->mBeaconOffset, rxTime);
		}

		// create connection to cluster if it doesn't already exist
		if (!pSender->mFollowing && mMyCluster != pCluster && mMySubCluster != pCluster && !isSubscribedToCluster(pCluster))
		{
			_MESHLOG(mName, "Found device %s, subscribing. STATE: %s", pSender->mDev->mName.c_str(), getStateString(mState).c_str());
			subscribe(pSender);
			//setCH(pSender);
		}
	}

	// special behavior
	// accounts for problems built into simulator
	switch (mState)
	{
	case CM_STATE_MAKE_CLUSTER:
		makeClusterCheck();
		break;
	case CM_STATE_REQ_CLUSTER:
		if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER)
		{
			// sender is clusterhead
			if (pCluster == NULL)
			{
				pCluster = getCluster(&mClusterHead->mAdvAddr);

				if (pCluster == NULL)
				{
					pCluster = new MeshCluster(&pSender->mAdvAddr);
					mClusters.push_back(pCluster);
					pCluster->mDevices[0] = pSender; // may have to update this other places when introducing cluster reorganization
				}
			}
			for (uint8_t i = 0; i < 4; ++i)
			{
				uint32_t clusterOffset = pMeshPacket->payload.str.payload.join_cluster.first_slot + i;
				if (pMeshPacket->payload.str.payload.join_cluster.node[i].isNull())
					break;

				if (clusterOffset >= MESH_MAX_CLUSTER_SIZE)
					_MESHERROR(mName, "Cluster offset out of bounds");

				if (pMeshPacket->payload.str.payload.join_cluster.node[i] == mAdvAddr)
				{
					// join cluster train
					mMyClusterOffset = clusterOffset;
					//mClusterHead = pSender; // in case there was any doubt (naive?)

					pCluster->mDevices[0] = pSender;
					//mTimer->abort(mBeaconTimer); // beacon will now rely on cluster head
					_MESHLOG(mName, "Stopped requesting /beaconing");
					cleanupPrevState();
					mRadio->shortToRx(); // scan "forever"
					mRadio->receive();
					/*if (mRadio->getState() != Radio::RADIO_STATE_RX)
					{
						mRadio->disable();
					}*/
					setState(CM_STATE_BRANCH_SCAN); //scan for more distant leaf nodes
					mChTimer = mTimer->orderRelative(MESH_INTERVAL * 5, [this](timestamp_t, void*) {
						mTimer->abort(mBeaconTimer);
						mRadio->shortDisable();
						mRadio->disable();
						setState(CM_STATE_LEAF);
					});
					mWSN->addConnection(this, pSender->mDev, true);
					mPacketQueue.pop(); // take away request packet

					// setup a scan after some time, when the mesh might have stabilized. Repeat occasionally
					startLeafScanTimer();

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
		else if (pSender == mClusterHead) /* Clusterhead is ignoring us */
		{
			if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_DEFAULT)
			{
				if (!pMeshPacket->payload.str.payload.default.ch_fields.isScanning)
				{
					/* CH has stopped scanning, no point in random TX */
					if ((mTimer->getTimestamp() - mLastStateChange) > MESH_INTERVAL * 3)
						mTimer->abort(mBeaconTimer);

					/* if CH is listening at the end of its cluster train, we may be able to get its attention */
					if (pMeshPacket->payload.str.payload.default.ch_fields.rxAtEnd)
					{
						mBeaconTimer = mTimer->orderAt(
							rxTime + MESH_INTERVAL * pMeshPacket->payload.str.payload.default.ch_fields.clusterMax - MESH_RX_RU_TIME,
							[=](timestamp_t, void*) {transmitSubTree = false; radioBeaconTX(); }
						);
					}
				}
				else
				{
					if (!mTimer->isValidTimer(mBeaconTimer))
						mBeaconTimer = mTimer->orderRelative(mRand.Float() * (MESH_INTERVAL - 1 * MS) + 1 * MS, [this](timestamp_t, void*) {transmitSubTree = false; radioBeaconTX(); });
				}
			}
		}
		break;
	case CM_STATE_BRANCH:
	case CM_STATE_BRANCH_SCAN:
		if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_CLUSTER_REQ) {
			if (pMeshPacket->payload.str.payload.cluster_req.clusterAddr == mAdvAddr) {
				mTimer->abort(mChTimer); //abort switch back to LEAF
				_MESHLOG(mName, "***BRANCH BECOMING CH*** Req from %s", pSender->mDev->mName.c_str());
				becomeCH();
				//mClusterReqs.push(pSender);
				//transmitClusterjoin();
			}
		}
	case CM_STATE_LEAF:
	case CM_STATE_LEAF_SCAN:
		if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_DEFAULT)
		{
			if (pMeshPacket->payload.str.payload.default.ch_fields.headCount)
			{
				MeshCluster* pNearestCluster = getLastClusterBefore();
				if (pNearestCluster != NULL)
				{
					transmitNearestClusterUpdate(pNearestCluster);
				}
			}
		}
		else if (pMeshPacket->payload.str.adv_type == MESH_ADV_TYPE_NUDGE_CLUSTER)
		{
			timestamp_t offset = pMeshPacket->payload.str.payload.nudge_cluster.offset_us;
			if (pSender == mClusterHead)
			{
				nudgeCluster(offset);
				transmitClusterNudge(offset, MESH_CHANNEL);
			}
			else if (pCluster != NULL && pCluster != mMyCluster)
			{
				MeshNeighbor* pFirstInCluster = pCluster->mDevices[pCluster->getFirstDeviceIndex()];
				if (mTimer->isValidTimer(pFirstInCluster->mRxTimer))
					mTimer->reschedule(pFirstInCluster->mRxTimer, mTimer->getExpiration(pFirstInCluster->mRxTimer) + offset);
			}
		}
		if (mState != CM_STATE_BRANCH && mState != CM_STATE_BRANCH_SCAN) break;

		case CM_STATE_CH:
    case CM_STATE_CH_SCAN:
      switch (pMeshPacket->payload.str.adv_type)
      {
        case MESH_ADV_TYPE_CLUSTER_REQ:
          if (pMeshPacket->payload.str.payload.cluster_req.clusterAddr == mAdvAddr)
          {
						bool newReq = true;
						MeshNeighbor* temp;
						for (int i = 0; i < mClusterReqs.size(); i++) {
							temp = mClusterReqs.front();
							mClusterReqs.pop();
							if (temp == pSender) newReq = false;
							mClusterReqs.push(temp);
						}
						if (newReq) {
							mClusterReqs.push(pSender);
							transmitClusterjoin();
						}
						else {
							_MESHLOG(mName, "Redundant request received from %s", pSender->mDev->mName.c_str());
						}
          }
          break;
        case MESH_ADV_TYPE_CLOSEST_NEIGHBOR:
          if (pCluster == mMyCluster)
          {
             MeshCluster* pNbCluster = getCluster(&pMeshPacket->payload.str.payload.closest_neighbor.neighbor_ch);
             if (pNbCluster == NULL)
             {
               pNbCluster = new MeshCluster(&pMeshPacket->payload.str.payload.closest_neighbor.neighbor_ch);
               mClusters.push_back(pNbCluster);
             }
             pNbCluster->mApproachSpeed = pMeshPacket->payload.str.payload.closest_neighbor.approach_speed;
             pNbCluster->mLastCHBeacon = getClusterTime() + pMeshPacket->payload.str.payload.closest_neighbor.offset_us;
          }
          break;
        case MESH_ADV_TYPE_NUDGE_CLUSTER:
        {
          timestamp_t offset = pMeshPacket->payload.str.payload.nudge_cluster.offset_us;
          if (pCluster != NULL && pCluster != mMyCluster)
          {
            if (pCluster->mDevices[pCluster->getFirstDeviceIndex()] == pSender)
            {
              if (mTimer->isValidTimer(pSender->mRxTimer))
                mTimer->reschedule(pSender->mRxTimer, mTimer->getExpiration(pSender->mRxTimer) + offset);
            }
            else
            {
              _MESHWARN(mName, "Attempt to nudge external cluster failed");
            }
          }
        }
        default:
          break;
      }
      break;
    default: 
      break;
  }


  mJustFinishedLeafScan = false;
}

void ClusterMeshDev::processPacket(mesh_packet_t* pMeshPacket)
{
  
}

void ClusterMeshDev::radioBeaconTX(void)
{
	
	
	bool transmitBeacon;
	mesh_packet_t* pPacket = NULL;
	if (!transmitSubTree) {
		transmitBeacon = mPacketQueue.empty();
		if (!transmitBeacon) {
			pPacket = mPacketQueue.front();
			mPacketQueue.pop();
		}
	}
	else {
		transmitBeacon = mSubPacketQueue.empty();
		if (!transmitBeacon) {
			//_MESHLOG(mName, "Sending special packet to sub tree");
			pPacket = mSubPacketQueue.front();
			mSubPacketQueue.pop();
		}
	}

	//_MESHLOG(mName, "Sending packet in state %s", getStateString(mState).c_str());
  if (!transmitBeacon)
  { 
    std::string nodestr;
		//if (pPacket->payload.str.adv_type == MESH_ADV_TYPE_CLUSTER_REQ && mClusterHead!=NULL)
			//_MESHLOG(mName, "Sent cluster req to %s", mClusterHead->mDev->mName.c_str());
		// clusterJoin
    if (pPacket->payload.str.adv_type == MESH_ADV_TYPE_JOIN_CLUSTER)
    {
      uint32_t count = min(4, mClusterReqs.size());
      uint32_t first, freeCount = 0;

      for (first = 1; first + freeCount < MESH_MAX_CLUSTER_SIZE; )
      {
        if (mMySubCluster->mDevices[first + freeCount] == NULL)
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

      if (first < MESH_MAX_CLUSTER_SIZE - 4)
      {
        pPacket->payload.str.payload.join_cluster.first_slot = first;
        uint8_t send_count = 0;
        for (uint8_t i = 0; i < count; ++i)
        {
          if (mClusterReqs.empty())
            break;
          MeshNeighbor* pNb = mClusterReqs.front();
          if (!mMySubCluster->contains(pNb))
          {
            mClusterReqs.pop();
            pPacket->payload.str.payload.join_cluster.node[i].set(pNb->mAdvAddr);
            send_count++;
            mMySubCluster->mDevices[i + first] = pNb;
            nodestr += "\n\t" + pNb->mDev->mName;
          }
          else if (i == 0) // can permit sending a single device message upon duplicate requests
          {
            for (uint8_t j = 0; j < MESH_MAX_CLUSTER_SIZE; ++j)
            {
              if (mMySubCluster->mDevices[j] == pNb)
              {
                mClusterReqs.pop();
                pPacket->payload.str.payload.join_cluster.first_slot = j;
                pPacket->payload.str.payload.join_cluster.node[i].set(pNb->mAdvAddr);
                send_count++;
                nodestr += "\n\t" + pNb->mDev->mName;
                i = count; // break outer loop
                break;
              }
            }
          }
        }
        if (send_count == 0)
        {
          transmitBeacon = true; 
        }
        else
        {
          _MESHLOG(mName, "CH sending join packet to %s", nodestr.c_str());
        }
      }
			if(count) for (uint8_t i = 0; i < count - 1; i++) mSubPacketQueue.pop(); //remove extra join packets
    }


    mRadio->setPacket((uint8_t*)pPacket, MESH_PACKET_OVERHEAD + pPacket->header.length);
    delete pPacket; // radio takes a copy
  }

  if (mScanTriggered)
  {
    if (isCH())
    {
			if (mState == CM_STATE_CH)
				setState(CM_STATE_CH_SCAN);
			else if (mState == CM_STATE_BRANCH)
				setState(CM_STATE_BRANCH_SCAN);
			else if(mState == CM_STATE_CH_SCAN){
        setState(CM_STATE_CH);
        mScanTriggered = false;
			}
			else if (mState == CM_STATE_BRANCH_SCAN) {
				setState(CM_STATE_BRANCH);
				mScanTriggered = false;
			}
    }
    else
    {
      if (mState != CM_STATE_LEAF_SCAN)
        setState(CM_STATE_LEAF_SCAN);
      else
      {
        setState(CM_STATE_LEAF);
        mScanTriggered = false;
      }
    }
  }

  if (transmitBeacon)
  {
    if (isCH())
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.set(mAdvAddr);
    else if (mClusterHead != NULL)
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.set(mClusterHead->mAdvAddr);
    else
      mDefaultPacket.top().payload.str.payload.default.clusterAddr.clear();

    mDefaultPacket.top().adv_addr.set(mAdvAddr);
    mDefaultPacket.top().payload.str.payload.default.ch_fields = { 0 };
    mDefaultPacket.top().payload.str.payload.default.ch_fields.isScanning = isScanningState(mState);
    mDefaultPacket.top().payload.str.payload.default.ch_fields.clusterMax = ((mMyCluster != NULL)? mMyCluster->getLastDeviceIndex() : 0);
    mDefaultPacket.top().payload.str.payload.default.nodeWeight = mScore;
    mDefaultPacket.top().payload.str.payload.default.offsetFromCH = mMyClusterOffset;
		mDefaultPacket.top().payload.str.payload.default.ch_fields.hasSubTree = mMySubCluster != NULL;
    mDefaultPacket.top().header.length = MESH_PACKET_OVERHEAD_DEFAULT;
    mRadio->setPacket((uint8_t*)&mDefaultPacket.top(), MESH_PACKET_OVERHEAD + mDefaultPacket.top().header.length);
  }

  mRadio->shortDisable();
  mRadio->disable();
  mRadio->transmit();

  switch (mState)
  {
		case CM_STATE_REQ_CLUSTER:
			transmitClusterReq();
		case CM_STATE_RECON:
    case CM_STATE_MAKE_CLUSTER:
      if (!mTimer->isValidTimer(mBeaconTimer))
				mBeaconTimer = mTimer->orderRelative((MESH_INTERVAL - mLastStateChange % MESH_INTERVAL) + mRand.Float() * (MESH_INTERVAL - 1 * MS) + 1 * MS, [this](timestamp_t, void*) { transmitSubTree = false; radioBeaconTX(); });
      //deliberate fallthrough
    case CM_STATE_CH_SCAN:
		case CM_STATE_BRANCH_SCAN:
    case CM_STATE_LEAF_SCAN:
      mRadio->shortToRx();
      break;
    default:
      mRadio->setChannel(MESH_CHANNEL);
      mRadio->shortDisable();
      break;
  }

  ((MeshWSN*)mWSN)->logTransmit();
}

void ClusterMeshDev::radioSubRX(MeshNeighbor* volatile pNb, bool noDrift)
{
  if (mState == CM_STATE_LEAF_SCAN && pNb == mClusterHead)
  {
    mJustFinishedLeafScan = true;
    mScanTriggered = false;
    setState(CM_STATE_LEAF);
  }
	if (mState == CM_STATE_BRANCH_SCAN && pNb == mClusterHead)
	{
		mJustFinishedLeafScan = true;
		mScanTriggered = false;
		setState(CM_STATE_BRANCH);
	}

  switch (mState)
  {
    case CM_STATE_LEAF_SCAN:
		case CM_STATE_BRANCH_SCAN:
    case CM_STATE_RECON:
    case CM_STATE_MAKE_CLUSTER:
    case CM_STATE_CH_SCAN:
      mRadio->shortToRx();
      // skip, this will be covered by ongoing scan
      break;
		case CM_STATE_BRANCH:
		case CM_STATE_CH: // subscription to node outside of own cluster
    case CM_STATE_LEAF:   
		case CM_STATE_REQ_CLUSTER:
      mRadio->shortDisable();
      mRadio->disable();
      mRadio->setChannel(MESH_CHANNEL);
      mRadio->receive();
      break;
  }

  mCurrentSubAbortTimer = mTimer->orderRelative(MESH_RX_RU_TIME + MESH_ADDRESS_OFFSET_US + MESH_RX_SAFETY_MARGIN + (!noDrift) * ((mTimer->getTimestamp() - pNb->mLastBeaconTime) * MESH_MAX_CLOCK_DRIFT_TWO_SIDED * 2),
    [=](timestamp_t timestamp, void*)
  {
    if (!mRadio->rxInProgress())
    {
      mRadio->disable();
      mRadio->removeFilter();
      if (isCH() && mClusterScanInProgress && mMySubCluster->contains(pNb))
      {
        mMySubCluster->mDevices[mMySubCluster->getIndexOf(pNb)] = NULL;
      }
      if (pNb->mFollowing && pNb->getLostPacketCount(timestamp) > MESH_MAX_LOSS_COUNT)
      {
        subscriptionAbort(pNb);
        //TODO: some neighbors must be erased..
      }
    }
  }
  );

  // filter on adv addr
	if (pNb != NULL);
    mRadio->setFilter([=](RadioPacket* pPacket)-> bool{ return ((mesh_packet_t*)pPacket->getContents())->adv_addr == pNb->mAdvAddr; });
}

void ClusterMeshDev::orderRestOfCluster(MeshCluster* pCluster, uint32_t anchorIndex, timestamp_t anchorTimestamp)
{
  // order own beacon if it's the correct cluster
  if (pCluster == mMyCluster && mMyClusterOffset > anchorIndex && pCluster->mCHaddr != mAdvAddr)
  {
		mTimer->orderAt(anchorTimestamp + (mMyClusterOffset - anchorIndex) * MESH_CLUSTER_SLOT_US - MESH_TX_RU_TIME, [=](timestamp_t, void*) {transmitSubTree = false;  radioBeaconTX(); });
  }
  else if (mJustFinishedLeafScan)
  {
    _MESHLOG(mName, 
      "Failed to reboot beaconing after leaf scan\n\tis my cluster: %s\n\tclusterOffset: %d, anchor offset: %d", 
      (pCluster == mMyCluster)? "yes" : "no", mMyClusterOffset, anchorIndex);
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

void ClusterMeshDev::scanCluster(MeshCluster* pCluster, uint32_t anchorIndex, timestamp_t anchorTimestamp)
{
  // order own beacon if it's the correct cluster
  if (pCluster == mMyCluster && mMyClusterOffset > anchorIndex)
  {
    mTimer->orderAt(anchorTimestamp + (mMyClusterOffset - anchorIndex) * MESH_CLUSTER_SLOT_US - MESH_TX_RU_TIME, 
			[=](timestamp_t, void*) {transmitSubTree = false;  radioBeaconTX(); });
  }
  // order all subscriptions in the cluster
  for (uint32_t i = anchorIndex + 1; i < MESH_MAX_CLUSTER_SIZE; ++i)
  {
    MeshNeighbor* pNextNb = pCluster->mDevices[i];
    if (pNextNb != NULL)
    {
      mTimer->orderAt(anchorTimestamp + (i - anchorIndex) * MESH_CLUSTER_SLOT_US - MESH_RX_RU_TIME, 
        [=](timestamp_t, void*){radioSubRX(pNextNb, true); });
    }
  }
}

/* change our speed to match the cluster train before ours */
void ClusterMeshDev::adjustPacing(void)
{
  if (!isCH())
    _MESHERROR(mName, "Non-CH attempted to adjust pacing");

  MeshCluster* pNearestCluster = getLastClusterBefore();
  bool doNudge = false;

  if (pNearestCluster == NULL)
    return;
#if 1
  timestamp_t nearestOffset = MESH_INTERVAL - pNearestCluster->getOffset(getClusterTime());

  if (nearestOffset < 30 * MS)
  {
    bool preadjust = false;
    // DANGER ZONE
    int64_t movespeed = pNearestCluster->mApproachSpeed;// +(20 * MS - (int64_t)nearestOffset) / (5);
    if (nearestOffset < 20 * MS)
    {
      movespeed = int64_t(20 * MS - nearestOffset) / (50);
    }
    else
    {
      movespeed = -int64_t(nearestOffset - 20 * MS) / (50);
    }

    if (int64_t(mInterval) + movespeed > MESH_INTERVAL * (1.0 + MESH_MAX_CLOCK_DRIFT_TWO_SIDED) ||
      int64_t(mInterval) + movespeed < MESH_INTERVAL * (1.0 - MESH_MAX_CLOCK_DRIFT))
    {
      if (movespeed > 0)
        mInterval = MESH_INTERVAL * (1.0 + MESH_MAX_CLOCK_DRIFT_TWO_SIDED);
      else if (movespeed < 0)
        mInterval = MESH_INTERVAL * (1.0 - MESH_MAX_CLOCK_DRIFT);
      movespeed = 0;
      preadjust = true;
    }

    if (movespeed != 0 || preadjust)
    {
      // impact of adjustment depends on distance to next 
      mInterval += (movespeed);
      if (mTimer->isValidTimer(mBeaconTimer))
        mTimer->changeInterval(mBeaconTimer, mInterval);
      else
        _MESHWARN(mName, "Adjusting non-valid beacon timer");
      pNearestCluster->mApproachSpeed -= movespeed;
    }
    else if (pNearestCluster->mApproachSpeed > 0 && nearestOffset < 10*MS)
    {
      // dodge the approaching train
      doNudge = true;
      _MESHLOG(mName, "Dodging distance: %llu", nearestOffset);
    }
    else if (false && mInterval != MESH_INTERVAL)
    {
      //mInterval += (mInterval > MESH_INTERVAL) ? -1 : 1;
      mTimer->changeInterval(mBeaconTimer, mInterval);
    }
  }
  else
  {
    // slowly recover from previous adjustments. Will cause "bouncing" effect in trains.
    if (false && mInterval != MESH_INTERVAL)
    {
      mInterval += (mInterval > MESH_INTERVAL)? -2 : 2;
      mTimer->changeInterval(mBeaconTimer, mInterval);
    }
  }
#else
  int64_t offset = pNearestCluster->getOffset(getClusterTime() + pNearestCluster->mInitialOffset);
  if (offset > MESH_INTERVAL / 2)
    offset -= (int64_t)MESH_INTERVAL;
  int64_t movespeed = offset / 5;
  if (pNearestCluster->getOffset(getClusterTime()) > MESH_INTERVAL - 40 *MS && movespeed != 0)
  {
    if (abs((int64_t)mInterval + movespeed - (int64_t)MESH_INTERVAL) > MESH_INTERVAL * MESH_MAX_CLOCK_DRIFT)
    {
      if (movespeed > 0)
      {
        mInterval = MESH_INTERVAL * (1.0 + MESH_MAX_CLOCK_DRIFT);
        movespeed = 0;
      }
      else if (movespeed < 0)
      {
        mInterval = MESH_INTERVAL * (1.0 - MESH_MAX_CLOCK_DRIFT);
        movespeed = 0;
      }
    }

    mInterval += movespeed;
    mTimer->changeInterval(mBeaconTimer, mInterval);
  }

#endif

  if (doNudge)
  {
    timestamp_t offset = getNonCollidingOffset(40 * MS, MESH_INTERVAL - 30 * MS); 
    transmitClusterNudge(MESH_INTERVAL - 30 * MS - mRand(30 * MS), MESH_CHANNEL);
  }


}
void ClusterMeshDev:: setCluster(MeshCluster* pCluster)
{
  if (pCluster == NULL || mClusterHead == NULL /*|| mClusterHead->mAdvAddr != pCluster->mCHaddr*/)
  {
    _MESHERROR(mName, "Attempting to set cluster illegally");
  }
  mMyCluster = pCluster;
}

void ClusterMeshDev::nudgeCluster(timestamp_t offset)
{
  resetLeafScanInterval();
  timer_t timer = 0;
  if (isCH())
  {
    timer = mBeaconTimer;
    _MESHWARN(mName, "NUDGING CLUSTER BY %llu", offset);
    mInterval = MESH_INTERVAL;
    mTimer->changeInterval(mBeaconTimer, mInterval);
    setupTimeOrientation();
  }
  else if (mClusterHead != NULL)
  {
    timer = mClusterHead->mRxTimer;
  }
  else
  {
    _MESHERROR(mName, "Nudging independent node");
  }

  for (MeshCluster* pCluster : mClusters)
  {
    pCluster->mApproachSpeed = 0;
  }

  
  mTimer->reschedule(timer, mTimer->getExpiration(timer) + offset);
}

void ClusterMeshDev::setupTimeOrientation(void)
{
  mesh_packet_t* pPacket = new mesh_packet_t(mDefaultPacket.top());
  pPacket->payload.str.payload.default.ch_fields.headCount = 1;
  pPacket->payload.str.payload.default.ch_fields.rxAtEnd = 1;
  mSubPacketQueue.push(pPacket);

  mTimer->orderRelative(MESH_INTERVAL * 1.5, [=](timestamp_t, void*){ adjustPacing(); });
}

void ClusterMeshDev::startLeafScanTimer(void)
{
  mScanInterval = MESH_LEAF_SCAN_INTERVAL_MIN;
  mScanTimer = mTimer->orderPeriodic(MESH_LEAF_SCAN_WAIT, mScanInterval,
    [this](timestamp_t, void*)
  {
    mScanTriggered = true;
    mScanInterval *= 2;
    if (mScanInterval > MESH_LEAF_SCAN_INTERVAL_MAX)
    {
      mTimer->abort(mScanTimer);
    }
    else if (mTimer->isValidTimer(mScanTimer))
    {
      mTimer->changeInterval(mScanTimer, mScanInterval);
    }
    else
    {
      startLeafScanTimer();
    }

  }
  );
}
void ClusterMeshDev::resetLeafScanInterval(void)
{
  mScanInterval = MESH_LEAF_SCAN_INTERVAL_MIN;
  if (mTimer->isValidTimer(mScanTimer))
    mTimer->changeInterval(mScanTimer, mScanInterval);
  else
    startLeafScanTimer();
}

timestamp_t ClusterMeshDev::getNonCollidingOffset(timestamp_t begin, timestamp_t end)
{
  timestamp_t bestOffset = begin;
  timestamp_t bestGap = 0;
  timestamp_t offset = begin;
  while (offset < end)
  {
    timestamp_t gap = 0;
    MeshCluster* nextCluster = getNextClusterAfter(offset);
    if (nextCluster == NULL)
    {
      gap = MESH_INTERVAL;
    }
    else if (nextCluster->getOffset(getClusterTime()) > offset)
    {
      gap = nextCluster->getOffset(getClusterTime()) - offset;
    }
    else
    {
      gap = 0;
    }

    if (gap > bestGap)
    {
      bestOffset = offset;
      bestGap = gap;
    }
    offset += gap + MESH_CLUSTER_SLOT_US;
  }
  return bestOffset + bestGap / 2;
}

bool ClusterMeshDev::isSubscribedToCluster(MeshCluster* pCluster)
{
  for (auto pSub : mSubscriptions)
  {
    if (pCluster->contains(pSub))
    {
      return true;
    }
  }
  return false;
}

void ClusterMeshDev::setState(cluster_mesh_state_t state) 
{ 
  std::string stateText = getStateString(state);
	std::string mStateText = getStateString(mState);
  /*if ((mState == CM_STATE_CH || mState == CM_STATE_CH_SCAN) && 
    state != CM_STATE_CH && state != CM_STATE_CH_SCAN)
    _MESHLOG(mName, "CH CHANGING STATE TO %s", stateText.c_str());
	if(mState == CM_STATE_BRANCH_SCAN || state == CM_STATE_BRANCH_SCAN)
		_MESHLOG(mName, "Changing from %s TO %s", mStateText.c_str(),stateText.c_str());
	if ((state == CM_STATE_CH || state == CM_STATE_CH_SCAN) && !isCH())
		_MESHWARN(mName, "Changing from %s TO %s", mStateText.c_str(), stateText.c_str());*/
	//_MESHLOG(mName, "Old State: %s    New State: %s", getStateString(mState).c_str(), getStateString(state).c_str());
/*	if (mState == state && mState) 
		_MESHWARN(mName, "What?????");*/
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
	pPacket->payload.str.payload.default.nodeWeight = mScore;
  for (uint8_t i = 0; i < 4; ++i)
    pPacket->payload.str.payload.join_cluster.node[i].clear();
  pPacket->adv_addr.set(mAdvAddr);

  mSubPacketQueue.push(pPacket);
}

void ClusterMeshDev::transmitClusterReq(void)
{
  //mesh_packet_t packet;
	mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_CLUSTER_REQ;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_CLUSTER_REQ;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_CLUSTER_REQ;
	pPacket->payload.str.payload.default.nodeWeight = mScore;
  pPacket->payload.str.payload.cluster_req.clusterAddr.set(mClusterHead->mAdvAddr);
  pPacket->adv_addr.set(mAdvAddr);
	mPacketQueue.push(pPacket);
}
void ClusterMeshDev::transmitNearestClusterUpdate(MeshCluster* pNearestCluster)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_CLOSEST_NB;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_CLOSEST_NB;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_CLOSEST_NEIGHBOR;
  pPacket->payload.str.payload.closest_neighbor.neighbor_ch.set(pNearestCluster->mCHaddr);
  pPacket->payload.str.payload.closest_neighbor.approach_speed = pNearestCluster->mApproachSpeed;
  pPacket->payload.str.payload.closest_neighbor.offset_us = pNearestCluster->getOffset(getClusterTime());
  pPacket->adv_addr.set(mAdvAddr);

  mPacketQueue.push(pPacket);
}

void ClusterMeshDev::transmitClusterNudge(timestamp_t offset, uint8_t new_channel)
{
  mesh_packet_t* pPacket = new mesh_packet_t();

  pPacket->access_addr = MESH_ACCESS_ADDR;
  pPacket->header.length = MESH_PACKET_OVERHEAD + MESH_PACKET_OVERHEAD_NUDGE_CLUSTER;
  pPacket->header.type = BLE_PACKET_TYPE_ADV_IND;
  pPacket->payload.str.adv_len = MESH_PACKET_OVERHEAD_NUDGE_CLUSTER;
  pPacket->payload.str.adv_type = MESH_ADV_TYPE_NUDGE_CLUSTER;
  pPacket->payload.str.payload.nudge_cluster.new_channel = new_channel;
  pPacket->payload.str.payload.nudge_cluster.offset_us = (uint32_t) offset;
  pPacket->adv_addr.set(mAdvAddr);

  // fast lane the packet
  if (!mPacketQueue.empty())
  {
    auto tempQueue = std::queue<mesh_packet_t*>(mPacketQueue);
    while (!mPacketQueue.empty())
      mPacketQueue.pop();
    mPacketQueue.push(pPacket);
    while (!tempQueue.empty())
    {
      mPacketQueue.push(tempQueue.front());
      tempQueue.pop();
    }
  }
  else
  {
    mPacketQueue.push(pPacket);
  }
}
