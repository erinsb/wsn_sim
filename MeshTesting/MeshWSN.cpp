#include "MeshWSN.h"
#include "ClusterMeshDev.h"


MeshWSN::MeshWSN(void)
{
}


MeshWSN::~MeshWSN()
{
}

void MeshWSN::logTransmit(void)
{
  mTXs++;
}

void MeshWSN::logReceive(void)
{
  mRXs++;
}

void MeshWSN::logCorruption(void)
{
  mCorrupted++;
}

void MeshWSN::logClusterHead(MeshDevice* pCH)
{
  mCHs++;
}

void MeshWSN::print(void)
{
  printf("STATS:-----------------------------------------------------\n");
  printf("Transmits: %d\n", mTXs);
  printf("Avg. RX per TX: %f\n", double(mRXs) / mTXs);
  printf("Corruption rate: %.2f%%\n", 100.0 * double(mCorrupted) / mRXs);
  printf("Cluster heads: %d\n", mCHs);
  printf("Cluster head rate: %f\n", double(mCHs) / getDeviceCount());
  uint32_t loners = 0;
  std::vector<ClusterMeshDev*> clusterHeads;
  for (Device* pDev : mDevices)
  {
    ClusterMeshDev* pMDev = (ClusterMeshDev*)pDev;
    if (pMDev->mNeighbors.size() == 0)
      ++loners;
    if (pMDev->isCH())
      clusterHeads.push_back(pMDev);
  }

  printf("Loners: %d\n", loners);

  struct ch
  {
    ClusterMeshDev* pCH;
    uint32_t conns;
  };
  std::vector<struct ch> clusterHeadSubscribers(clusterHeads.size());
  uint32_t symmetric = 0;

  for (connection_t& conn : mConnections)
  {
    if (conn.symmetric)
      symmetric++;
    ClusterMeshDev* chsInConn[] = { (ClusterMeshDev*)conn.pFirst, (ClusterMeshDev*)conn.pSecond };
    for (uint8_t i = 0; i < 2; ++i)
    {
      if (chsInConn[i]->isCH())
      {
        bool exists = false;
        for (struct ch& chs : clusterHeadSubscribers)
        {
          if (chs.pCH == chsInConn[i])
          {
            chs.conns++;
            exists = true;
            break;
          }
        }
        if (!exists)
          clusterHeadSubscribers.push_back({ chsInConn[i], 1 });
      }
    }
  }

  struct ch *pMaxCh=NULL;
  uint32_t totalCHSubs = 0;
  for (struct ch& chs : clusterHeadSubscribers)
  {
    if (pMaxCh == NULL || chs.conns > pMaxCh->conns)
      pMaxCh = &chs;
    totalCHSubs += chs.conns;
  }

  printf("Symmetric connection rate: %.2f%%\n", 100.0 * double(symmetric) / mConnections.size());
  printf("Max CH subs: %d\n", pMaxCh->conns);
  printf("Useless CHs: %d\n", clusterHeadSubscribers.size() - clusterHeads.size());
  printf("Avg CH subs: %.2f\n", double(totalCHSubs) / clusterHeads.size());

  double totalRadioDutyCycle = 0.0;
  for (Device* pDev : mDevices)
  {
    totalRadioDutyCycle += pDev->getRadio()->getTotalDutyCycle();
  }
  printf("Average radio duty cycle: %.2f%%\n", totalRadioDutyCycle / double(mDevices.size()) * 100.0);


  double totPowerUsage = 0.0;
  double maxPowerUsage = 0.0;
  double minPowerUsage = 100000000.0;
  double peukert = 1.15;

  for (auto it = mDevices.begin(); it != mDevices.end(); it++)
  {
    double usage = (*it)->getPowerUsageAvg(MESH_INTERVAL * 5, getEnvironment()->getTimestamp(), peukert); // don't count the search
    totPowerUsage += usage;
    if (usage > maxPowerUsage)
      maxPowerUsage = usage;
    if (usage < minPowerUsage)
      minPowerUsage = usage;
  }
  
  totPowerUsage *= (getEnvironment()->getTimestamp() - MESH_INTERVAL * 5) / 1000000.0 / HOURS; // nA -> mAh
  maxPowerUsage *= (getEnvironment()->getTimestamp() - MESH_INTERVAL * 5) / 1000000.0 / HOURS; // nA -> mAh
  minPowerUsage *= (getEnvironment()->getTimestamp() - MESH_INTERVAL * 5) / 1000000.0 / HOURS; // nA -> mAh

  printf("Avg power usage: %.5fmAh\n", totPowerUsage / mDevices.size());
  printf("Max power usage: %.5fmAh\n", maxPowerUsage);
  printf("Min power usage: %.5fmAh\n", minPowerUsage);

  timestamp_t firstDeath = pow(240.0 / 1000.0, peukert) / (maxPowerUsage * pow(1263.0, peukert)); // in hours
  timestamp_t lastDeath = pow(240.0 / 1000.0, peukert) / (minPowerUsage * pow(1263.0, peukert)); // in hours

  printf("First dead node: %d years, %d days and %d hours\n", uint32_t(firstDeath / (24ULL * 365ULL)), uint32_t((firstDeath / 24ULL) % 365ULL), uint32_t(firstDeath % 24ULL));
  printf("Last dead node: %d years, %d days and %d hours\n", uint32_t(lastDeath / (24ULL * 365ULL)), uint32_t((lastDeath / 24ULL) % 365ULL), uint32_t(lastDeath % 24ULL));
}