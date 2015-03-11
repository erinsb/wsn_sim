#pragma once
#include "WSN.h"
#include "MeshDevice.h"


class MeshDevice;
// Singleton
class MeshWSN : public WSN
{
public:
  MeshWSN(void);
  ~MeshWSN(void);

  void logTransmit(void);
  void logReceive(void);
  void logCorruption(void);
  void logClusterHead(MeshDevice* pCH);
  void print(void);

private:
  uint64_t mTXs;
  uint64_t mRXs;
  uint64_t mCorrupted;
  uint64_t mCHs;
};

