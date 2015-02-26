#pragma once
#include "Device.h"
#include "RadioPacket.h"
#include <vector>
#include <queue>
#include <stdint.h>

class MeshDevice : public Device
{
public:
  MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x = 0.0, double y = 0.0);
  ~MeshDevice();

  void startSearch(uint32_t slotCount);
  void stopSearch();

  void registerNeighbor(MeshDevice* device);

  void transmit(uint8_t* data, uint32_t length);
  void setDefaultPacket(uint8_t* data, uint32_t length);

private:
  std::vector<MeshDevice*> mNeighbors;
  std::queue<RadioPacket*> mPacketQueue;
  RadioPacket* mDefaultPacket = NULL;
  uint32_t mSearchLightSlots;
  uint32_t mCurrentSlotIndex;
  uint32_t mSearchLightStartTime;

  void searchLightBeacon(bool doubleSlot = false);
};

