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
  void setAdvAddress(uint64_t addr);

  void startSearch(void);
  void stopSearch(void);

  void registerNeighbor(MeshDevice* device);

  void transmit(uint8_t* data, uint32_t length);
  void setDefaultPacket(uint8_t* data, uint32_t length);

private:
  std::vector<MeshDevice*> mNeighbors;
  std::queue<RadioPacket*> mPacketQueue;
  RadioPacket* mDefaultPacket = NULL;
  bool mSearching;
  ble_adv_addr_t myAddr;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);
};

  