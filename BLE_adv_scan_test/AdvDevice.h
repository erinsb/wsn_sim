#pragma once
#include "Device.h"

class AdvDevice : public Device
{
public:
  AdvDevice(uint8_t* advPacket, uint32_t advPacketLength, uint8_t* scanPacket, uint32_t scanPacketLength, uint32_t advInt = 100000);
  ~AdvDevice();

  void start(void);

  virtual void step(uint32_t timestamp);

private:
  uint8_t* mAdvPacket;
  uint32_t mAdvPacketLength;
  uint8_t* mScanPacket;
  uint32_t mScanPacketLength;

  uint32_t mAdvInt;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);

  void startAdv(uint32_t timestamp, void* context);
  void stopRadio(uint32_t timestamp, void* context);
};

