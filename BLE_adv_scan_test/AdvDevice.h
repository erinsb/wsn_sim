#pragma once
#include "Device.h"

class AdvDevice : public Device
{
public:
  AdvDevice(uint8_t* advPacket, uint32_t advPacketLength, uint8_t* scanPacket, uint32_t scanPacketLength);
  ~AdvDevice();

  void start(void);

  virtual void step(uint32_t timestamp);

private:
  uint8_t* mAdvPacket;
  uint32_t mAdvPacketLength;
  uint8_t* mScanPacket;
  uint32_t mScanPacketLength;


  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);
  virtual void timerEnded(Timeout* timeout);
};

