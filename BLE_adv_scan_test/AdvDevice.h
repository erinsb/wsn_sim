#pragma once
#include "Device.h"
#include "BlePacket.h"

class AdvDevice : public Device
{
public:
  AdvDevice(ble_adv_packet_t* advPacket, ble_adv_packet_t* scanPacket, uint32_t advInt = 100000);
  ~AdvDevice();

  void start(void);

  virtual void step(uint32_t timestamp);

private:
  ble_adv_packet_t* mAdvPacket;
  ble_adv_packet_t* mScanPacket;

  uint32_t mAdvInt;

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);

  void startAdv(uint32_t timestamp, void* context);
  void stopRadio(uint32_t timestamp, void* context);
};

