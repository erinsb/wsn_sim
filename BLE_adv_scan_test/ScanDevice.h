#pragma once
#include "Device.h"

class ScanDevice : public Device
{
public:
  ScanDevice();
  ~ScanDevice();

  void start(uint32_t scanInterval, uint32_t scanTime);
  
  virtual void step(uint32_t timestamp);

private:
  uint32_t mScanInterval, mScanTime;
  uint8_t* mScanRsp;
  bool mWaitingForAdv;

  void startScanning();
  void stopScanning();
  void abortRspRx();

  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);
};

