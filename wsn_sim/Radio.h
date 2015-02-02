#pragma once
#include <stdint.h>
#include "Device.h"
#include "Runnable.h"

class Device;

class Radio : public Runnable
{
public:
  typedef enum
  {
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_RAMPUP_RX,
    RADIO_STATE_RAMPUP_TX,
    RADIO_STATE_IDLE
  } state_t;

  Radio(Device* device, uint8_t sigStrength, uint32_t turnaroundTime_us, uint32_t tifs_us, uint32_t bitrate);
  ~Radio();

  void transmit(uint8_t* packet, uint8_t length);
  void receive(uint8_t* packet, uint8_t maxLength);
  void disable(void);

  uint8_t getSignalStrength(void) const { return mSigStrength; }

  double getX(void) { return mDevice->pos.x; }
  double getY(void) { return mDevice->pos.y; }

  Device* getDevice(void) const { return mDevice; }

  virtual void step(uint32_t time);


private:
  state_t mState;
  Device* mDevice;
  uint32_t mTurnaroundTime_us;
  uint32_t mBitrate;
  uint8_t mTifs_us;
  uint8_t mSigStrength;
};

