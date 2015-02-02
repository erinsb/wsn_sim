#pragma once
#include "Device.h"
#include "Radio.h"
#include "WSN.h"
#include <stdint.h>

#define RADIO_SIGNAL_DECAY (1.0)

class RadioPacket
{
public:
  RadioPacket(Radio* sender, uint8_t* data, uint32_t length);
  RadioPacket(const RadioPacket& packet);
  ~RadioPacket();

  double getMaxDistance(void) { return mSignalStrength * RADIO_SIGNAL_DECAY; }

  bool collidesWith(RadioPacket* pOther);

  const Radio* getSender(void) { return mSender; }

private:
  uint32_t mStartTime;
  uint32_t mEndTime;
  SimEnv* p_mEnvironment;
  uint8_t mSignalStrength;
  Radio* mSender;
  uint8_t* mData;
  uint32_t mLength;
};

