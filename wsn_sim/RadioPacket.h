#pragma once
#include "Device.h"
#include "WSN.h"
#include <stdint.h>
#include <iostream>
#include <iomanip>

#define RADIO_SIGNAL_DECAY (1.0)

class Radio;

class RadioPacket
{
public:
  RadioPacket(const Radio* sender, uint8_t* data, uint32_t length);
  RadioPacket(const RadioPacket& packet);
  ~RadioPacket();

  double getMaxDistance(void) { return mSignalStrength * RADIO_SIGNAL_DECAY; }

  bool collidesWith(RadioPacket* pOther);

  const uint8_t* getContents(void) const { return mData; }
  const uint32_t getLength(void) const { return mLength; }

  const Radio* getSender(void) { return mSender; }

  friend std::ostream& operator<<(std::ostream& ostream, RadioPacket& packet);
  

  uint32_t mStartTime;
  uint32_t mEndTime;
private:
  SimEnv* p_mEnvironment;
  uint8_t mSignalStrength;
  const Radio* mSender;
  uint8_t* mData;
  uint32_t mLength;
};

