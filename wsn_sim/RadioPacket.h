#pragma once
#include "Device.h"
#include "WSN.h"
#include <stdint.h>
#include <iostream>
#include <iomanip>

#define RADIO_SIGNAL_DECAY (100.0 / 30.0) // 30 meters

class Radio;

class RadioPacket
{
  friend Radio;
public:
  RadioPacket(Radio* const sender, uint8_t* data, uint32_t length);
  RadioPacket(const RadioPacket& packet);
  RadioPacket(void);
  ~RadioPacket();

  double getMaxDistance(void) { return mSignalStrength / RADIO_SIGNAL_DECAY; }

  bool collidesWith(RadioPacket* pOther) const;

  const uint8_t* getContents(void) const { return mData; }
  const uint32_t getLength(void) const { return mLength; }

  Radio* const getSender(void) const { return mSender; }

  friend std::ostream& operator<<(std::ostream& ostream, RadioPacket& packet);
  RadioPacket& operator=(RadioPacket& rhs);
  std::string ToString(void);

  timestamp_t mStartTime;
  timestamp_t mEndTime;
  uint32_t mChannel;

protected:
  SimEnv* p_mEnvironment;
  uint8_t mSignalStrength;
  Radio* mSender;
  uint8_t* mData;
  uint32_t mLength;
};

