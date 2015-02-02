#include "RadioPacket.h"


RadioPacket::RadioPacket(Radio* sender, uint8_t* data, uint32_t length) : mSender(sender), mLength(length), p_mEnvironment(sender.getEnvironment())
{
  mData = new uint8_t[length];
  memcpy_s(mData, length, data, length);
  mStartTime = p_mEnvironment->getTimestamp();
  mSignalStrength = sender->getSignalStrength();
}

RadioPacket::RadioPacket(const RadioPacket& packet) : RadioPacket(packet.mSender, packet.mData, packet.mLength){}


RadioPacket::~RadioPacket()
{
  delete mData;
}


bool RadioPacket::collidesWith(RadioPacket* pOther)
{
  if (this->mStartTime > pOther->mStartTime && pOther->mEndTime > this->mStartTime) //TODO

}