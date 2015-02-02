#include "RadioPacket.h"
#include "Radio.h"


RadioPacket::RadioPacket(const Radio* sender, uint8_t* data, uint32_t length) : mSender(sender), mLength(length), p_mEnvironment(sender->getEnvironment())
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
  return (
      (this != pOther) && 
    (
      (this->mStartTime >= pOther->mStartTime && this->mStartTime >= pOther->mEndTime) ||
      (this->mStartTime >= pOther->mStartTime && this->mStartTime <= pOther->mEndTime) ||
      (this->mStartTime <= pOther->mStartTime && this->mStartTime >= pOther->mEndTime) ||
      (this->mStartTime <= pOther->mStartTime && this->mStartTime <= pOther->mEndTime) 
    ) &&
    !(
      (this->mStartTime > pOther->mEndTime) ||
      (pOther->mStartTime > this->mEndTime)
    )
    );
}


std::ostream& operator<<(std::ostream& ostr, RadioPacket& packet)
{
  ostr.setf(std::ios::hex, std::ios::basefield);
  ostr.setf(std::ios::showbase);

  for (uint32_t i = 0; i < packet.mLength; ++i)
  {
    ostr << packet.mData[i] << " ";
  }

  ostr << std::resetiosflags(std::ios::basefield) << std::resetiosflags(std::ios::showbase);

  return ostr;
}