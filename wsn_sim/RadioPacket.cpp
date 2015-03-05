#include "RadioPacket.h"
#include "Radio.h"

#include <iostream>
#include <sstream>


RadioPacket::RadioPacket(Radio* const sender, uint8_t* data, uint32_t length) :
  mSender(sender), 
  mLength(length), 
  p_mEnvironment(sender->getEnvironment())
{
  mData = new uint8_t[length];
  memcpy_s(mData, length, data, length);
  mStartTime = p_mEnvironment->getTimestamp();  
  mEndTime = UINT32_MAX; //yet to be determined
  mSignalStrength = sender->getSignalStrength();
}

RadioPacket::RadioPacket(const RadioPacket& packet) : RadioPacket(packet.mSender, packet.mData, packet.mLength){}

RadioPacket::RadioPacket(void) : 
  mSender(NULL), 
  mLength(0), 
  p_mEnvironment(NULL), 
  mData(NULL), 
  mStartTime(0), 
  mEndTime(UINT32_MAX),
  mSignalStrength(0)
{}

RadioPacket::~RadioPacket()
{
  delete mData;
}


bool RadioPacket::collidesWith(RadioPacket* pOther) const
{
  return (
      (pOther != NULL) &&
      (this != pOther) &&
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

RadioPacket& RadioPacket::operator=(RadioPacket& rhs)
{
  delete this->mData;
  
  this->mLength = rhs.mLength;
  this->mStartTime = rhs.mStartTime;
  this->mEndTime = rhs.mEndTime;
  this->mSender = rhs.mSender;
  this->mSignalStrength = rhs.mSignalStrength;
  this->p_mEnvironment = rhs.p_mEnvironment;
  
  this->mData = new uint8_t[rhs.mLength];
  memcpy_s(this->mData, rhs.mLength, rhs.mData, rhs.mLength);

  return (*this);
}

std::string RadioPacket::ToString(void)
{
  std::stringstream os;
  os << (*this);
  return os.str();
}