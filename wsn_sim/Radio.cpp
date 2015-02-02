#include "Radio.h"
#include "Logger.h"

#include <ostream>

#define LOG_RX Logger("[RX]: ", RADIO_LOG_ENABLED)

Radio::Radio(Device* device, uint8_t sigStrength, uint32_t turnaroundTime_us, uint32_t tifs_us, uint32_t bitrate) : 
  mDevice(device), 
  mSigStrength(sigStrength), 
  mTurnaroundTime_us(turnaroundTime_us), 
  mTifs_us(tifs_us),
  mBitrate(bitrate),
  mState(RADIO_STATE_IDLE)
{
  
}


Radio::~Radio()
{
}

void Radio::receivePacket(RadioPacket* pPacket)
{


  LOG_RX << pPacket;
}

void Radio::step(uint32_t timestamp)
{

}