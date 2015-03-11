#include "Radio.h"
#include "Logger.h"

#include <ostream>

#ifdef LOG_ENABLE
#undef LOG_ENABLE
#endif
#define LOG_ENABLE  (0)

#define RADIO_SAME_STATE_SWITCH_TIME  (10)

const double Radio::powerProfile[] =
{
  RADIO_POWER_RX,
  RADIO_POWER_TX,
  RADIO_POWER_RAMPUP_RX,
  RADIO_POWER_RAMPUP_TX,
  RADIO_POWER_IDLE
};

Radio::Radio(Device* device, uint8_t sigStrength, uint32_t turnaroundTime_us, uint32_t tifs_us, uint32_t bitrate) : 
  mDevice(device), 
  mSigStrength(sigStrength), 
  mTurnaroundTime_us(turnaroundTime_us), 
  mBitrate(bitrate),
  mState(RADIO_STATE_IDLE),
  mShort(SHORT_DISABLED), 
  mNextActionTime(100000)
{
  // can't be faster than hardware allows
  mTifs_us = (tifs_us > turnaroundTime_us) ? tifs_us : turnaroundTime_us;
}

Radio::~Radio()
{
}

void Radio::transmit(void)
{
  if (mState == RADIO_STATE_IDLE)
  {
    setState(RADIO_STATE_RAMPUP_TX);
    wait(mTurnaroundTime_us);
    _LOG("%s Radio TX", mDevice->mName.c_str());
  }
  else
  {
    _ERROR("Radio TX ordered outside IDLE state (state: %d)", mState);
  }
}

void Radio::receive(void)
{
  if (mState == RADIO_STATE_IDLE)
  {
    setState(RADIO_STATE_RAMPUP_RX);
    wait(mTurnaroundTime_us);
    _LOG("%s Radio RX", mDevice->mName.c_str());
  }
  else
  {
    _ERROR("Radio TX ordered outside IDLE state (state: %d)", mState);
  }
}

void Radio::disable(void)
{
  if (mState == RADIO_STATE_RX)
  {
    mWSN->removeReceiver(this);
  }
  else if (mState == RADIO_STATE_TX)
  {
    mWSN->abortTransmit(mTxPacketHandle);
  }
  setState(RADIO_STATE_IDLE);
  mNextActionTime = 0;
}

void Radio::setPacket(uint8_t* packet, uint8_t length)
{
  mCurrentPacket = RadioPacket(this, packet, length);
}

void Radio::shortToRx(void)
{
  mShort = SHORT_TO_RX;
}

void Radio::shortToTx(void)
{
  mShort = SHORT_TO_TX;
}

void Radio::shortDisable(void)
{
  mShort = SHORT_DISABLED;
}

void Radio::receivePacket(RadioPacket* pPacket, uint8_t rx_strength, bool corrupted)
{
  mWSN->removeReceiver(this);
  shortToNextState();
  RadioPacket localPacket(*pPacket);
  mDevice->radioCallbackRx(&localPacket, rx_strength, corrupted);
}

void Radio::step(uint32_t timestamp)
{
  if (timestamp != mNextActionTime)
  {
    _WARN("Wrongful radio step");
  }
   

  switch (mState)
  {
  case RADIO_STATE_IDLE:
    // wait for external tx or rx call
    break;

  case RADIO_STATE_RAMPUP_RX:
    mWSN->addReceiver(this);
    setState(RADIO_STATE_RX);
    _LOG("%s RU RX done", mDevice->mName.c_str());
    break;

  case RADIO_STATE_RAMPUP_TX:  
    setState(RADIO_STATE_TX);
    mCurrentPacket.mStartTime = getEnvironment()->getTimestamp();
    mTxPacketHandle = mWSN->startTransmit(mCurrentPacket);
    wait(getTxTime(mCurrentPacket.getLength()));
    _LOG("%s RU TX done", mDevice->mName.c_str());
    break;

  case RADIO_STATE_RX:
    // is this possible?
    //mWSN->removeReceiver(this);
    //shortToNextState();
    //mDevice->radioCallbackRx(NULL, 0, true);
    _LOG("%s RX done", mDevice->mName.c_str());
    break;

  case RADIO_STATE_TX:
    mWSN->endTransmit(mTxPacketHandle);
    shortToNextState();
    mDevice->radioCallbackTx(&mCurrentPacket);
    _LOG("%s TX done", mDevice->mName.c_str());
    break;

  default:
    _ERROR("Radio: illegal state");
  }
}

void Radio::shortToNextState(void)
{
  state_t prevState = mState;

  if (mShort == SHORT_DISABLED)
  {
    setState(RADIO_STATE_IDLE);
  }
  else
  {
    switch (mShort)
    {
    case SHORT_TO_RX:
      setState(RADIO_STATE_RAMPUP_RX);
      break;

    case SHORT_TO_TX:
      setState(RADIO_STATE_RAMPUP_TX);
      break;
    }
    // don't go all the way around when radio just needs to go back to active
    if ((prevState == RADIO_STATE_RX && mState == RADIO_STATE_RAMPUP_RX) ||
      (prevState == RADIO_STATE_TX && mState == RADIO_STATE_RAMPUP_TX))
    {
      wait(RADIO_SAME_STATE_SWITCH_TIME);
    }
    else
    {
      wait(mTifs_us);
    }
  }
}


void Radio::setState(state_t newState)
{
  mDevice->removePowerDrain(powerProfile[mState]);  
  mDevice->registerPowerDrain(powerProfile[newState]);
  mState = newState;
}

void Radio::wait(uint32_t time)
{
  mNextActionTime = getEnvironment()->getTimestamp() + time;
  getEnvironment()->registerExecution(this, mNextActionTime);
}