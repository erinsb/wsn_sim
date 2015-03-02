#include "Radio.h"
#include "Logger.h"

#include <ostream>

#define LOG_RX Logger("[RX]: ", RADIO_LOG_ENABLED)

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
  mShort(SHORT_DISABLED)
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
    getEnvironment()->registerExecution(this, getEnvironment()->getTimestamp() + mTurnaroundTime_us);
  }
  else
  {
    LOG_ERROR << "Radio TX ordered outside IDLE state (state: " << mState << ")";
  }
}

void Radio::receive(void)
{
  if (mState == RADIO_STATE_IDLE)
  {
    setState(RADIO_STATE_RAMPUP_RX);
    getEnvironment()->registerExecution(this, getEnvironment()->getTimestamp() + mTurnaroundTime_us);
  }
  else
  {
    LOG_ERROR << "Radio RX ordered outside IDLE state (state: " << mState << ")";
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

  //LOG_RX << pPacket;
}

void Radio::step(uint32_t timestamp)
{
  switch (mState)
  {
  case RADIO_STATE_IDLE:
    // wait for external tx or rx call
    break;

  case RADIO_STATE_RAMPUP_RX:
    mWSN->addReceiver(this);
    setState(RADIO_STATE_RX);
    break;

  case RADIO_STATE_RAMPUP_TX:
    mTxPacketHandle = mWSN->startTransmit(mCurrentPacket);
    getEnvironment()->registerExecution(this, timestamp + getTxTime(mCurrentPacket.getLength()));
    setState(RADIO_STATE_TX);
    break;

  case RADIO_STATE_RX:
    // is this possible?
    mWSN->removeReceiver(this);
    shortToNextState();
    mDevice->radioCallbackRx(NULL, 0, true);
    break;

  case RADIO_STATE_TX:
    mWSN->endTransmit(mTxPacketHandle);
    shortToNextState();
    mDevice->radioCallbackTx(&mCurrentPacket);
    break;

  default:
    LOG_ERROR << "Radio: illegal state";
  }
}

void Radio::shortToNextState(void)
{
  if (mShort == SHORT_DISABLED)
  {
    setState(RADIO_STATE_IDLE);
  }
  else
  {
    getEnvironment()->registerExecution(this, getEnvironment()->getTimestamp() + mTifs_us);

    if (mShort == SHORT_TO_RX)
    {
      setState(RADIO_STATE_RAMPUP_RX);
    }
    else
    {
      setState(RADIO_STATE_RAMPUP_TX);
    }
  }
}


void Radio::setState(state_t newState)
{
  mDevice->removePowerDrain(powerProfile[mState]);  
  mDevice->registerPowerDrain(powerProfile[newState]);
  mState = newState;
}
