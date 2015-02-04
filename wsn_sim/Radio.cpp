#include "Radio.h"
#include "Logger.h"

#include <ostream>

#define LOG_RX Logger("[RX]: ", RADIO_LOG_ENABLED)

Radio::Radio(Device* device, uint8_t sigStrength, uint32_t turnaroundTime_us, uint32_t tifs_us, uint32_t bitrate) : 
  mDevice(device), 
  mSigStrength(sigStrength), 
  mTurnaroundTime_us(turnaroundTime_us), 
  mBitrate(bitrate),
  mState(RADIO_STATE_IDLE),
  mShort(SHORT_DISABLED),
  mNextStateTime(0)
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
    mState = RADIO_STATE_RAMPUP_TX;
    mNextStateTime = getEnvironment()->getTimestamp() + mTurnaroundTime_us;
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
    mState = RADIO_STATE_RAMPUP_RX;
    mNextStateTime = getEnvironment()->getTimestamp() + mTurnaroundTime_us;
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
  mState = RADIO_STATE_IDLE;
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
  mDevice->radioCallbackRx(pPacket, rx_strength, corrupted);
  mWSN->removeReceiver(this);
  shortToNextState();

  LOG_RX << pPacket;
}

void Radio::step(uint32_t timestamp)
{
  if (timestamp >= mNextStateTime)
  {
    switch (mState)
    {
    case RADIO_STATE_IDLE:
      // wait for external tx or rx call
      mNextStateTime = UINT32_MAX;
      break;

    case RADIO_STATE_RAMPUP_RX:
      mWSN->addReceiver(this);
      mNextStateTime = UINT32_MAX;
      mState = RADIO_STATE_RX;
      break;

    case RADIO_STATE_RAMPUP_TX:
      mTxPacketHandle = mWSN->startTransmit(mCurrentPacket);
      mNextStateTime = timestamp + getTxTime(mCurrentPacket.getLength());
      mState = RADIO_STATE_TX;
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
}

void Radio::shortToNextState(void)
{
  if (mShort == SHORT_DISABLED)
  {
    mState = RADIO_STATE_IDLE;
    mNextStateTime = UINT32_MAX;
  }
  else
  {
    mNextStateTime = getEnvironment()->getTimestamp() + mTifs_us;

    if (mShort == SHORT_TO_RX)
    {
      mState = RADIO_STATE_RAMPUP_RX;
    }
    else
    {
      mState = RADIO_STATE_RAMPUP_TX;
    }
  }
}
