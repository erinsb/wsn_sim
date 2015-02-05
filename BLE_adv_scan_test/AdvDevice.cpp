#include "AdvDevice.h"
#include "Timer.h"
#include "Radio.h"

#include "Logger.h"

#define ADV_INT (100000)

typedef void(*timer_action)(AdvDevice*);

static void stop_radio(AdvDevice* dev)
{
  dev->getRadio()->disable();
  dev->getRadio()->shortDisable();
}


AdvDevice::AdvDevice(uint8_t* advPacket, uint32_t advPacketLength, uint8_t* scanPacket, uint32_t scanPacketLength):
  mAdvPacket(advPacket),
  mAdvPacketLength(advPacketLength),
  mScanPacket(scanPacket),
  mScanPacketLength(scanPacketLength)
{
}


AdvDevice::~AdvDevice()
{
}

void AdvDevice::start(void)
{
  mTimer->orderRelative(10);
}



void AdvDevice::step(uint32_t timestamp)
{

}

void AdvDevice::radioCallbackTx(RadioPacket* packet)
{
  mTimer->orderRelative(60, (void*) &stop_radio);
  mRadio->shortToTx();

}

void AdvDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (packet != NULL)
  {
    mRadio->setPacket(mScanPacket, mScanPacketLength);
    mRadio->shortDisable();
  }
}

void AdvDevice::timerEnded(Timeout* timeout)
{
  if (timeout->mContext == NULL)
  {
    LOG_DEBUG << "ADV";
    mTimer->orderRelative(100000);

    mRadio->setPacket(mAdvPacket, mAdvPacketLength);
    mRadio->shortToRx();
    mRadio->transmit();
  }
  else
  {
    ((timer_action)timeout->mContext)(this); // fuck, this is ugly
  }
}