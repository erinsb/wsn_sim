#include "AdvDevice.h"
#include "Timer.h"
#include "Radio.h"

#include "Logger.h"

#include <functional>

#define ADV_INT (100000)


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
  startAdv(getEnvironment()->getTimestamp(), NULL);
}

void AdvDevice::stopRadio(uint32_t timestamp, void* context)
{
  mRadio->disable();
  mRadio->shortDisable();
}



void AdvDevice::step(uint32_t timestamp)
{

}

void AdvDevice::radioCallbackTx(RadioPacket* packet)
{
  mTimer->orderRelative(600, std::bind(&AdvDevice::stopRadio, this, std::placeholders::_1, std::placeholders::_2));
  mRadio->shortToTx();
  mRadio->setTifs(150);

}

void AdvDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (packet != NULL)
  {
    mRadio->setPacket(mScanPacket, mScanPacketLength);
    mRadio->shortDisable();
  }
}

void AdvDevice::startAdv(uint32_t timestamp, void* context)
{
  _LOG("ADV %d", timestamp);
  mTimer->orderRelative(ADV_INT, std::bind(&AdvDevice::startAdv, this, std::placeholders::_1, std::placeholders::_2));

  mRadio->setPacket(mAdvPacket, mAdvPacketLength);
  mRadio->shortToRx();
  mRadio->setTifs(148);
  mRadio->transmit();
}