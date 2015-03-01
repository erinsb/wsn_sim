#include "AdvDevice.h"
#include "Timer.h"
#include "Radio.h"

#include "Logger.h"

#include <functional>



AdvDevice::AdvDevice(ble_adv_packet_t* advPacket, ble_adv_packet_t* scanPacket, uint32_t advInt) :
  mAdvPacket(advPacket),
  mScanPacket(scanPacket),
  mAdvInt(advInt)
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
    mRadio->setPacket((uint8_t*)mScanPacket, mScanPacket->header.length + BLE_PACKET_OVERHEAD_LENGTH);
    mRadio->shortDisable();
  }
}

void AdvDevice::startAdv(uint32_t timestamp, void* context)
{
  _LOG("ADV %d", timestamp);
  mTimer->orderRelative(mAdvInt, std::bind(&AdvDevice::startAdv, this, std::placeholders::_1, std::placeholders::_2));

  mRadio->setPacket((uint8_t*)mAdvPacket, mAdvPacket->header.length + BLE_PACKET_OVERHEAD_LENGTH);
  mRadio->shortToRx();
  mRadio->setTifs(148);
  mRadio->transmit();
}