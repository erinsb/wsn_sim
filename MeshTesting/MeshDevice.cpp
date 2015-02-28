#include "MeshDevice.h"
#include "BlePacket.h"
#include "Radio.h"
#include "RadioPacket.h"

#define SEARCHLIGHT_SLOT (10000)

MeshDevice::MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x, double y) : Device(x, y), mSearching(false)
{
}


MeshDevice::~MeshDevice()
{
}



void MeshDevice::startSearch(void)
{
  if (mSearching)
    return;
  mSearching = true;
  if (mRadio->getState() == Radio::RADIO_STATE_IDLE)
  {
    mRadio->receive();
  }
}

void MeshDevice::stopSearch(void)
{
  mSearching = false;
}

void MeshDevice::registerNeighbor(MeshDevice* device)
{

}

void MeshDevice::transmit(uint8_t* data, uint32_t length)
{
  mPacketQueue.push(new RadioPacket(mRadio, data, length));
  
}

void MeshDevice::setDefaultPacket(uint8_t* data, uint32_t length)
{

}


void MeshDevice::radioCallbackTx(RadioPacket* packet)
{

}

void MeshDevice::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{
  if (corrupted)
    return;

  ble_adv_packet_t blePacket;
  memcpy(&blePacket, packet->getContents(), packet->getLength());

  blePacket.
}