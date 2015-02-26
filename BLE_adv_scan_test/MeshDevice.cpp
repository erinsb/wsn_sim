#include "MeshDevice.h"
#include "RadioPacket.h"

#define SEARCHLIGHT_SLOT (10000)

MeshDevice::MeshDevice(uint8_t* defaultData, uint32_t defaultLength, double x, double y) : Device(x, y)
{
}


MeshDevice::~MeshDevice()
{
}



void MeshDevice::startSearch(uint32_t slotCount)
{
  mSearchLightSlots = slotCount;
  mCurrentSlotIndex = 1;
  mSearchLightStartTime = getEnvironment()->getTimestamp();
}

void MeshDevice::stopSearch()
{

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

void MeshDevice::searchLightBeacon(bool doubleSlot)
{
  
}
