#include "Device.h"
#include <math.h>

Device::Device() : mRadio(*this, RADIO_DEFAULT_SIGSTRENGTH, RADIO_DEFAULT_TURNAROUND, RADIO_DEFAULT_TIFS, RADIO_DEFAULT_BITRATE)
{

}


Device::~Device()
{
}

double Device::getDistanceTo(Device& device) const
{
  double distX = device.pos.x - this->pos.x;
  double distY = device.pos.y - this->pos.y;
  return (sqrt(distX * distX + distY * distY));
}

void Device::step(uint32_t timestamp)
{

}