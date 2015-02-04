#include "Device.h"
#include "Radio.h"
#include <math.h>

Device::Device(double x, double y)
{
  pos.x = x;
  pos.y = y;
  mRadio = new Radio(this, 
    RADIO_DEFAULT_SIGSTRENGTH, 
    RADIO_DEFAULT_TURNAROUND, 
    RADIO_DEFAULT_TIFS, 
    RADIO_DEFAULT_BITRATE);
  mTimer = new Timer(1.0, *this);
}

Device::Device() : Device(0.0, 0.0)
{
}


Device::~Device()
{
  delete mRadio;
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


void Device::radioCallbackTx(RadioPacket* packet)
{

}

void Device::radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted)
{

}
void Device::timerEnded(Timeout* timeout)
{

}