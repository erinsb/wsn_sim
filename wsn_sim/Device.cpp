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
  mTimer = new Timer(1.0);
}

Device::Device() : Device(0.0, 0.0)
{
}


Device::~Device()
{
  delete mRadio;
  delete mTimer;
}

double Device::getDistanceTo(Device& device) const
{
  double distX = device.pos.x - this->pos.x;
  double distY = device.pos.y - this->pos.y;
  return (sqrt(distX * distX + distY * distY));
}


void Device::registerPowerDrain(double power_mA)
{
  powerMut.lock();
  if (powerUsage.size() == 0)
  {
    powerUsage.push_back(powerEvent_t{ power_mA, getEnvironment()->getTimestamp() });
  }
  else if (powerUsage.back().timestamp == getEnvironment()->getTimestamp())
  {
    powerUsage.back().power_mA += power_mA;
  }
  else
  {
    powerUsage.push_back(powerEvent_t{ powerUsage.back().power_mA + power_mA, getEnvironment()->getTimestamp() });
  }
  powerMut.unlock();
}

void Device::removePowerDrain(double power_mA)
{
  registerPowerDrain(-power_mA);
}

std::vector<double> Device::getPowerUsage(uint32_t firstSample, uint32_t lastSample) const 
{
  std::vector<double> resultVector;
  uint32_t time = firstSample;
  uint32_t now = getEnvironment()->getTimestamp();
  
  if (lastSample > now)
    lastSample = now;

  if (powerUsage.size() == 0)
  {
    for (; time <= lastSample; ++time)
      resultVector.push_back(0.0);
  }

  double usageNow = 0.0;
  
  for (powerEvent_t evt : powerUsage)
  {
    for (; time < evt.timestamp && time <= lastSample; ++time)
      resultVector.push_back(usageNow);
    usageNow = evt.power_mA;
  }
  for (; time <= lastSample; ++time)
    resultVector.push_back(usageNow);

  return resultVector;
}

double Device::getPowerUsageAvg(uint32_t firstSample, uint32_t lastSample) const
{
  auto usage = getPowerUsage(firstSample, lastSample);
  uint32_t size = usage.size();
  double total = 0.0;
  for (double mA : usage)
    total += mA;
  return total / size;
}

void Device::plotPower(uint32_t startTime, uint32_t endTime)
{
  PowerPlotter().displayGraph(startTime, endTime, this);
}

void Device::step(uint32_t timestamp)
{

}