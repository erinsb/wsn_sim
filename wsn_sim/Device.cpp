#include "Device.h"
#include "Radio.h"
#include <math.h>

uint32_t Device::smDeviceCount = 0;

Device::Device(double x, double y) : mName("Device"), mExtraPowerUsagePart(0.0), mBackgroundPowerUsage(0.0)
{
  pos.x = x;
  pos.y = y;
  mRadio = new Radio(this, 
    RADIO_DEFAULT_SIGSTRENGTH, 
    RADIO_DEFAULT_TURNAROUND, 
    RADIO_DEFAULT_TIFS, 
    RADIO_DEFAULT_BITRATE);
  mTimer = new Timer(1.0);
  mRand.Reseed(MESH_DEV_RAND_SEED + smDeviceCount++);
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


void Device::registerPowerDrain(double power_mA, timestamp_t time)
{
  // avoid power registration when dealing with extremely long simulations
  if (getEnvironment()->getTotalSimTime() > 10 * HOURS)
    return;

  if (time == TIME_MAX)
  {
    time = getEnvironment()->getTimestamp();
  }
  powerMut.lock();

  auto eventBeforeThis = powerUsage.rbegin();
  while (eventBeforeThis != powerUsage.rend() && eventBeforeThis->timestamp > time)
    eventBeforeThis++;

  if (powerUsage.size() == 0)
  {
    powerUsage.push_back(powerEvent_t{ power_mA, time });
  }
  else if (powerUsage.back().timestamp < time)
  {
    powerUsage.push_back(powerEvent_t{ powerUsage.back().power_mA + power_mA, time });
  }
  else if (powerUsage.back().timestamp == time)
  {
    powerUsage.back().power_mA += power_mA;
  }
  else if (eventBeforeThis->timestamp == time)
  {
    eventBeforeThis->power_mA += power_mA;
  }
  else 
  {
    powerUsage.insert(eventBeforeThis.base() + 1, powerEvent_t{ eventBeforeThis->power_mA + power_mA, time });
  }
  powerMut.unlock();
}

void Device::removePowerDrain(double power_mA)
{
  registerPowerDrain(-power_mA);
}

std::vector<double> Device::getPowerUsage(timestamp_t firstSample, timestamp_t lastSample) const
{
  std::vector<double> resultVector;
  timestamp_t time = firstSample;
  timestamp_t now = getEnvironment()->getTimestamp();
  
  if (lastSample > now)
    lastSample = now;

  if (powerUsage.size() == 0)
  {
    for (; time <= lastSample; ++time)
      resultVector.push_back(mBackgroundPowerUsage);
  }

  double usageNow = 0.0;
  
  for (powerEvent_t evt : powerUsage)
  {
    for (; time < evt.timestamp && time <= lastSample; ++time)
      resultVector.push_back(usageNow + mBackgroundPowerUsage);
    usageNow = evt.power_mA;
  }
  for (; time <= lastSample; ++time)
    resultVector.push_back(usageNow + mBackgroundPowerUsage);

  return resultVector;
}

double Device::getPowerUsageAvg(timestamp_t firstSample, timestamp_t lastSample, double paukert) const
{ 
  double sum = 0.0;
  double current = 0.0;
  timestamp_t lastChange = firstSample;
  for (auto it = powerUsage.begin(); it != powerUsage.end(); it++)
  {
    if (it->timestamp > lastSample)
    {
      sum += (lastSample - lastChange) * current;
      break;
    }
    if (it->timestamp > firstSample)
    {
      sum += (it->timestamp - lastChange) * current;
      lastChange = it->timestamp;
    }
    current = pow(it->power_mA, paukert);
  }

  if (powerUsage.back().timestamp < lastSample)
    lastSample = powerUsage.back().timestamp;

  return (1.0 + mExtraPowerUsagePart) * sum / (double(lastSample - firstSample)) + mBackgroundPowerUsage; 
}

void Device::plotPower(timestamp_t startTime, timestamp_t endTime)
{
  PowerPlotter().displayGraph(startTime, endTime, this);
}

void Device::step(timestamp_t timestamp)
{

}