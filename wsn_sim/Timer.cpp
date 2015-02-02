#include "Timer.h"


Timer::Timer(double drift, TimerClient& client) : mDriftFactor(drift), mClient(client), mDriftAnchor(0)
{
}

Timer::~Timer()
{
  mTimeouts.clear();
}



void Timer::resetDrift(void)
{
  mDriftAnchor = getEnvironment()->getTimestamp();
}

void Timer::orderAt(uint32_t timestamp, void* context)
{
  mTimeouts.push_back(Timeout(timestamp, context));
}

void Timer::orderRelative(uint32_t deltaTime, void* context)
{
  orderAt(getTimestamp() + deltaTime, context);

}

uint32_t Timer::getTimestamp(void)
{
  return getTimerTime(getEnvironment()->getTimestamp());
}


void Timer::step(uint32_t timestamp)
{

}

uint32_t Timer::getTimerTime(uint32_t globalTime)
{
  return uint32_t(mDriftFactor * (globalTime - mDriftAnchor) + mDriftAnchor);
}
