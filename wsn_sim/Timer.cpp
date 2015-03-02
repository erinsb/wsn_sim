#include "Timer.h"

Timer::Timer(double drift) : mDriftFactor(drift), mDriftAnchor(0)
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

timer_t Timer::orderAt(uint32_t timestamp, const std::function<void(uint32_t, void*)> callback, void* context)
{
  mTimeouts.push_back(new Timeout(timestamp, mNextTimerIndex, callback, context));
  getEnvironment()->registerExecution(this, getGlobalTimeAtLocalTime(timestamp));
  mIteratorInvalidated = true;

  return mNextTimerIndex++;
}

timer_t Timer::orderRelative(uint32_t deltaTime, const std::function<void(uint32_t, void*)> callback, void* context)
{
  return orderAt(getTimestamp() + deltaTime, callback, context);
}

void Timer::reschedule(timer_t timer, uint32_t timestamp)
{
  Timeout* pTo = getTimeoutStruct(timer);

  if (pTo == NULL)
    return;

  pTo->mTimestamp = timestamp;
}

void Timer::abort(timer_t timer)
{
  for (auto it = mTimeouts.begin(); it != mTimeouts.end(); it++)
  {
    if ((*it)->mTimerID == timer)
    {
      delete (*it);
      mTimeouts.erase(it);
    }
  }
}

uint32_t Timer::getExpiration(timer_t timer)
{
  Timeout* pTo = getTimeoutStruct(timer);
  if (pTo == NULL)
    return 0;

  return pTo->mTimestamp;
}

uint32_t Timer::getTimestamp(void)
{
  return getTimerTime(getEnvironment()->getTimestamp());
}



void Timer::step(uint32_t timestamp)
{
  auto it = mTimeouts.begin(); 
  while (it != mTimeouts.end())
  {
    if ((*it)->mTimestamp <= getTimerTime(timestamp)) // watch out for the difference in drift and execution time for event driven Env
    {
      Timeout* to = *it;
      if (!to->invalid)
      {
        to->fire();
      }
      else
      {
        delete to;
        it = mTimeouts.erase(it);
      }

      if (mIteratorInvalidated)
      {
        it = mTimeouts.begin();
        mIteratorInvalidated = false;
      }
    }
    else
    {
      it++;
    }
  }
}

uint32_t Timer::getTimerTime(uint32_t globalTime)
{
  return uint32_t(mDriftFactor * (globalTime - mDriftAnchor) + mDriftAnchor);
}

uint32_t Timer::getGlobalTimeAtLocalTime(uint32_t time)
{
  return uint32_t(mDriftAnchor + (time - mDriftAnchor) / mDriftFactor);
}

Timeout* Timer::getTimeoutStruct(timer_t timer)
{
  for (Timeout* pTo : mTimeouts)
  {
    if (pTo->mTimerID == timer)
      return pTo;
  }

  return NULL;
}