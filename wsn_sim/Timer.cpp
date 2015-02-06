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

void Timer::orderAt(uint32_t timestamp, const std::function<void(uint32_t, void*)> callback, void* context)
{
  mTimeouts.push_back(new Timeout(timestamp, callback, context));
  mIteratorInvalidated = true;
}

void Timer::orderRelative(uint32_t deltaTime, const std::function<void(uint32_t, void*)> callback, void* context)
{
  orderAt(getTimestamp() + deltaTime, callback, context);

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
    if ((*it)->mTimestamp <= timestamp)
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
