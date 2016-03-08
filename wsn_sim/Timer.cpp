#include "Timer.h"
#include "Logger.h"

void Timeout::fire(Timer* pTimer)
{
  if (mInterval == 0)
    invalid = true;
  mCallback(pTimer->getTimerTime(mTimestamp), mContext);
}


Timer::Timer(double drift) : mDriftFactor(drift), mDriftAnchor(0), mCurrentTimeout(NULL), mNextTimerIndex(1)
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

timer_t Timer::orderAt(timestamp_t timestamp, const std::function<void(timestamp_t, void*)> callback, void* context)
{
  mTimeouts.push_back(new Timeout(getGlobalTimeAtLocalTime(timestamp), mNextTimerIndex, callback, context));
  getEnvironment()->registerExecution(this, getGlobalTimeAtLocalTime(timestamp));
  mIteratorInvalidated = true;

  return mNextTimerIndex++;
}

timer_t Timer::orderRelative(timestamp_t deltaTime, const std::function<void(timestamp_t, void*)> callback, void* context)
{
  return orderAt((timestamp_t) (getTimestamp() + deltaTime * mDriftFactor), callback, context);
}

timer_t Timer::orderPeriodic(timestamp_t firstTimeout, timestamp_t interval, const std::function<void(timestamp_t, void*)> callback, void* context)
{
  timer_t id = orderAt(firstTimeout, callback, context);
  getTimeoutStruct(id)->mInterval = interval;

  return id;
}

void Timer::reschedule(timer_t timer, timestamp_t timestamp)
{
  Timeout* pTo = getTimeoutStruct(timer);

  if (pTo == NULL)
    _ERROR("Attempted to reschedule non-existent timer: %d", timer);

  pTo->mTimestamp = getGlobalTimeAtLocalTime(timestamp);
  getEnvironment()->registerExecution(this, getGlobalTimeAtLocalTime(timestamp));
}

void Timer::changeInterval(timer_t timer, timestamp_t interval)
{
  Timeout* pTo = getTimeoutStruct(timer);

  if (pTo == NULL)
    _ERROR("Attempted to change periodicity of non-existent timer: %d", timer);
  
  pTo->mInterval = interval;
}

void Timer::abort(timer_t& timer)
{
  for (auto it = mTimeouts.begin(); it != mTimeouts.end();)
  {
    if ((*it)->mTimerID == timer)
    {
      if (*it == mCurrentTimeout)
      {
        (*it)->invalid = true;
        it++;
      }
      else
      {
        delete (*it);
        it = mTimeouts.erase(it);
      }
      mIteratorInvalidated = true;
    }
    else
    {
      it++;
    }
  }
  timer = TIMER_INVALID;
}

timestamp_t Timer::getExpiration(timer_t timer)
{
  Timeout* pTo = getTimeoutStruct(timer);

	if (pTo == NULL) {
		//_ERROR("Asked for expiration of non-existent timer");
		//_WARN("Asked for expiration of non-existent timer");
		return NULL;
	}
  return getTimerTime(pTo->mTimestamp);
}

timestamp_t Timer::getTimestamp(void)
{
  return getTimerTime(getEnvironment()->getTimestamp());
}

void Timer::setContext(timer_t timer, void* context)
{
  Timeout* pTo = getTimeoutStruct(timer);
  if (pTo != NULL)
    pTo->mContext = context;
}



void Timer::step(timestamp_t timestamp)
{
  auto it = mTimeouts.begin(); 
  while (it != mTimeouts.end())
  {
    if ((*it)->mTimestamp <= timestamp + TIMER_DRIFT_MARGIN) // watch out for the difference in drift and execution time for event driven Env
    {
      mCurrentTimeout = *it;
      if (!mCurrentTimeout->invalid)
      {
        mCurrentTimeout->fire(this);

        // reschedule periodic timer:
        if (mCurrentTimeout->mInterval > 0 && !mCurrentTimeout->invalid)
        {
          mCurrentTimeout->mTimestamp += (timestamp_t)(mCurrentTimeout->mInterval * mDriftFactor);
          mCurrentTimeout->invalid = false;

          getEnvironment()->registerExecution(this, mCurrentTimeout->mTimestamp);
        }
        mCurrentTimeout = NULL;
      }
      else
      {
        delete mCurrentTimeout;
        mCurrentTimeout = NULL;
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

timestamp_t Timer::getTimerTime(timestamp_t globalTime)
{
  return timestamp_t(mDriftFactor * (globalTime - mDriftAnchor) + mDriftAnchor);
}

bool Timer::isValidTimer(timer_t timer)
{
  return (getTimeoutStruct(timer) != NULL);
}

timestamp_t Timer::getGlobalTimeAtLocalTime(timestamp_t time)
{
  return timestamp_t(mDriftAnchor + (time - mDriftAnchor) / mDriftFactor);
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