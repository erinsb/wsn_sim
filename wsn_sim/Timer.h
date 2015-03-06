#pragma once
#include "Runnable.h"
#include <functional>

#define PPM   (1.0 / 1000000.0)
#define MEMBER_TIMEOUT(func)   std::bind(&func, this, std::placeholders::_1, std::placeholders::_2)
#define TIMER_DRIFT_MARGIN  (2)

typedef uint32_t timer_t;

class Timeout
{
public:
  Timeout(uint32_t timestamp, timer_t id, const std::function<void(uint32_t, void*)> callback, void* context) : mTimestamp(timestamp), mCallback(callback), mContext(context), mInterval(0), mTimerID(id) {}

  void fire(void) 
  { 
    if (mInterval == 0)
      invalid = true;
    mCallback(mTimestamp, mContext); 
  }
  const std::function<void(uint32_t, void*)> mCallback;
  uint32_t mTimestamp;
  uint32_t mInterval;
  bool invalid = false;
  timer_t mTimerID;
  void* mContext;
};

class Timer : public Runnable
{
public:
  Timer(double drift);
  ~Timer();

  void setDriftFactor(double driftFactor) { mDriftFactor = driftFactor; }
  double getDriftFactor(void) { return mDriftFactor; }
  void resetDrift(void);

  timer_t orderAt(uint32_t timestamp, const std::function<void(uint32_t, void*)> callback, void* context = NULL);
  timer_t orderRelative(uint32_t deltaTime, const std::function<void(uint32_t, void*)> callback, void* context = NULL);
  timer_t orderPeriodic(uint32_t firstTimeout, uint32_t interval, const std::function<void(uint32_t, void*)> callback, void* context = NULL);
  void reschedule(timer_t timer, uint32_t timestamp);
  void abort(timer_t timer);
  uint32_t getExpiration(timer_t timer);
  void setContext(timer_t timer, void* context);

  uint32_t getTimestamp(void);

  virtual void step(uint32_t timestamp);

  uint32_t getGlobalTimeAtLocalTime(uint32_t time);
  uint32_t getTimerTime(uint32_t globalTime);

private:
  uint32_t mDriftAnchor;
  double mDriftFactor;
  std::vector<Timeout*> mTimeouts;
  bool mIteratorInvalidated = false;
  timer_t mNextTimerIndex;
  Timeout* mCurrentTimeout;

  Timeout* getTimeoutStruct(timer_t timer);
};

