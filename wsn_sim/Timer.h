#pragma once
#include "Runnable.h"
#include <functional>
#include <stdint.h>

#define PPM   (1.0 / 1000000.0)
#define MEMBER_TIMEOUT(func)   std::bind(&func, this, std::placeholders::_1, std::placeholders::_2)
#define TIMER_DRIFT_MARGIN  (2)
#define TIMER_INVALID       (0)

typedef uint32_t timer_t; // handle

class Timer;

class Timeout
{
public:
  Timeout(timestamp_t timestamp, 
    timer_t id, 
    const std::function<void(timestamp_t, void*)> callback, 
    void* context) 
    : mTimestamp(timestamp)
    , mCallback(callback)
    , mContext(context)
    , mInterval(0)
    , mTimerID(id) {}

  void fire(Timer* pTimer);
  const std::function<void(timestamp_t, void*)> mCallback;
  timestamp_t mTimestamp;
  timestamp_t mInterval;
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

  timer_t orderAt(timestamp_t timestamp, const std::function<void(timestamp_t, void*)> callback, void* context = NULL);
  timer_t orderRelative(timestamp_t deltaTime, const std::function<void(timestamp_t, void*)> callback, void* context = NULL);
  timer_t orderPeriodic(timestamp_t firstTimeout, timestamp_t interval, const std::function<void(timestamp_t, void*)> callback, void* context = NULL);
  void reschedule(timer_t timer, timestamp_t timestamp);
  void changeInterval(timer_t timer, timestamp_t interval);
  void abort(timer_t& timer);
  timestamp_t getExpiration(timer_t timer);
  void setContext(timer_t timer, void* context);

  timestamp_t getTimestamp(void);

  virtual void step(timestamp_t timestamp);

  timestamp_t getGlobalTimeAtLocalTime(timestamp_t time);
  timestamp_t getTimerTime(timestamp_t globalTime);
  bool isValidTimer(timer_t timer);
private:
  timestamp_t mDriftAnchor;
  double mDriftFactor;
  std::vector<Timeout*> mTimeouts;
  bool mIteratorInvalidated = false;
  timer_t mNextTimerIndex;
  Timeout* mCurrentTimeout;

  Timeout* getTimeoutStruct(timer_t timer);
};

