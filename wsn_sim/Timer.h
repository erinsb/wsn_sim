#pragma once
#include "Runnable.h"
#include <functional>

#define PPM   (1.0 / 1000000.0)

class Timeout
{
public:
  Timeout(uint32_t timestamp, const std::function<void(uint32_t, void*)> callback, void* context) : mTimestamp(timestamp), mCallback(callback), mContext(context){}

  void fire(void) 
  { 
    invalid = true;
    mCallback(mTimestamp, mContext); 
  }
  const std::function<void(uint32_t, void*)> mCallback;
  uint32_t mTimestamp;
  bool invalid = false;
  void* mContext;
};

class Timer : public Runnable
{
public:
  Timer(double drift);
  ~Timer();

  void setDriftFactor(double driftFactor) { mDriftFactor = driftFactor; }
  void resetDrift(void);

  void orderAt(uint32_t timestamp, const std::function<void(uint32_t, void*)> callback, void* context = NULL);
  
  void orderRelative(uint32_t deltaTime, const std::function<void(uint32_t, void*)> callback, void* context = NULL);

  uint32_t getTimestamp(void);

  virtual void step(uint32_t timestamp);

  uint32_t getGlobalTimeAtLocalTime(uint32_t time);

private:
  uint32_t mDriftAnchor;
  double mDriftFactor;
  std::vector<Timeout*> mTimeouts;
  bool mIteratorInvalidated = false;

  uint32_t getTimerTime(uint32_t globalTime);
};

