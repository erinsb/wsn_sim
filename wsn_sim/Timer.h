#pragma once
#include "Runnable.h"


class Timeout
{
public:
  Timeout(uint32_t timestamp, void* context) : mTimestamp(timestamp), mContext(context){}
  uint32_t mTimestamp;
  void* mContext;
};

class TimerClient
{
public:
  virtual void timerEnded(Timeout* timeout) = 0;
};


class Timer : public Runnable
{
public:
  Timer(double drift, TimerClient& client);
  ~Timer();

  void resetDrift(void);

  void orderAt(uint32_t timestamp, void* context);
  void orderRelative(uint32_t deltaTime, void* context);

  uint32_t getTimestamp(void);

  virtual void step(uint32_t timestamp);

private:
  uint32_t getTimerTime(uint32_t globalTime);

  uint32_t mDriftAnchor;
  double mDriftFactor;
  TimerClient& mClient;
  std::vector<Timeout> mTimeouts;
};

