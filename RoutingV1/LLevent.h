#pragma once
#include <stdint.h>
#include <functional>

class LLevent
{
public:
  LLevent(uint32_t connEvent, uint32_t duration, std::function<void(void)> callback)
    : mConnEvent(connEvent)
    , mDuration(duration)
    , mCallback(callback)
  {}

  uint32_t getStart(void) { return mConnEvent; }
  uint32_t getEnd(void) { return mConnEvent + mDuration; }
  void fire(void) { mCallback(); }
  

private:
  std::function<void(void)> mCallback;
  uint32_t mConnEvent;
  uint32_t mDuration;
};