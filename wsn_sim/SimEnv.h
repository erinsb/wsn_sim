#pragma once
#include "Runnable.h"
#include "Multithreading.h"
#include <mutex>
#include <vector>
#include <thread>
#include <stdint.h>

#define THREAD_COUNT  (4)

class Runnable;
class SimThread;

class SimEnv
{
  friend Runnable;
  friend SimThread;

public:
  SimEnv();
  ~SimEnv();

  uint32_t getTimestamp(void){ return mTime; }
  uint32_t numberOfRunnables(void) const;
  void attachRunnable(Runnable* runnable);
  void stop(void) { mRunning = false; }

  void run(uint32_t stopTime = UINT32_MAX, uint32_t deltaTime = 1);

private:
  uint32_t mTime;
  std::vector<Runnable*> mTempRunnables;
  SimThread* mThreads[THREAD_COUNT];
  Barrier mStartBarrier, mEndBarrier;
  std::mutex mStartMut;
  std::condition_variable mStartCV;
  bool mRunning = true;
};

class SimThread
{
  friend SimEnv;
public:
  SimThread(SimEnv* myEnv, uint8_t index);
  ~SimThread();
private:
  bool mRunning;
  uint8_t mIndex;
  SimEnv* mEnv;
  std::thread* mThread;
  std::vector<Runnable*> mRunnables;
  void run(void);
};
