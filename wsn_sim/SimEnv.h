#pragma once
#include "Runnable.h"
#include "Multithreading.h"
#include <mutex>
#include <vector>
#include <thread>
#include <stdint.h>

#define THREAD_COUNT  (8)

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
  uint32_t numberOfRunnables(void) const { return mRunnables.size(); }
  void attachRunnable(Runnable* runnable);

  void step(uint32_t deltaTime = 1);

private:
  uint32_t mTime;
  std::vector<Runnable*> mRunnables;
  std::vector<Runnable*> mTempRunnables;
  SimThread mThreads[THREAD_COUNT];
  Barrier mBarrier;
  std::mutex mStartMut;
  std::condition_variable mStartCV;
};

class SimThread
{
public:
  SimThread(SimEnv* myEnv, uint8_t index);
private:
  uint8_t mIndex;
  SimEnv* mEnv;
  std::thread mThread;
  void run(void);
};
