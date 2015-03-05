#pragma once
#include "Runnable.h"
#include "Multithreading.h"
#include <mutex>
#include <vector>
#include <thread>
#include <stdint.h>

#define USE_THREADS   (0)
#define THREAD_COUNT  (4)
#define MS            (1000)
#define SECONDS       (1000 * MS)
#define MINUTES       (60 * SECONDS)
#define HOURS         (60 * MINUTES)

class Runnable;

#if USE_THREADS
class SimThread;
#endif

class SimEnv
{
  friend Runnable;
#if USE_THREADS
  friend SimThread;
#endif

public:
  SimEnv();
  ~SimEnv();

  uint32_t getTimestamp(void){ return mTime; }
  uint32_t numberOfRunnables(void) const;
  void attachRunnable(Runnable* runnable);
  void stop(void) { mRunning = false; }

  void run(uint32_t stopTime = UINT32_MAX, uint32_t deltaTime = 1);
  void registerExecution(Runnable* executor, uint32_t timestamp);

private:
  typedef struct
  {
    Runnable* pRunnable;
    uint32_t timestamp;
  } execution_t;

  uint32_t mTime;
#if USE_THREADS
  std::vector<Runnable*> mTempRunnables;
  SimThread* mThreads[THREAD_COUNT];
  Barrier mStartBarrier, mEndBarrier;
  std::mutex mStartMut;
  std::condition_variable mStartCV;
#else
  std::vector<Runnable*> mRunnables;
#endif
  bool mRunning = true;
  std::vector<execution_t> mExecutionList;
  uint32_t mNextExecution = 0;
  bool mExecutionListInvalidated = false;
  void step(uint32_t stopTime);
};


#if USE_THREADS
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
#endif

