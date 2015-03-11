#pragma once
#include "Runnable.h"
#include "Multithreading.h"
#include <vector>
#include <stdint.h>

#define USE_THREADS   (0)
#define THREAD_COUNT  (4)
#define MS            (1000ULL)
#define SECONDS       (1000ULL * MS)
#define MINUTES       (60ULL * SECONDS)
#define HOURS         (60ULL * MINUTES)

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

  timestamp_t getTimestamp(void){ return mTime; }
  uint32_t numberOfRunnables(void) const;
  void attachRunnable(Runnable* runnable);
  void setReportRate(timestamp_t rate) { mReportRate = rate; };
  void stop(void) { mRunning = false; }

  void run(timestamp_t stopTime = UINT32_MAX, timestamp_t deltaTime = 1);
  void registerExecution(Runnable* executor, timestamp_t timestamp);

private:
  typedef struct
  {
    Runnable* pRunnable;
    timestamp_t timestamp;
  } execution_t;

  timestamp_t mTime;
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
  timestamp_t mNextExecution = 0;
  bool mExecutionListInvalidated = false;
  timestamp_t mReportRate;
  timestamp_t mLastReport;

  void step(timestamp_t stopTime);
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

