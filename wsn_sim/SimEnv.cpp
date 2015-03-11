#include "SimEnv.h"
#include "Timer.h"
#include "Multithreading.h"
#include <thread>
#include "Logger.h"

#undef LOG_ENABLE
#define LOG_ENABLE  (1)

#define SORTED_EXECUTION_LIST (1)

SimEnv::SimEnv(void) : mReportRate(0), mLastReport(0)
{
  mTime = 0;
#if USE_THREADS
  mEndBarrier.reset(THREAD_COUNT + 1);
  mStartBarrier.reset(THREAD_COUNT + 1);
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
  {
    mThreads[i] = new SimThread(this, i);
  }
#endif
}


SimEnv::~SimEnv()
{
#if USE_THREADS
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
  {
    mThreads[i]->mRunning = false;
  }
  mStartBarrier.free();
  mEndBarrier.wait();
  mEndBarrier.free();
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
  {
    mThreads[i]->mThread->join();
  }
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
    delete mThreads[i];
#endif
}

void SimEnv::attachRunnable(Runnable* runnable)
{
  runnable->assignEnvironment(this);
#if USE_THREADS
  mTempRunnables.push_back(runnable);
#else
  mRunnables.push_back(runnable);
#endif
}

uint32_t SimEnv::numberOfRunnables(void) const
{
#if USE_THREADS
  uint32_t sum = 0;
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
    sum += mThreads[i]->mRunnables.size();

  return sum;
#else
  return mRunnables.size();
#endif
}

void SimEnv::run(timestamp_t stopTime, timestamp_t deltaTime)
{
#if USE_THREADS
  uint32_t index = 0;
  while (mTempRunnables.size() > 0)
  {
    mThreads[(index++) % THREAD_COUNT]->mRunnables.push_back(mTempRunnables.back());
    mTempRunnables.pop_back();
  }

  while (mRunning && mTime < stopTime)
  { 
    mTime += deltaTime;

    mEndBarrier.reset(THREAD_COUNT + 1);
    mStartBarrier.wait(); // wait for everyone to get here

    //threads will run now

    mStartBarrier.reset(THREAD_COUNT + 1);
    mEndBarrier.wait();
  }
  mEndBarrier.reset(THREAD_COUNT + 1);
#else
  if (mReportRate > 0 && LOG_ENABLE)
  {
    registerExecution(NULL, mTime + mReportRate - (mTime + mReportRate) % mReportRate);
  }


  while (mRunning && mTime < stopTime)
  {
    step(stopTime);
    mTime = mNextExecution;
  }
#endif
}


void SimEnv::registerExecution(Runnable* executor, timestamp_t timestamp)
{
  if (true || timestamp > getTimestamp())
  {
    if (timestamp < mNextExecution || mExecutionList.size() == 0)
    {
      mNextExecution = timestamp;
    }
    execution_t ex = { executor, timestamp };
    mExecutionListInvalidated = true;
#if SORTED_EXECUTION_LIST
    if (mExecutionList.size() == 0)
    {
      mExecutionList.push_back(ex);
      return;
    }

    for (auto it = mExecutionList.rbegin(); it != mExecutionList.rend(); it++)
    {
      if (it->timestamp > timestamp)
      {
        mExecutionList.insert(it.base(), ex);
        return;
      }
    }
    mExecutionList.insert(mExecutionList.begin(), ex); // last exec
#else
    mExecutionList.push_back(ex);
#endif
  }
  else
  {
    executor->step(getTimestamp());
  }
}


void SimEnv::step(timestamp_t stopTime)
{
  mNextExecution = stopTime;
  auto it = mExecutionList.rbegin();
  volatile uint32_t i = 0;
  while (it != mExecutionList.rend())
  {
    execution_t* pEx = &(*it);
    if (pEx->timestamp <= mTime)
    {
      Runnable* pRunnable = pEx->pRunnable;
      timestamp_t timestamp = pEx->timestamp;

      mExecutionList.erase(--(it.base()));

      if (pRunnable != NULL)
        pRunnable->step(timestamp);

      if (mReportRate > 0 && mTime % mReportRate == 0 && mTime > mLastReport && LOG_ENABLE)
      {
        LOG_COLOR(FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        printf("|%02llu:%02llu.%06llu|----------------------------------------------------------------\n", mTime / MINUTES, (mTime / SECONDS) % 60, mTime % SECONDS);
        LOG_COLOR_RESET;
        mLastReport = mTime;
        registerExecution(NULL, mTime + mReportRate);
      }

      if (mExecutionListInvalidated)
      {
        it = mExecutionList.rbegin(); // reset, as the vector is dirty
      }
    }
    else
    {
      if (pEx->timestamp < mNextExecution)
      {
        mNextExecution = pEx->timestamp;
      }
#if SORTED_EXECUTION_LIST
      break;
#else
      it++;
#endif
    }
    i++;
  }
}



#if USE_THREADS
SimThread::SimThread(SimEnv* myEnv, uint8_t index) : mIndex(index), mEnv(myEnv), mRunning(true)
{
  mThread = new std::thread(&SimThread::run, this);
}

SimThread::~SimThread()
{
  mEnv->mStartBarrier.free();
  mEnv->mEndBarrier.free();
  mRunning = false;
  delete mThread;
}

void SimThread::run(void)
{
  while (mRunning)
  {
    //wait for start
    mEnv->mStartBarrier.wait();
    if (!mRunning)
      break;

    for (uint32_t i = 0; i < mRunnables.size(); ++i)
    {
      mRunnables[i]->step(mEnv->mTime);
    }
    //wait for the rest to finish
    mEnv->mEndBarrier.wait();

  }
  mEnv->mEndBarrier.wait();
}

#endif