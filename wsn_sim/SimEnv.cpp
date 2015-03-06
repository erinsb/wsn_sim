#include "SimEnv.h"
#include "Multithreading.h"
#include <thread>
#include "Logger.h"


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

void SimEnv::run(uint32_t stopTime, uint32_t deltaTime)
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
  if (mReportRate > 0)
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


void SimEnv::registerExecution(Runnable* executor, uint32_t timestamp)
{
  if (timestamp > getTimestamp())
  {
    if (timestamp < mNextExecution || mExecutionList.size() == 0)
    {
      mNextExecution = timestamp;
    }
    execution_t ex = { executor, timestamp };
    mExecutionList.push_back(ex);
    mExecutionListInvalidated = true;
  }
  else
  {
    executor->step(getTimestamp());
  }
}


void SimEnv::step(uint32_t stopTime)
{
  mNextExecution = stopTime;
  auto it = mExecutionList.begin();
  volatile uint32_t i = 0;
  while (it != mExecutionList.end())
  {
    if (it->timestamp <= mTime)
    {
      Runnable* pRunnable = it->pRunnable;
      uint32_t timestamp = it->timestamp;

      it = mExecutionList.erase(it);

      if (pRunnable != NULL)
        pRunnable->step(timestamp);

      if (mTime % mReportRate == 0 && mTime > mLastReport)
      {
        LOG_COLOR(FOREGROUND_BLUE | FOREGROUND_INTENSITY);
        printf("|%02d:%02d.%06d|----------------------------------------------------------------\n", mTime / MINUTES, mTime / SECONDS, mTime % SECONDS);
        LOG_COLOR_RESET;
        mLastReport = mTime;
        registerExecution(NULL, mTime + mReportRate);
      }

      if (mExecutionListInvalidated)
      {
        it = mExecutionList.begin(); // reset, as the vector is dirty
      }
    }
    else
    {
      if (it->timestamp < mNextExecution)
      {
        mNextExecution = it->timestamp;
      }
      it++;
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