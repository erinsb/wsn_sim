#include "SimEnv.h"
#include "Multithreading.h"
#include <thread>
#include "Logger.h"


SimEnv::SimEnv(void)
{
  mTime = 0;
  mEndBarrier.reset(THREAD_COUNT + 1);
  mStartBarrier.reset(THREAD_COUNT + 1);
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
  {
    mThreads[i] = new SimThread(this, i);
  }
}


SimEnv::~SimEnv()
{
#if 1
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
  mTempRunnables.push_back(runnable);
}

uint32_t SimEnv::numberOfRunnables(void) const
{
  uint32_t sum = 0;
  for (uint8_t i = 0; i < THREAD_COUNT; ++i)
    sum += mThreads[i]->mRunnables.size();

  return sum;
}

void SimEnv::run(uint32_t stopTime, uint32_t deltaTime)
{
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
}

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
