#include "SimEnv.h"
#include "Multithreading.h"
#include <thread>


SimEnv::SimEnv(void)
{
  mTime = 0;
}


SimEnv::~SimEnv()
{
}

void SimEnv::attachRunnable(Runnable* runnable)
{
  runnable->assignEnvironment(this);
  mTempRunnables.push_back(runnable);
}

void SimEnv::step(uint32_t deltaTime)
{
  mTime += deltaTime;

  while (mTempRunnables.size() > 0)
  {
    mRunnables.push_back(mTempRunnables.back());
    mTempRunnables.pop_back();
  }
  uint32_t runnable_count = mRunnables.size();

  mBarrier.reset(runnable_count + 1);

  for (Runnable* pRunnable : mRunnables)
  {
    pRunnable->step(mTime);
  }
}

SimThread::SimThread(SimEnv* myEnv, uint8_t index) : mIndex(index), mEnv(myEnv), mThread(&SimThread::run, this)
{
}

void SimThread::run(void)
{
  while (true)
  {
    for (uint32_t i = mIndex; i < mEnv->mRunnables.size(); i += THREAD_COUNT)
    {
      mEnv->mRunnables[i]->step(mEnv->mTime);
    }
    mEnv->mBarrier.wait();
  }
}
