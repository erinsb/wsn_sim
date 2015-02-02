#include "SimEnv.h"


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
  mRunnables.push_back(runnable);
}

void SimEnv::step(uint32_t deltaTime)
{
  mTime += deltaTime;

  for (Runnable* pRunnable : mRunnables)
  {
    pRunnable->step(mTime);
  }
}