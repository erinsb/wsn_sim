#pragma once
#include "SimEnv.h"
#include <stdint.h>
#include <stdlib.h>

class SimEnv;

class Runnable
{
  friend SimEnv;
public:
  Runnable();
  ~Runnable();
  void assignEnvironment(SimEnv* simEnv) { this->p_mSimEnv = simEnv; }

  SimEnv* getEnvironment(void) const { return p_mSimEnv; }

  virtual void step(uint32_t time) = 0;

private:
  SimEnv* p_mSimEnv = NULL;
  void synchronizedStep(uint32_t time, Barrier& barrier);
};

