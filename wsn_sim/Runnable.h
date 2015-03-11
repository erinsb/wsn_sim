#pragma once
#include <stdint.h>

typedef uint64_t timestamp_t;

#include "SimEnv.h"
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

  virtual void step(timestamp_t time) = 0;

private:
  SimEnv* p_mSimEnv = NULL;
};

