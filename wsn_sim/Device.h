#pragma once
#include "Radio.h"
#include "Runnable.h"

#define RADIO_DEFAULT_SIGSTRENGTH (100)
#define RADIO_DEFAULT_TURNAROUND  (138)
#define RADIO_DEFAULT_TIFS        (150)
#define RADIO_DEFAULT_BITRATE     (1000)

class Device : public Runnable
{
public:
  Device();
  ~Device();

  const Radio& getRadio(void) const { return mRadio; }

  double getDistanceTo(Device& device) const;

  virtual void step(uint32_t timestamp);

  struct
  {
    double x, y;
  } pos;

private:
  Radio mRadio;
};

