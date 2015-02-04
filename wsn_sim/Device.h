#pragma once
#include "WSN.h"
#include "Runnable.h"
#include "Timer.h"



class Device : public Runnable, public TimerClient
{
  friend WSN;
  friend Radio;
  friend Timer;
public:
  Device(double x, double y);
  Device();
  ~Device();

  Radio* getRadio(void) const { return mRadio; }

  double getDistanceTo(Device& device) const;

  virtual void step(uint32_t timestamp);

  typedef struct
  {
    double x, y;
  } pos_t;

  pos_t pos;


protected:
  Radio* mRadio;
  Timer* mTimer;
  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);
  virtual void timerEnded(Timeout* timeout);
private:
};

