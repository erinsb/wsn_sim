#pragma once
#include "WSN.h"
#include "Runnable.h"



class Device : public Runnable
{
  friend WSN;
  friend Radio;
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
  virtual void radioCallbackTx(RadioPacket* packet);
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted);
private:
};

