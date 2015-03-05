#pragma once
#include "WSN.h"
#include "Runnable.h"
#include "Timer.h"
#include "PowerPlotter.h"


class Device : public Runnable
{
  friend WSN;
  friend Radio;
  friend Timer;
public:
  typedef struct 
  {
    double power_mA;
    uint32_t timestamp;
  }powerEvent_t;


  Device(double x, double y);
  Device();
  ~Device();

  Radio* getRadio(void) const { return mRadio; }

  double getDistanceTo(Device& device) const;

  virtual void step(uint32_t timestamp);

  //power
  void registerPowerDrain(double power_mA);
  void removePowerDrain(double power_mA);
  std::vector<double> getPowerUsage(uint32_t firstSample = 0, uint32_t lastSample = UINT32_MAX) const;
  std::vector<powerEvent_t> getPowerUsageEvents(void) { return powerUsage; }
  double getPowerUsageAvg(uint32_t firstSample = 0, uint32_t lastSample = UINT32_MAX) const;
  void plotPower(uint32_t startTime = 0, uint32_t endTime = UINT32_MAX);

  typedef struct
  {
    double x, y;
  } pos_t;

  pos_t pos;
  std::string mName;


protected:
  Radio* mRadio;
  Timer* mTimer;
  WSN* mWSN;
  virtual void radioCallbackTx(RadioPacket* packet){};
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted){};
private:
  std::mutex powerMut;
  std::vector<powerEvent_t> powerUsage;
};

