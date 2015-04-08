#pragma once
#include "WSN.h"
#include "Runnable.h"
#include "Timer.h"
#include "PowerPlotter.h"
#include "RandomLib\Random.hpp"


typedef enum
{
  DEVICE_TAG_NONE,
  DEVICE_TAG_WEAK,
  DEVICE_TAG_MEDIUM,
  DEVICE_TAG_STRONG
} device_tag_t;

class Device : public Runnable
{
  friend WSN;
  friend Radio;
  friend Timer;
public:
  typedef struct 
  {
    double power_mA;
    timestamp_t timestamp;
  }powerEvent_t;


  Device(double x, double y);
  Device();
  ~Device();

  Radio* getRadio(void) const { return mRadio; }

  double getDistanceTo(Device& device) const;

  virtual void step(timestamp_t timestamp);

  //power
  void registerPowerDrain(double power_mA, timestamp_t time = TIME_MAX);
  void removePowerDrain(double power_mA);
  std::vector<double> getPowerUsage(timestamp_t firstSample = 0, timestamp_t lastSample = TIME_MAX) const;
  std::vector<powerEvent_t> getPowerUsageEvents(void) { return powerUsage; }
  double getPowerUsageAvg(timestamp_t firstSample = 0, timestamp_t lastSample = TIME_MAX, double paukert = 1.0) const;
  void plotPower(timestamp_t startTime = 0, timestamp_t endTime = TIME_MAX);

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
  device_tag_t mDevTag; // for graph output
  double mExtraPowerUsagePart;
  double mBackgroundPowerUsage;
  RandomLib::Random mRand;
  virtual void radioCallbackTx(RadioPacket* packet){};
  virtual void radioCallbackRx(RadioPacket* packet, uint8_t rx_strength, bool corrupted){};
private:
  std::mutex powerMut;
  std::vector<powerEvent_t> powerUsage;
};

