#pragma once
#include "../MatplotPP/matplotpp.h"
#include "Device.h"
#include "Timer.h"

class PowerPlotter : public MatPlot
{
public:
  PowerPlotter::PowerPlotter(void) : mGrid(0){}

  void specialKeys(int key, int x, int y);

  void toggleGrid(void);
  void addDevice(Device* pDev) { mDevices.push_back(pDev); mStartOffsets.push_back(0); }
  void displayGraph(timestamp_t startTime, timestamp_t endTime, Device* pDev = NULL);

  void DISPLAY(void); // don't use
  timestamp_t mStartTime, mEndTime;
  timestamp_t mDelta;
private:
  int mGrid;
  std::vector<Device*> mDevices;
  std::vector<uint32_t> mStartOffsets;

  void plotDevice(Device* pDev, uint32_t index);
};

