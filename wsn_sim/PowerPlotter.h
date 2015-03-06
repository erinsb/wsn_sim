#pragma once
#include "../MatplotPP/matplotpp.h"
#include "Device.h"

class PowerPlotter : public MatPlot
{
public:
  PowerPlotter::PowerPlotter(void) : mGrid(0){}

  void specialKeys(int key, int x, int y);

  void toggleGrid(void);
  void addDevice(Device* pDev) { mDevices.push_back(pDev); }
  void displayGraph(uint32_t startTime, uint32_t endTime, Device* pDev = NULL);

  void DISPLAY(void); // don't use
  uint32_t mStartTime, mEndTime;
private:
  int mGrid;
  std::vector<Device*> mDevices;

  void plotDevice(Device* pDev);
};

