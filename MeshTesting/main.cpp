#include "MeshDevice.h"
#include "WSN.h"
#include "SimEnv.h"
#include "Logger.h"
#include "PowerPlotter.h"
#include <Windows.h>

#define DEVICE_COUNT  (8)
#define AREA_SIZE     (1.0)
#define SIM_TIME      (1 * MINUTES)

int main(void)
{
  system("mode con:cols=120 lines=10000");
  HWND console = GetConsoleWindow();
  MoveWindow(console, 0, 0, 800, 1100, true);

  SimEnv env;
  WSN wsn;
  std::vector<MeshDevice*> devices;
  pLoggerSimEnv = &env;
  env.attachRunnable(&wsn);
  env.setReportRate(1 * SECONDS);

  for (uint32_t i = 0; i < DEVICE_COUNT; ++i)
  {
    MeshDevice* pDev = new MeshDevice("mesh_" + std::to_string(i), ((double)rand() / RAND_MAX) * AREA_SIZE, ((double)rand() / RAND_MAX) * AREA_SIZE);
    devices.push_back(pDev);
    wsn.addDevice(pDev);
    pDev->start();
  }
  
  env.run(SIM_TIME);

  PowerPlotter plotter;
  for (MeshDevice* pDev : devices)
  {
    pDev->print();
    plotter.addDevice(pDev);
  }
  plotter.displayGraph(100*MS, 200*MS);
  return 0;
}