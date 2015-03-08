#include "MeshDevice.h"
#include "WSN.h"
#include "SimEnv.h"
#include "Logger.h"
#include "PowerPlotter.h"
#include "RandomLib\Random.hpp"
#include <Windows.h>

#define DEVICE_COUNT  (20)
#define AREA_SIZE     (30.0)
#define SIM_TIME      (1 * SECONDS)

int main(void)
{
  system("mode con:cols=120 lines=10000");
  HWND console = GetConsoleWindow();
  MoveWindow(console, 0, 0, 800, 1000, true);

  SimEnv env;
  WSN wsn;
  std::vector<MeshDevice*> devices;
  RandomLib::Random randPlacer;

  pLoggerSimEnv = &env;
  env.attachRunnable(&wsn);
  env.setReportRate(1 * SECONDS);
  wsn.setDropRate(0.1);

  for (uint32_t i = 0; i < DEVICE_COUNT; ++i)
  {
    MeshDevice* pDev = new MeshDevice("MESH_" + std::to_string(i), randPlacer.Float() * AREA_SIZE, randPlacer.Float() * AREA_SIZE);
    devices.push_back(pDev);
    wsn.addDevice(pDev);
    pDev->start();
  }
  
  env.run(SIM_TIME / 2);

  // kill the CH
  //devices[6]->stopBeaconing();

  env.run(SIM_TIME);

  PowerPlotter plotter;
  uint32_t orphans = 0;
  for (MeshDevice* pDev : devices)
  {
    //pDev->print();
    //plotter.addDevice(pDev);
    if (!pDev->hasClusterHead())
    {
      orphans++;
    }
  }
  printf("Orphans: %d\n", orphans);
  wsn.exportGraphViz("test_small", AREA_SIZE);
  //plotter.displayGraph(0, 200*MS);
  system("pause");
  return 0;
}