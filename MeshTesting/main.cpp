#include "ClusterMeshDev.h"
#include "WSN.h"
#include "SimEnv.h"
#include "Logger.h"
#include "MeshWSN.h"
#include "PowerPlotter.h"
#include "RandomLib\Random.hpp"
#include <Windows.h>

#define DEVICE_COUNT  (6)
#define AREA_SIZE     (5.0)
#define SIM_TIME      (30ULL * SECONDS)

BOOL CtrlHandler(DWORD fdwCtrlType)
{
  if (fdwCtrlType == CTRL_C_EVENT)
    exit(0);


  return true;
}


int main(void)
{
  system("mode con:cols=120 lines=10000");
  HWND console = GetConsoleWindow();
  MoveWindow(console, 0, 0, 800, 1000, true);
  SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);

  SimEnv env;
  MeshWSN wsn;
  std::vector<ClusterMeshDev*> devices;
  RandomLib::Random randPlacer;
  randPlacer.Reseed(4565);

  pLoggerSimEnv = &env;
  env.attachRunnable(&wsn);
  env.setReportRate(1ULL * SECONDS);
  //wsn.setDropRate(0.1);

  for (uint32_t i = 0; i < DEVICE_COUNT; ++i)
  {
    ClusterMeshDev* pDev = new ClusterMeshDev("MESH_" + std::to_string(i), randPlacer.Float() * AREA_SIZE, randPlacer.Float() * AREA_SIZE);
    pDev->setScore(i);
    devices.push_back(pDev);
    wsn.addDevice(pDev);
    pDev->start();
  }
  
#if 0
  //env.run(25*SECONDS);

  // restart beaconing
  devices[1]->startBeaconing();
  //env.run(26 * SECONDS);
  MeshNeighbor* pNb = devices[2]->getNeighbor(devices[1]->getAdvAddress());
  if (pNb)
    devices[2]->resubscribe(pNb);
#endif
  env.run(SIM_TIME);

  PowerPlotter plotter;
  uint32_t orphans = 0;
  for (ClusterMeshDev* pDev : devices)
  {
    //pDev->print();
    plotter.addDevice(pDev);
    /*if (!pDev->hasClusterHead())
    {
      orphans++;
    }*/
  }
  printf("Orphans: %d\n", orphans);
  wsn.print();
  wsn.exportGraphViz("test_small", AREA_SIZE);
  plotter.displayGraph(0, MESH_INTERVAL);
  system("pause");
  return 0;
}