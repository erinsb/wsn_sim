#include "AdvDevice.h"
#include "ScanDevice.h"
#include "WSN.h"
#include <iostream>
#include <fstream>
#include <vector>

#define ADVDEV_COUNT  (100)
#define AREA          (100)

int main(void)
{
  WSN wsn;
  SimEnv simEnv;
  simEnv.attachRunnable(&wsn);

  uint8_t advMsg[] {1, 2, 3, 4};
  uint8_t scanMsg[] {5, 6, 7, 8};
  ScanDevice scanDev;
  wsn.addDevice(scanDev);

  for (uint32_t i = 0; i < ADVDEV_COUNT; ++i)
  {
    AdvDevice* pDev = new AdvDevice(advMsg, 4, scanMsg, 4, rand() % 100000 + 100000);

    pDev->pos.x = rand() % AREA;
    pDev->pos.y = rand() % AREA;
    wsn.addDevice(*pDev);
  
    pDev->start();
  }
  

  scanDev.start(30000, 30000);

  simEnv.run(2 * SECONDS);

  
#if 1
  auto usage = scanDev.getPowerUsageEvents();

  std::cout << std::endl;
  std::ofstream ofs;
  ofs.open("log.txt");
  for (Device::powerEvent_t& ev : usage)
    ofs << ev.power_mA << "\t" << ev.timestamp << std::endl;

  std::cout << advDev.getPowerUsageAvg() << "mA" << std::endl;
#endif
  system("pause");
  return 0;
}