#include "AdvDevice.h"
#include "ScanDevice.h"
#include "WSN.h"
#include <iostream>
#include <fstream>

int main(void)
{
  WSN wsn;
  SimEnv simEnv;
  simEnv.attachRunnable(&wsn);

  uint8_t advMsg[] {1, 2, 3, 4};
  uint8_t scanMsg[] {5, 6, 7, 8};
  AdvDevice advDev(advMsg, 4, scanMsg, 4);
  ScanDevice scanDev;
  wsn.addDevice(advDev);
  wsn.addDevice(scanDev);

  advDev.start();
  scanDev.start(30000, 30000);

  simEnv.run(2000000);
  
  wsn.exportGraphViz("testGraph");

#if 0
  auto usage = scanDev.getPowerUsage();

  std::cout << std::endl;
  std::ofstream ofs;
  ofs.open("log.txt");
  for (double mA : usage)
    ofs << mA << std::endl;

  std::cout << advDev.getPowerUsageAvg() << "mA" << std::endl;
#endif
  system("pause");
  return 0;
}