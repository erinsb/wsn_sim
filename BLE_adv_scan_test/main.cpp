#include "AdvDevice.h"
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
  wsn.addDevice(advDev);

  advDev.start();

  simEnv.run(200000);
  
  auto usage = advDev.getPowerUsage();

  std::cout << std::endl;
  std::ofstream ofs;
  ofs.open("log.txt");
  for (double mA : usage)
    ofs << mA << std::endl;

  std::cout << advDev.getPowerUsageAvg() << "mA" << std::endl;

  system("pause");
  return 0;
}