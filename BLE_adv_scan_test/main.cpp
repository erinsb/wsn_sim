#include "AdvDevice.h"
#include "WSN.h"


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

  while (true)
  {
    simEnv.step(1);
  }

  return 0;
}