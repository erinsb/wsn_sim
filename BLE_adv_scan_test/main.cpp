#include "AdvDevice.h"
#include "ScanDevice.h"
#include "WSN.h"
#include "BlePacket.h"
#include <iostream>
#include <fstream>
#include <vector>

#define ADVDEV_COUNT  (10)
#define AREA          (100)

#define SIM_TIME      (1 * SECONDS)

int main(int* argcp, char** argv)
{
  WSN wsn;
  SimEnv simEnv;
  simEnv.attachRunnable(&wsn);

  ble_adv_packet_t adv_packet = {};
  adv_packet.access_addr = 0xAABBCCDD;
  adv_packet.header.addr_type = 0;
  adv_packet.setAdvAddr(0xAABBCCDDEEFF);
  adv_packet.header.length = 6 + 2;
  adv_packet.payload.adv.raw[0] = 0xAA;
  adv_packet.payload.adv.raw[1] = 0xBB;
  adv_packet.header.type = BLE_PACKET_TYPE_ADV_DISCOVER_IND;

  ble_adv_packet_t scan_req_packet = {};
  scan_req_packet.access_addr = 0xAABBCCDD;
  scan_req_packet.header.addr_type = 0;
  scan_req_packet.setAdvAddr(0xAABBCCDDEEFF);
  scan_req_packet.header.length = 6 + 2;
  scan_req_packet.payload.adv.raw[0] = 0xCC;
  scan_req_packet.payload.adv.raw[1] = 0xDD;
  scan_req_packet.header.type = BLE_PACKET_TYPE_SCAN_RSP;

  ScanDevice scanDev;
  wsn.addDevice(&scanDev);
  AdvDevice* pFirstAdv = NULL;
  for (uint32_t i = 0; i < ADVDEV_COUNT; ++i)
  {
    AdvDevice* pDev = new AdvDevice(&adv_packet, &scan_req_packet, rand() % 100000 + 100000);
    if (i == 0)
      pFirstAdv = pDev;
    pDev->pos.x = rand() % AREA;
    pDev->pos.y = rand() % AREA;
    wsn.addDevice(pDev);
  
    pDev->start();
  }

  scanDev.start(30000, 30000);

  simEnv.run(SIM_TIME);

  PowerPlotter plotter;
  plotter.addDevice(pFirstAdv);
  plotter.addDevice(&scanDev);
  plotter.displayGraph(100*MS, 101*MS);
  
  wsn.exportGraphViz("testGraph");

  system("pause");
  return 0;
}