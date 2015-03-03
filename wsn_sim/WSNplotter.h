#pragma once
#include "../MatplotPP/matplotpp.h"
#include "WSN.h"
#include <thread>

class WSNplotter : public MatPlot
{
public:
  WSNplotter(WSN* pWSN);
  ~WSNplotter();


  void toggleGrid(void);
  void createDisplay(void);
  void refresh(void);

  void DISPLAY(void);
private:
  WSN* m_pWSN;
  std::thread mGLThread;
  bool mGrid;

  void drawDevice(Device* pDev);
  void drawConnection(Device* pDev1, Device* pDev2);
};

