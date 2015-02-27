#include "PowerPlotter.h"
#include <vector>
#include <Windows.h>

#define MOVE_DELTA      (20)
#define MOVE_DELTA_FAST (100)

static void runGLUT(PowerPlotter* pPlotter);
static PowerPlotter* g_pPlotter;

void display(void)
{
  g_pPlotter->display();
}

void reshape(int w, int h)
{
  g_pPlotter->reshape(w, h);
}

void mouse(int button, int state, int x, int y)
{
  g_pPlotter->mouse(button, state, x, y);
}

void motion(int x, int y)
{
  g_pPlotter->motion(x, y);
}

void passivemotion(int x, int y)
{
  g_pPlotter->passivemotion(x, y);
}

void specialKeys(int key, int x, int y)
{
  g_pPlotter->specialKeys(key, x, y);
}

void keyboard(unsigned char key, int x, int y)
{
  if (key == 'g')
    g_pPlotter->toggleGrid();
  g_pPlotter->keyboard(key, x, y);
}

void idle(void)
{ 
  glutPostRedisplay(); 
  Sleep(10); 
}


void runGLUT(PowerPlotter* pPlotter)
{
  g_pPlotter = pPlotter;
  int argc = 1;
  char* argv[] = { "whatever" };
  glutInit(&argc, argv);
  glutCreateWindow(100, 100, 1024, 800);
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(idle);
  glutSpecialFunc(specialKeys);
  //glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutPassiveMotionFunc(passivemotion);
  glutKeyboardFunc(keyboard);

  glutMainLoop();
}

void PowerPlotter::specialKeys(int key, int x, int y)
{
  uint32_t delta = mEndTime - mStartTime;
  uint32_t maxTime = mDevices[0]->getEnvironment()->getTimestamp();
  int movement_speed = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) ? MOVE_DELTA_FAST : MOVE_DELTA;
  movement_speed *= delta / 1000.0;
  uint32_t tempStartTime = mStartTime;
  uint32_t tempEndTime = mEndTime;
  switch (key)
  {
  case GLUT_KEY_LEFT:
    tempStartTime -= movement_speed;
    tempEndTime -= movement_speed;
    break;
  case GLUT_KEY_RIGHT:
    tempStartTime += movement_speed;
    tempEndTime += movement_speed;
    break;
  case GLUT_KEY_PAGE_UP:
    tempStartTime += delta;
    tempEndTime += delta;
    break;
  case GLUT_KEY_PAGE_DOWN:
    tempStartTime -= delta;
    tempEndTime -= delta;
    break;
  case GLUT_KEY_UP:
    if (tempStartTime < movement_speed)
    {
      tempEndTime += movement_speed - tempStartTime;
      tempStartTime = 0;
    }
    else if (tempEndTime + movement_speed > maxTime)
    {
      tempStartTime -= movement_speed + (maxTime - tempEndTime);
      tempEndTime = maxTime;
    }
    else
    {
      tempStartTime -= movement_speed;
      tempEndTime += movement_speed;
    }
    break;
  case GLUT_KEY_DOWN:
    tempStartTime += movement_speed;
    tempEndTime -= movement_speed;
    break;
  }
  if (tempStartTime > tempEndTime || tempEndTime > maxTime)
    return;

  mStartTime = tempStartTime;
  mEndTime = tempEndTime;

  glutPostRedisplay();
}

void PowerPlotter::toggleGrid(void)
{
  mGrid = !mGrid;
  glutPostRedisplay();
}

void PowerPlotter::displayGraph(uint32_t startTime, uint32_t endTime, Device* pDev)
{
  if (pDev != NULL)
    mDevices.push_back(pDev);
  mStartTime = startTime;
  mEndTime = endTime;
  if (mDevices.size() > 0)
    runGLUT(this);
}

void PowerPlotter::DISPLAY(void)
{
  for (uint32_t i = 0; i < mDevices.size(); ++i)
  {
    subplot(mDevices.size(), 1, i + 1);
    plotDevice(mDevices[i]);
  }
}

void PowerPlotter::plotDevice(Device* pDev)
{
  auto evs = pDev->getPowerUsageEvents();
  mStartTime = max(0, int32_t(mStartTime));
  mEndTime = min(pDev->getEnvironment()->getTimestamp(), mEndTime);
  std::vector<double> power, time;

  double currVal = 0.0;
  auto it = evs.begin();

  while (it->timestamp < mStartTime && it != evs.end()) // find last point before start
  {
    currVal = it->power_mA;
    it++;
  }

  power.push_back(currVal);
  time.push_back(mStartTime);


  for (Device::powerEvent_t& powEv : evs)
  {
    if (powEv.timestamp > mEndTime)
    {
      break;
    }
    if (powEv.timestamp > mStartTime)
    {
      power.push_back(currVal);
      time.push_back(powEv.timestamp - 1);
      power.push_back(powEv.power_mA);
      time.push_back(powEv.timestamp);
    }
    currVal = powEv.power_mA;
  }
  power.push_back(currVal);
  time.push_back(min(pDev->getEnvironment()->getTimestamp(), mEndTime));

  grid(mGrid); //flakey
  axis(mStartTime, mEndTime, 0, 15);
  plot(time, power);
}