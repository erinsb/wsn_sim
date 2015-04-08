#include "PowerPlotter.h"
#include "Radio.h"
#include <vector>
#include <Windows.h>

#define MOVE_DELTA      (20)
#define MOVE_DELTA_FAST (100)

static void runGLUT(PowerPlotter* pPlotter);
static PowerPlotter* g_pPlotter;

static void display(void)
{
  g_pPlotter->display();
}

static void reshape(int w, int h)
{
  g_pPlotter->reshape(w, h);
}

static void mouse(int button, int state, int x, int y)
{
  g_pPlotter->mouse(button, state, x, y);
}

static void motion(int x, int y)
{
  g_pPlotter->motion(x, y);
}

static void passivemotion(int x, int y)
{
  g_pPlotter->passivemotion(x, y);
}

static void specialKeys(int key, int x, int y)
{
  g_pPlotter->specialKeys(key, x, y);
}

static void keyboard(unsigned char key, int x, int y)
{
  if (key == 'g')
    g_pPlotter->toggleGrid();
  g_pPlotter->keyboard(key, x, y);
}

static void idle(void)
{ 
  glutPostRedisplay(); 
  Sleep(10); 
}


static void runGLUT(PowerPlotter* pPlotter)
{
  g_pPlotter = pPlotter;
  int argc = 1;
  char* argv[] = { "whatever" };
  glutInit(&argc, argv);
  glutCreateWindow(800, 100, 1024, 800);
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
  timestamp_t delta = mEndTime - mStartTime;
  timestamp_t maxTime = mDevices[0]->getEnvironment()->getTimestamp();
  int movement_speed = (glutGetModifiers() & GLUT_ACTIVE_CTRL)? mDelta : (glutGetModifiers() & GLUT_ACTIVE_SHIFT) ? MOVE_DELTA_FAST : MOVE_DELTA;
  if (!(glutGetModifiers() & GLUT_ACTIVE_CTRL))
    movement_speed *= delta / 1000.0;
  timestamp_t tempStartTime = mStartTime;
  timestamp_t tempEndTime = mEndTime;
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
    if (maxTime - tempEndTime < delta)
    {
      tempStartTime = maxTime - delta;
      tempEndTime = maxTime;
    }
    else
    {
      if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
        delta *= 10;
      
      tempStartTime += delta;
      tempEndTime += delta;
    }
    break;
  case GLUT_KEY_PAGE_DOWN:
    if (tempStartTime < delta)
    {
      tempStartTime = 0;
      tempEndTime = delta;
    }
    else
    {
      if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
        delta *= 10;
      tempStartTime -= delta;
      tempEndTime -= delta;
    }
    break;
  case GLUT_KEY_HOME:
    tempStartTime = 0;
    tempEndTime = delta;
    break;
  case GLUT_KEY_END:
    tempStartTime = maxTime - delta;
    tempEndTime = maxTime;
    break;
  case GLUT_KEY_UP:
    if (tempStartTime < (uint32_t) movement_speed)
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
    if (delta > 100)
    {
      tempStartTime += movement_speed;
      tempEndTime -= movement_speed;
    }
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

void PowerPlotter::displayGraph(timestamp_t startTime, timestamp_t endTime, Device* pDev)
{
  if (pDev != NULL)
    mDevices.push_back(pDev);
  mStartTime = startTime;
  mEndTime = endTime;
  mDelta = endTime - startTime;
  if (mDevices.size() > 0)
    runGLUT(this);
}

void PowerPlotter::DISPLAY(void)
{
  for (uint32_t i = 0; i < mDevices.size(); ++i)
  {
    subplot(mDevices.size(), 1, i + 1);
    this->ylabel(mDevices[i]->mName);
    plotDevice(mDevices[i], i);
    if (i == 0)
      ticklabel(1);
    else
      ticklabel(0);

  }
}

void PowerPlotter::plotDevice(Device* pDev, uint32_t index)
{
  auto evs = pDev->getPowerUsageEvents();
  mStartTime = max(0, mStartTime);
  mEndTime = min(pDev->getEnvironment()->getTimestamp(), mEndTime);
  dvec power;
  dvec time;

  double currVal = 0.0;
  auto it = evs.begin() + mStartOffsets[index];
  while (it->timestamp > mStartTime)
  {
    it = evs.begin() + (--mStartOffsets[index]);
    currVal = it->power_mA;
  }

  while (it != evs.end() && it->timestamp <= mStartTime) // find last point before start
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
  if (time.size() > 1)
    plot(time, power);

  dvec txPower, rxPower;
  dvec timeStartEnd;

  txPower.push_back(RADIO_POWER_TX);
  txPower.push_back(RADIO_POWER_TX);

  rxPower.push_back(RADIO_POWER_RX);
  rxPower.push_back(RADIO_POWER_RX);
  timeStartEnd.push_back(mStartTime);
  timeStartEnd.push_back(mEndTime);

  plot(timeStartEnd, txPower);
  set(":");
  set("r");
  plot(timeStartEnd, rxPower);
  set(":");
  set("g");
}