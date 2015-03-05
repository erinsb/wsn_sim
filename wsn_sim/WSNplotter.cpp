#include "WSNplotter.h"

WSNplotter* g_pPlotter = NULL;

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
  //g_pPlotter->specialKeys(key, x, y);
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


static void runGLUT(WSNplotter* pPlotter)
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


WSNplotter::WSNplotter(WSN* pWSN) : m_pWSN(pWSN), mGrid(false)
{
}


WSNplotter::~WSNplotter()
{
}

void WSNplotter::DISPLAY(void)
{

}

void WSNplotter::toggleGrid(void)
{
  mGrid = !mGrid;
  glutPostRedisplay();
}