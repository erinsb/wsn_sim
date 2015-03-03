#include "WSNplotter.h"

WSNplotter* g_pPlotter = NULL;

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
  //g_pPlotter->specialKeys(key, x, y);
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


void runGLUT(WSNplotter* pPlotter)
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


void WSNplotter::toggleGrid(void)
{
  mGrid = !mGrid;
  glutPostRedisplay();
}