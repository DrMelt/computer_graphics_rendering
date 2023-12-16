#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#include <System3DHead.hpp>
#include <Transform.hpp>
#include <iostream>
#include <math.h>
#include <random>
#include <string.h>
using namespace std;
using namespace Eigen;
#define ZVALUE 1000.0f

int nearplane_width = 1;      // 视景体宽度
int nearplane_height = 1;     // 视景体高度
int nearplane_distance = 1;   // 视景体近平面与视点距离
int farplane_distance = 3000; // 视景体远平面与视点距离

int window_x = 900;
int window_y = 900;

float canvas_x = 1.0f;
float canvas_y = 1.0f;

float deltaPrePixel = canvas_x / window_x;

Vector3f eyePos = {0, 50, 5};
Vector3f eyeUp = {0, 0, 1};
Vector3f lookAtPos = {0, 0, 0};

const float rotateSpeed = 0.1;

Geometry geo, geoBSpline;

void Init() {
  auto system = System3D::InitSystem();
  system->_SetWindowSize(window_x, window_y);
  system->activeCamera = new Camera;
  system->activeCamera->eyePos = eyePos;
  system->activeCamera->eyeUp = eyeUp;
  system->activeCamera->lookAtPos = lookAtPos;

  // Bezier
  System3D::GetSystem()->_ColorStateSet(Vector4f(0.8f, 0.5f, 0.5f, 1.0f));

  geo.AddPoint(Vector3f(-20, 0, 10));
  geo.AddPoint(Vector3f(-10, 0, -20));
  geo.AddPoint(Vector3f(0, 10, 0));
  geo.AddPoint(Vector3f(20, -10, 10));
  geo.AddPoint(Vector3f(20, -10, 10));
  geo.AddPoint(Vector3f(20, -10, -20));
  geo.AddPoint(Vector3f(10, -10, -20));

  Bezier *bezier = new Bezier;
  bezier->PushBack(geo.GetPoint(0));
  bezier->PushBack(geo.GetPoint(1));
  bezier->PushBack(geo.GetPoint(2));
  bezier->PushBack(geo.GetPoint(3));
  bezier->PushBack(geo.GetPoint(4));
  bezier->PushBack(geo.GetPoint(5));
  bezier->PushBack(geo.GetPoint(6));

  geo.AddPrimitive(bezier);

  // B Spline
  System3D::GetSystem()->_ColorStateSet(Vector4f(0.5f, 0.8f, 0.5f, 1.0f));

  geoBSpline.AddPoint(Vector3f(-20, 0, 10));
  geoBSpline.AddPoint(Vector3f(-10, 0, -20));
  geoBSpline.AddPoint(Vector3f(0, 10, 0));
  geoBSpline.AddPoint(Vector3f(20, -10, 10));
  geoBSpline.AddPoint(Vector3f(20, -10, 10));
  geoBSpline.AddPoint(Vector3f(20, -10, -20));
  geoBSpline.AddPoint(Vector3f(10, -10, -20));

  BSpline *bSpline = new BSpline;
  bSpline->PushBack(geoBSpline.GetPoint(0));
  bSpline->PushBack(geoBSpline.GetPoint(1));
  bSpline->PushBack(geoBSpline.GetPoint(2));
  bSpline->PushBack(geoBSpline.GetPoint(3));
  bSpline->PushBack(geoBSpline.GetPoint(4));
  bSpline->PushBack(geoBSpline.GetPoint(5));
  bSpline->PushBack(geoBSpline.GetPoint(6));

  geoBSpline.AddPrimitive(bSpline);
}

// 绘制内容
void display(void) {

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  System3D::ClearBuffer();

  // System3D::DrawTrianglesInRange(Vector2f(0.1f, 0.9f), Vector2f(0.1f, 0.9f));

  // Draw Splines
  System3D::GetSystem()->_ColorStateSet(Vector4f(0.3f, 0.8f, 1.0f, 1.0f));
  System3D::ClearRef();
  geo.PushPrimsToSystem();
  System3D::DrawSplines();
  geo.DrawPointsToBuffer();

  System3D::GetSystem()->_ColorStateSet(Vector4f(1.0f, 0.8f, 0.3f, 1.0f));
  System3D::ClearRef();
  geoBSpline.PushPrimsToSystem();
  System3D::DrawSplines();
  geoBSpline.DrawPointsToBuffer();

  System3D::ShowBuffer();

  glFlush();
}

// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {

  case 'a':
  case 'A': {
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    glutPostRedisplay();
    break;

  case 27:
    exit(0);
    break;
  }
  }
}

// 投影方式、modelview方式等设置
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h); // 视口大小
  glMatrixMode(GL_PROJECTION); // 设置投影模式以及视景体大小
  glLoadIdentity();
  glOrtho(0, 1, 0, 1, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {

  Init();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(window_x, window_y);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("Spline");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}