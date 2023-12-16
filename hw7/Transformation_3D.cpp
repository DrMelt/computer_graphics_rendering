#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#include <DataStruct.hpp>
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

Geometry cube1, cube2;
Primitive2D clipPrim2D;

void Init() {
  auto system = System3D::InitSystem();
  system->canvas_x = canvas_x;
  system->canvas_y = canvas_y;
  system->window_x = window_x;
  system->window_y = window_y;
  system->eyePos = eyePos;
  system->eyeUp = eyeUp;
  system->lookAtPos = lookAtPos;

  clipPrim2D.InsertPoint(Vector2f(0.1f, 0.1f));
  clipPrim2D.InsertPoint(Vector2f(0.9f, 0.1f));
  clipPrim2D.InsertPoint(Vector2f(0.9f, 0.9f));
  clipPrim2D.InsertPoint(Vector2f(0.1f, 0.9f));

  Triangle *tria;
  System3D::GetSystem()->colorState = Vector4f(0.5f, 0.3f, 0.5f, 1.0f);
  // cube1
  Axis axis(Vector3f(0, 0, 5), Vector3f(1, 0, 0), Vector3f(0, 0, 1));
  cube1.axis = axis;

  cube1.AddPoint(Vector3f(0, 0, 10));     // 0
  cube1.AddPoint(Vector3f(0, -10, 10));   // 1
  cube1.AddPoint(Vector3f(-20, -10, 10)); // 2
  cube1.AddPoint(Vector3f(-20, 0, 10));   // 3
  cube1.AddPoint(Vector3f(0, 0, 0));      // 4
  cube1.AddPoint(Vector3f(0, -10, 0));    // 5
  cube1.AddPoint(Vector3f(-20, -10, 0));  // 6
  cube1.AddPoint(Vector3f(-20, 0, 0));    // 7

  // 上面
  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(1), cube1.GetPoint(2));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(2), cube1.GetPoint(3));
  cube1.AddPrimitive(tria);

  // 侧面
  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(3), cube1.GetPoint(7));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(7), cube1.GetPoint(4));
  cube1.AddPrimitive(tria);

  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(4), cube1.GetPoint(5));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(0), cube1.GetPoint(5), cube1.GetPoint(1));
  cube1.AddPrimitive(tria);

  tria = new Triangle(cube1.GetPoint(2), cube1.GetPoint(6), cube1.GetPoint(7));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(2), cube1.GetPoint(7), cube1.GetPoint(3));
  cube1.AddPrimitive(tria);

  tria = new Triangle(cube1.GetPoint(2), cube1.GetPoint(1), cube1.GetPoint(5));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(2), cube1.GetPoint(5), cube1.GetPoint(6));
  cube1.AddPrimitive(tria);

  // 下面
  tria = new Triangle(cube1.GetPoint(4), cube1.GetPoint(7), cube1.GetPoint(6));
  cube1.AddPrimitive(tria);
  tria = new Triangle(cube1.GetPoint(4), cube1.GetPoint(6), cube1.GetPoint(5));
  cube1.AddPrimitive(tria);

  // cube2
  System3D::GetSystem()->colorState = Vector4f(0.5f, 0.5f, 0.3f, 1.0f);
  cube2.axis = axis;

  cube2.AddPoint(Vector3f(0, 0, 10));   // 0
  cube2.AddPoint(Vector3f(0, 10, 10));  // 1
  cube2.AddPoint(Vector3f(20, 10, 10)); // 2
  cube2.AddPoint(Vector3f(20, 0, 10));  // 3
  cube2.AddPoint(Vector3f(0, 0, 0));    // 4
  cube2.AddPoint(Vector3f(0, 10, 0));   // 5
  cube2.AddPoint(Vector3f(20, 10, 0));  // 6
  cube2.AddPoint(Vector3f(20, 0, 0));   // 7
  // 上面
  tria = new Triangle(cube2.GetPoint(0), cube2.GetPoint(1), cube2.GetPoint(2));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(0), cube2.GetPoint(2), cube2.GetPoint(3));
  cube2.AddPrimitive(tria);

  // 侧面
  tria = new Triangle(cube2.GetPoint(3), cube2.GetPoint(7), cube2.GetPoint(4));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(3), cube2.GetPoint(4), cube2.GetPoint(0));
  cube2.AddPrimitive(tria);

  tria = new Triangle(cube2.GetPoint(4), cube2.GetPoint(5), cube2.GetPoint(1));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(4), cube2.GetPoint(1), cube2.GetPoint(0));
  cube2.AddPrimitive(tria);

  tria = new Triangle(cube2.GetPoint(6), cube2.GetPoint(7), cube2.GetPoint(3));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(6), cube2.GetPoint(3), cube2.GetPoint(2));
  cube2.AddPrimitive(tria);

  tria = new Triangle(cube2.GetPoint(1), cube2.GetPoint(5), cube2.GetPoint(6));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(1), cube2.GetPoint(6), cube2.GetPoint(2));
  cube2.AddPrimitive(tria);

  // 下面
  tria = new Triangle(cube2.GetPoint(7), cube2.GetPoint(6), cube2.GetPoint(5));
  cube2.AddPrimitive(tria);
  tria = new Triangle(cube2.GetPoint(7), cube2.GetPoint(5), cube2.GetPoint(4));
  cube2.AddPrimitive(tria);
}

void rotateCube2() {
  Vector3f center = cube2.axis.origin;
  Vector3f dir = cube2.axis.originZ;

  auto rotateM = Rotate3dH(dir, rotateSpeed);
  Matrix4f trans = Translate3dH(-center);
  Matrix4f trans_invert = Translate3dH(center);

  Matrix4f transformM = trans_invert * rotateM * trans;

  cube2.TransformWorld(transformM);
}

void rotateAroundAxis() {
  Vector3f center = {20, 20, 20};
  Vector3f dir = {1, 1, 1};

  auto rotateM = Rotate3dH(Vector3f(1, 1, 1), rotateSpeed);
  Matrix4f trans = Translate3dH(-center);
  Matrix4f trans_invert = Translate3dH(center);

  Matrix4f transformM = trans_invert * rotateM * trans;

  cube1.TransformWorld(transformM);
  cube2.TransformWorld(transformM);
}

// 绘制内容
void display(void) {
  // auto MVP =
  //     PerspectiveProjection(-0.5f * nearplane_width, 0.5f * nearplane_width,
  //                           -0.5f * nearplane_height, 0.5f *
  //                           nearplane_height,
  //                           static_cast<float>(nearplane_distance),
  //                           static_cast<float>(farplane_distance)) *
  //     LookAtMatrix(eyePos, lookAtPos, eyeUp);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  System3D::ClearBuffer();
  System3D::ClearTriangleRef();

  cube1.PushPrimsToSystem();
  cube2.PushPrimsToSystem();



  // System3D::GetSystem()->colorState = Vector4f(0.5f, 0.3f, 0.5f, 1.0f);
  // cube1.Draw(clipPrim2D);
  // System3D::GetSystem()->colorState = Vector4f(0.5f, 0.5f, 0.3f, 1.0f);
  // cube2.Draw(clipPrim2D);

  // System3D::DrawTrianglesInClip(clipPrim2D);
  System3D::DrawTrianglesInRange(Vector2f(0.1f, 0.9f), Vector2f(0.1f, 0.9f));

  // Draw ClipPrim
  System3D::GetSystem()->colorState = Vector4f(0.3f, 0.8f, 1.0f, 1.0f);
  clipPrim2D.DrawLines();

  System3D::ShowBuffer();

  glFlush();
}

// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {

  case 'a':
  case 'A': {
    rotateCube2();
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    rotateAroundAxis();
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
  glutCreateWindow("paint");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}