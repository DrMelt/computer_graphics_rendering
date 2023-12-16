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

Vector3f eyePos = {-1, 0.5, 1.5};
Vector3f eyeUp = {0, 1, 0};
Vector3f lookAtPos = {0, 0.5, 0};

const float rotateSpeed = 0.1;

Geometry *model = nullptr;
Light light;

void Init() {
  auto system = System3D::InitSystem();
  system->_SetWindowSize(window_x, window_y);
  system->activeCamera = new Camera;
  system->activeCamera->eyePos = eyePos;
  system->activeCamera->eyeUp = eyeUp;
  system->activeCamera->lookAtPos = lookAtPos;

  light.axis.origin = Vector3f(2, 2, 2);
  System3D::PushLightRef(&light);

  model = ReadOBJ("models/Miku.obj");
  model->FlipX();

  System3D::ClearRef();
  model->PushPrimsToSystem();

  System3D::BuildBVH();

  System3D::RefreshShadowMap();

  // Test
  // Vector3f origin(-0.3f, 1.0f, -0.5f);
  // cout << origin.normalized() << endl << endl;
  // auto mapped2D = MapDir2TextureInBox(origin);
  // cout << mapped2D << endl << endl;
  // auto mapped3D = MapTexture2DirInBox(mapped2D);
  // cout << mapped3D << endl << endl;
}

// 绘制内容
void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  System3D::ClearBuffer();

  System3D::DrawTrianglesInRangeMultiThread(Vector2f(0.0f, 1.0f),
                                            Vector2f(0.0f, 1.0f));

  System3D::GetSystem()->_ColorStateSet(Vector4f(0.3f, 0.8f, 1.0f, 1.0f));
  // System3D::DrawBVHsFramwork();

  // light.ShowShadowMapToBuffer(0.1f);

  System3D::ShowBuffer();

  glFlush();
}

const float cameraStep = 0.1f;
// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {

  case 'a':
  case 'A': {
    System3D::GetSystem()->activeCamera->eyePos.x() += cameraStep;
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    System3D::GetSystem()->activeCamera->eyePos.x() -= cameraStep;
    glutPostRedisplay();
    break;
  }
  case 'w':
  case 'W': {
    System3D::GetSystem()->activeCamera->eyePos.z() -= cameraStep;
    glutPostRedisplay();
    break;
  }
  case 's':
  case 'S': {
    System3D::GetSystem()->activeCamera->eyePos.z() += cameraStep;
    glutPostRedisplay();
    break;
  }
  case 'c':
  case 'C': {
    System3D::GetSystem()->activeCamera->eyePos.y() -= cameraStep;
    glutPostRedisplay();
    break;
  }
  case 32: {
    System3D::GetSystem()->activeCamera->eyePos.y() += cameraStep;
    glutPostRedisplay();
    break;
  }
  case 27: {
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

  delete model;
  return 0;
}