#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>

// clang-format on
#include <System3DHead.hpp>
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

int window_x = 600;
int window_y = 800;

Vector3f lookAtPos = {0, 1.0f, -10};
Vector3f eyePos = {0, 1.0f, 0};
Vector3f eyeUp = {0, 1, 0};

// Vector3f lookAtPos = {0, 1.0f, 0.0f};
// Vector3f eyePos = {0, 20.0f, 0};
// Vector3f eyeUp = {0, 0, -1};

Geometry *model = nullptr, *meshLight = nullptr, *extraTranslucence = nullptr;
Light light1, light2, light3;

void Init() {
  auto system = System3D::InitSystem();
  system->_SetWindowSize(window_x, window_y);
  system->activeCamera = new Camera;
  system->activeCamera->eyePos = eyePos;
  system->activeCamera->eyeUp = eyeUp;
  system->activeCamera->lookAtPos = lookAtPos;
  system->activeCamera->nearplane_width =
      static_cast<float>(window_y) / window_x;

  if constexpr (!IS_PERSPECTIVE_PROJECT) {
    system->activeCamera->nearplane_height *= 10;
    system->activeCamera->nearplane_width *= 10;
  }

  System3D::SetPixelSampleTimes(8);
  System3D::SetPixelSampleDeep({3, 20});
  System3D::SetThreads(94);

  System3D::ClearRef();

  if constexpr (!IS_RAY_TRACING) {
    light1.axis.origin = Vector3f(0, 16, -20);
    light1.intensity = Vector3f(400, 400, 400);
    light1.shadowMapHeight = 2048;

    light2.axis.origin = Vector3f(1.5f, 2, -2.5f);
    light2.intensity = Vector3f(3, 3, 3);

    light3.axis.origin = Vector3f(-1.5f, 2, -2.5f);
    light3.intensity = Vector3f(3, 3, 3);
    System3D::PushLightRef(&light1);
    System3D::PushLightRef(&light2);
    System3D::PushLightRef(&light3);
  }

  // Read models
  const string folderPath = "../../../../../models/";
  // const string folderPath = "../../models/";

  model = ReadOBJ(folderPath + "bighw/stage.obj");
  model->PushPrimsToSystem();

  // For Path Tracing
  if constexpr (IS_RAY_TRACING) {
    meshLight = ReadOBJ(folderPath + "bighw/light.obj");

    Material *lightMaterial = new Material;
    lightMaterial->diffuseColor = Vector3f::Zero();
    lightMaterial->specularColor = Vector3f::Zero();
    lightMaterial->emissionColor = Vector3f(50, 50, 50);
    meshLight->AssignMaterial(lightMaterial);

    meshLight->PushPrimsToSystem();

    extraTranslucence = ReadOBJ(folderPath + "bighw/extraTranslucence.obj");
    extraTranslucence->PushPrimsToSystem();
  }

  System3D::BuildBVH();

  // Read hdr and set
  Texture<Vector3f> hdr(0, 0);
  hdr.ReadImageAndMatchSize(folderPath + "bighw/textures/sky.png");
  system->skyBox.texture = new Texture(hdr);
  system->skyBox.emissionColor = Vector3f(0.3f, 0.3f, 0.3f);

  if constexpr (!IS_RAY_TRACING) {
    System3D::RefreshShadowMap();
  }
}

// 绘制内容
void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  System3D::ClearBuffer();

  System3D::DrawTrianglesInRangeMultiThread(Vector2f(0.0f, 1.0f),
                                            Vector2f(0.0f, 1.0f));

  if constexpr (IS_RAY_TRACING && OPEN_DENOISE) {
    System3D::DenoiseForBuffer();
  }

  System3D::ShowBuffer();

  glFlush();
}

const float cameraStep = 0.1f;
const float cameraRotationStep = 0.1f;
// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {

  case 'a':
  case 'A': {
    Vector3f stepVec = {cameraStep, 0, 0};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    Vector3f stepVec = {-cameraStep, 0, 0};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
  case 'w':
  case 'W': {
    Vector3f stepVec = {0, 0, cameraStep};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
  case 's':
  case 'S': {
    Vector3f stepVec = {0, 0, -cameraStep};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
    // down
  case 'c':
  case 'C': {
    Vector3f stepVec = {0, -cameraStep, 0};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
    // up
  case 32: {
    Vector3f stepVec = {0, cameraStep, 0};
    auto viewM = System3D::GetV().inverse();
    stepVec = TransformForVector(stepVec, viewM);

    System3D::GetSystem()->activeCamera->eyePos += stepVec;
    System3D::GetSystem()->activeCamera->lookAtPos += stepVec;
    glutPostRedisplay();
    break;
  }
    // rotation
  case 'q':
  case 'Q': {
    auto rotateM = Rotate3dH(System3D::GetSystem()->activeCamera->eyeUp,
                             -cameraRotationStep);

    Vector3f dir = System3D::GetSystem()->activeCamera->lookAtPos -
                   System3D::GetSystem()->activeCamera->eyePos;
    dir = TransformForVector(dir, rotateM);

    System3D::GetSystem()->activeCamera->lookAtPos =
        System3D::GetSystem()->activeCamera->eyePos + dir;
    glutPostRedisplay();
    break;
  }
    // rotation
  case 'e':
  case 'E': {
    auto rotateM = Rotate3dH(System3D::GetSystem()->activeCamera->eyeUp,
                             cameraRotationStep);
    Vector3f dir = System3D::GetSystem()->activeCamera->lookAtPos -
                   System3D::GetSystem()->activeCamera->eyePos;
    dir = TransformForVector(dir, rotateM);

    System3D::GetSystem()->activeCamera->lookAtPos =
        System3D::GetSystem()->activeCamera->eyePos + dir;
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
  glutInitWindowSize(window_y, window_x);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("paint");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();

  delete model;
  return 0;
}