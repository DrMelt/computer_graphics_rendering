#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#include <Transform.hpp>
#include <iostream>
#include <math.h>
#include <random>
#include <string.h>
using namespace std;
using namespace Eigen;
#define ZVALUE 10.0f

int nearplane_width = 1;      // 视景体宽度
int nearplane_height = 1;     // 视景体高度
int nearplane_distance = 1;   // 视景体近平面与视点距离
int farplane_distance = 3000; // 视景体远平面与视点距离

int window_x = 900;
int window_y = 900;

float canvas_x = 1.0f;
float canvas_y = 1.0f;

Vector3f eyePos = {0, -80, 0};
Vector3f eyeUp = {0, 0, 1};
Vector3f lookAtPos = {0, 0, 0};

const float rotateSpeed = 0.1;

struct Line2D {
  uint32_t vind0;
  uint32_t vind1;
  vector<Vector4f> *verticesSet;

  Line2D(const uint32_t vind0, const uint32_t vind1,
         vector<Vector4f> *verticesSet)
      : vind0(vind0), vind1(vind1), verticesSet(verticesSet) {}
  Line2D(const Line2D &line)
      : vind0(line.vind0), vind1(line.vind1), verticesSet(line.verticesSet) {}

  void Tranfrom(const Matrix4f tranM) {
    verticesSet->at(vind0) = tranM * verticesSet->at(vind0);
    verticesSet->at(vind1) = tranM * verticesSet->at(vind1);

    verticesSet->at(vind0) /= (verticesSet->at(vind0)).w();
    verticesSet->at(vind1) /= (verticesSet->at(vind0)).w();
  }

  void DrawLine(const Matrix4f MVP) const {
    Vector4f tranP0 = MVP * verticesSet->at(vind0),
             tranP1 = MVP * verticesSet->at(vind1);

    tranP0 /= tranP0.w();
    tranP1 /= tranP1.w();

    Vector2f dir_vector = (tranP1 - tranP0).block<2, 1>(0, 0);

    bool max_is_x = true;
    float max_len = 0.0f;
    if (abs(dir_vector[0]) > abs(dir_vector[1])) {
      max_len = abs(dir_vector[0]);
    } else {
      max_len = abs(dir_vector[1]);
      max_is_x = false;
    }

    uint32_t step_times;
    if (max_is_x) {
      step_times = (uint32_t)(window_x / canvas_x * max_len);
    } else {
      step_times = (uint32_t)(window_y / canvas_y * max_len);
    }
    Vector2f step_dir_v = dir_vector / step_times;
    float pos_x = tranP0[0], pos_y = tranP0[1];
    for (int i = 0; i < step_times; ++i) {
      glVertex2f(pos_x, pos_y);
      pos_x += step_dir_v[0];
      pos_y += step_dir_v[1];
    }
  }
};

class Geometry {
public:
  Geometry(vector<Vector4f> *verticesSet) : verticesSet(verticesSet) {}

  Line2D GetLine(const uint32_t ind) const { return lines.at(ind); }

  void AddLine(const Line2D &line) { lines.push_back(line); }

  vector<uint32_t> VerticesInd() const {
    vector<uint32_t> vertices_inds;
    for (auto &line : lines) {
      auto it =
          std::find(vertices_inds.begin(), vertices_inds.end(), line.vind0);
      if (it == vertices_inds.end()) {
        vertices_inds.push_back(line.vind0);
      }

      it = std::find(vertices_inds.begin(), vertices_inds.end(), line.vind1);
      if (it == vertices_inds.end()) {
        vertices_inds.push_back(line.vind1);
      }
    }
    return vertices_inds;
  }

  void Draw(const Matrix4f MVP) const {
    for (auto &line : lines) {
      line.DrawLine(MVP);
    }
  }

  void Transform(const Matrix4f tranM) {
    vector<uint32_t> verticesInd = VerticesInd();
    for (auto &vInd : verticesInd) {
      verticesSet->at(vInd) = tranM * verticesSet->at(vInd);
    }
  }

protected:
  vector<Line2D> lines;
  vector<Vector4f> *verticesSet;
};

vector<Vector4f> vertices;

Geometry cube1(&vertices);
Geometry cube2(&vertices);

void initVertices() {
  vertices.push_back(Vector4f(0, 0, 10, 1));     // 0
  vertices.push_back(Vector4f(0, -10, 10, 1));   // 1
  vertices.push_back(Vector4f(-20, -10, 10, 1)); // 2
  vertices.push_back(Vector4f(-20, 0, 10, 1));   // 3
  vertices.push_back(Vector4f(0, 0, 0, 1));      // 4
  vertices.push_back(Vector4f(0, -10, 0, 1));    // 5
  vertices.push_back(Vector4f(-20, -10, 0, 1));  // 6
  vertices.push_back(Vector4f(-20, 0, 0, 1));    // 7

  vertices.push_back(Vector4f(0, 0, 10, 1));   // 8
  vertices.push_back(Vector4f(0, 10, 10, 1));  // 9
  vertices.push_back(Vector4f(20, 10, 10, 1)); // 10
  vertices.push_back(Vector4f(20, 0, 10, 1));  // 11
  vertices.push_back(Vector4f(0, 0, 0, 1));    // 12
  vertices.push_back(Vector4f(0, 10, 0, 1));   // 13
  vertices.push_back(Vector4f(20, 10, 0, 1));  // 14
  vertices.push_back(Vector4f(20, 0, 0, 1));   // 15
}

void initCubes() {
  // 上面
  cube1.AddLine(Line2D(0, 1, &vertices));
  cube1.AddLine(Line2D(1, 2, &vertices));
  cube1.AddLine(Line2D(2, 3, &vertices));
  cube1.AddLine(Line2D(3, 0, &vertices));
  // 侧面
  cube1.AddLine(Line2D(0, 4, &vertices)); // 共用线
  cube1.AddLine(Line2D(1, 5, &vertices));
  cube1.AddLine(Line2D(2, 6, &vertices));
  cube1.AddLine(Line2D(3, 7, &vertices));
  // 下面
  cube1.AddLine(Line2D(4, 5, &vertices));
  cube1.AddLine(Line2D(5, 6, &vertices));
  cube1.AddLine(Line2D(6, 7, &vertices));
  cube1.AddLine(Line2D(7, 4, &vertices));
  // 上面
  cube2.AddLine(Line2D(8, 9, &vertices));
  cube2.AddLine(Line2D(9, 10, &vertices));
  cube2.AddLine(Line2D(10, 11, &vertices));
  cube2.AddLine(Line2D(11, 8, &vertices));
  // 侧面
  cube2.AddLine(Line2D(8, 12, &vertices)); // 共用线
  cube2.AddLine(Line2D(9, 13, &vertices));
  cube2.AddLine(Line2D(10, 14, &vertices));
  cube2.AddLine(Line2D(11, 15, &vertices));
  // 下面
  cube2.AddLine(Line2D(12, 13, &vertices));
  cube2.AddLine(Line2D(13, 14, &vertices));
  cube2.AddLine(Line2D(14, 15, &vertices));
  cube2.AddLine(Line2D(15, 12, &vertices));
}

void rotateCube2() {
  Line2D line = cube2.GetLine(4);
  auto p0 = (line.verticesSet->at(line.vind0)).block<3, 1>(0, 0);
  auto p1 = (line.verticesSet->at(line.vind1)).block<3, 1>(0, 0);
  Vector3f center = (p0 + p1) * 0.5f;
  Vector3f dir = (p1 - p0);

  auto rotateM = Rotate3dH(dir, rotateSpeed);
  Matrix4f trans = Translate3dH(-center);
  Matrix4f trans_invert = Translate3dH(center);

  Matrix4f transformM = trans_invert * rotateM * trans;

  cube2.Transform(transformM);
}

void rotateAroundAxis() {
  Vector3f center = {20, 20, 20};
  Vector3f dir = {1, 1, 1};

  auto rotateM = Rotate3dH(Vector3f(1, 1, 1), rotateSpeed);
  Matrix4f trans = Translate3dH(-center);
  Matrix4f trans_invert = Translate3dH(center);

  Matrix4f transformM = trans_invert * rotateM * trans;

  cube1.Transform(transformM);
  cube2.Transform(transformM);
}

// 绘制内容
void display(void) {
  auto MVP =
      PerspectiveProjection(-0.5f * nearplane_width, 0.5f * nearplane_width,
                            -0.5f * nearplane_height, 0.5f * nearplane_height,
                            static_cast<float>(nearplane_distance),
                            static_cast<float>(farplane_distance)) *
      LookAtMatrix(eyePos, lookAtPos, eyeUp);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glColor3f(0.3f, 0.0f, 1.0f);
  glBegin(GL_POINTS);

  cube1.Draw(MVP);
  cube2.Draw(MVP);
  glEnd();
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
  glOrtho(0, canvas_x, 0, canvas_y, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
  initVertices();

  initCubes();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(window_x, window_y);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("paint_line");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}