#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
#include <stack>
#include <string>

#define ZVALUE 20.0f

using namespace std;
using namespace Eigen;

int w_width = 600;  // 视口、窗口的宽度
int w_height = 600; // 视口、窗口的高度

float canvas_x = 600.0f, canvas_y = 600.0f;

const int buffer_x = 20, buffer_y = 20;

float buffer_scale_x = buffer_x / canvas_x,
      buffer_scale_y = buffer_y / canvas_y;

float scale_x = w_width / canvas_x;
float scale_y = w_height / canvas_y;

float fillError = 0.05f;

Vector3f buffer[buffer_x][buffer_y] = {};

float random_0_to_1() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  return dis(gen);
}

struct Line2D {
  Vector2f point1_pos;
  Vector2f point2_pos;
};

void DrawLineToBuffer(const Line2D &line, const Vector3f color) {
  float dir_vector[2] = {line.point2_pos[0] - line.point1_pos[0],
                         line.point2_pos[1] - line.point1_pos[1]};
  bool max_is_x = true;
  float max_len = 0.0f;
  if (abs(dir_vector[0]) > abs(dir_vector[1])) {
    max_len = abs(dir_vector[0]);
  } else {
    max_len = abs(dir_vector[1]);
    max_is_x = false;
  }

  int step_times;
  if (max_is_x) {
    step_times = (int)(max_len);
  } else {
    step_times = (int)(max_len);
  }
  Vector2f step_dir_v = {dir_vector[0] / step_times,
                         dir_vector[1] / step_times};
  Vector2f pos = line.point1_pos;
  for (int i = 0; i < step_times; ++i) {
    buffer[static_cast<int>(pos.x())][static_cast<int>(pos.y())] = color;
    pos += step_dir_v;
  }
}

void InitBuffer() {
  Vector3f color(0.2, 0.3, 0.5);
  Line2D line1 = {Vector2f(1, 1), Vector2f(15, 3)};
  DrawLineToBuffer(line1, color);
  Line2D line2 = {Vector2f(15, 3), Vector2f(12, 12)};
  DrawLineToBuffer(line2, color);
  Line2D line3 = {Vector2f(12, 12), Vector2f(2, 17)};
  DrawLineToBuffer(line3, color);
  Line2D line4 = {Vector2f(2, 17), Vector2f(5, 10)};
  DrawLineToBuffer(line4, color);
  Line2D line5 = {Vector2f(5, 10), Vector2f(1, 1)};
  DrawLineToBuffer(line5, color);
}

void DrawBuffer() {
  glBegin(GL_POINTS); // GL_LINE_LOOP

  float delta_x = canvas_x / w_width, delta_y = canvas_y / w_height;

  float current_x = 0;
  int count_x = 0;
  while (count_x < w_width) {
    float current_y = 0;
    int count_y = 0;
    while (count_y < w_height) {
      int buffer_x = static_cast<int>(current_x * buffer_scale_x);
      int buffer_y = static_cast<int>(current_y * buffer_scale_y);
      auto &pixel = buffer[buffer_x][buffer_y];
      glColor3f(pixel.x(), pixel.y(), pixel.z());
      glVertex2f(current_x, current_y);
      current_y += delta_y;
      count_y++;
    }
    current_x += delta_x;
    count_x++;
  }
  glEnd();
}

struct Seed {
  Vector2i startPos;
  Vector3f color;

  void SeedFill() {
    bool filled[buffer_x][buffer_y] = {};
    stack<Vector2i> seedStack;
    seedStack.push(startPos);
    Vector3f startColor = buffer[startPos.x()][startPos.y()];
    while (seedStack.size() > 0) {
      auto seed = seedStack.top();
      seedStack.pop();
      buffer[seed.x()][seed.y()] = color;
      filled[seed.x()][seed.y()] = true;

      // L
      auto nextSeed = seed;
      if (nextSeed.x() > 0) {
        nextSeed.x() -= 1;
        if (!filled[nextSeed.x()][nextSeed.y()]) {
          if ((buffer[nextSeed.x()][nextSeed.y()] - startColor).norm() <
              fillError) {
            seedStack.push(nextSeed);
          }
        }
      }
      // R
      nextSeed = seed;
      if (nextSeed.x() < buffer_x - 1) {
        nextSeed.x() += 1;
        if (!filled[nextSeed.x()][nextSeed.y()]) {
          if ((buffer[nextSeed.x()][nextSeed.y()] - startColor).norm() <
              fillError) {
            seedStack.push(nextSeed);
          }
        }
      }
      // U
      nextSeed = seed;
      if (nextSeed.y() < buffer_y - 1) {
        nextSeed.y() += 1;
        if (!filled[nextSeed.x()][nextSeed.y()]) {
          if ((buffer[nextSeed.x()][nextSeed.y()] - startColor).norm() <
              fillError) {
            seedStack.push(nextSeed);
          }
        }
      }
      // D
      nextSeed = seed;
      if (nextSeed.y() > 0) {
        nextSeed.y() -= 1;
        if (!filled[nextSeed.x()][nextSeed.y()]) {
          if ((buffer[nextSeed.x()][nextSeed.y()] - startColor).norm() <
              fillError) {
            seedStack.push(nextSeed);
          }
        }
      }
    }
  }
};

// 绘制内容
void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 0.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glColor3f(157.0 / 256, 195.0 / 256, 230.0 / 256);
  DrawBuffer();

  glutSwapBuffers();
}

// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case 'a':
  case 'A': {
    break;
  }
  case 'd':
  case 'D': {
    break;
  }
  case 27:
    exit(0);
    break;
  }
}

// 投影方式、modelview方式设置
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(0, canvas_x, 0, canvas_y * (GLfloat)canvas_y / (GLfloat)canvas_x,
          -ZVALUE, ZVALUE);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
  InitBuffer();
  Seed seed;
  seed.color = Vector3f(0.8, 0.7, 0.9);
  seed.startPos = Vector2i(6, 3);
  seed.SeedFill();

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("平移变换");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}
