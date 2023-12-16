#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
// clang-format on
#include <Transform.hpp>

#define ZVALUE 20.0f
// 绘制坐标系
int window_x = 900;
int window_y = 900;

float canvas_x = 16.0f;
float canvas_y = 16.0f;

Eigen::Matrix3f rotate_m = Rotate2dH(0.05f * static_cast<float>(M_PI));
std::vector<Eigen::Vector3f> vertices;

void full_vertices() {
  vertices.push_back(Eigen::Vector3f(3, 3, 1));
  vertices.push_back(Eigen::Vector3f(3, 5, 1));
  vertices.push_back(Eigen::Vector3f(10, 5, 1));
  vertices.push_back(Eigen::Vector3f(10, 3, 1));
}
void rotate() {
  for (auto &vex : vertices) {
    vex = rotate_m * vex;
  }
}

// 绘制内容
void display(void) {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  rotate();

  glColor3f(0.0f, 0.0f, 1.0f); //
  glPushMatrix();
  glBegin(GL_QUADS); // GL_LINE_LOOP
  glVertex2f(vertices.at(0)(0), vertices.at(0)(1));
  glVertex2f(vertices.at(1)(0), vertices.at(1)(1));
  glVertex2f(vertices.at(2)(0), vertices.at(2)(1));
  glVertex2f(vertices.at(3)(0), vertices.at(3)(1));
  glEnd();
  glPopMatrix();
  glFlush();
}

// 投影方式、modelview方式等设置
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h); // 视口大小
  glMatrixMode(GL_PROJECTION); // 设置投影模式以及视景体大小
  glLoadIdentity();
  glOrtho(-canvas_x, canvas_x, -canvas_y, canvas_y, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
  full_vertices();
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(window_x, window_y);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("paint_line");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}