#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <GL/glut.h>
#include <GL/gl.h>
#include <random>
#include <cstdlib>
// clang-format on

float random_0_to_1() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  return dis(gen);
}

#define ZVALUE 20.0f
int window_x = 900;
int window_y = 900;

float canvas_x = 16.0f;
float canvas_y = 16.0f;

struct Line2D {
  float point1_pos[2];
  float point2_pos[2];

  Line2D() {
    point1_pos[0] = random_0_to_1() * canvas_x;
    point1_pos[1] = random_0_to_1() * canvas_y;
    point2_pos[0] = random_0_to_1() * canvas_x;
    point2_pos[1] = random_0_to_1() * canvas_y;
  }
};

Line2D lines[1000] = {};

void paint_line(Line2D &line) {
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
    step_times = (int)(window_x / canvas_x * max_len);
  } else {
    step_times = (int)(window_y / canvas_y * max_len);
  }
  float step_dir_v[] = {dir_vector[0] / step_times, dir_vector[1] / step_times};
  float pos_x = line.point1_pos[0], pos_y = line.point1_pos[1];
  for (int i = 0; i < step_times; ++i) {
    glVertex2f(pos_x, pos_y);
    pos_x += step_dir_v[0];
    pos_y += step_dir_v[1];
  }
}

// 绘制内容
void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
  glBegin(GL_POINTS); // GL_LINE_LOOP

  glColor3f(1.0f, 1.0f, 1.0f);

  for (int i = 0; i < 1000; ++i) {
    paint_line(lines[i]);
  }

  glEnd();
  glPopMatrix();
  glFlush();
}

// 投影方式、modelview方式等设置
void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h); // 视口大小
  glMatrixMode(GL_PROJECTION); // 设置投影模式以及视景体大小
  glLoadIdentity();
  glOrtho(0, 16, 0, 16, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
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
