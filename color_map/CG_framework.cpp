#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <GL/glut.h>
// clang-format on

#define ZVALUE 20.0f
int window_x = 900;
int window_y = 900;
// 绘制内容
void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
  glBegin(GL_POINTS); // GL_LINE_LOOP
  float r = 0.0f, g = 0.0f, b = 0.0f;
  for (int i = 0; i < window_x; ++i) {
    for (int j = 0; j < window_y; ++j) {
      r = float(i) / window_x;
      g = float(j) / window_y;

      glColor3f(r, g, b);
      glVertex2i(i, j);
    }
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
  if (w <= h)
    glOrtho(0, window_x, 0, window_y, -ZVALUE, ZVALUE);
  else
    glOrtho(-16.0 * (GLfloat)h / (GLfloat)w, 16.0 * (GLfloat)h / (GLfloat)w,
            -16.0, 16.0, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(window_x, window_y);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("color_map");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}
