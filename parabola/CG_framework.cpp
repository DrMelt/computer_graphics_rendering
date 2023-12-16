#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <GL/glut.h>
#include <GL/gl.h>
#include <random>
#include <cstdlib>
#include <Eigen/Geometry>
// clang-format on
#define M_PI 3.14159265358979323846f // pi

float random_0_to_1() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  return dis(gen);
}

#define ZVALUE 20.0f
int window_x = 1000;
int window_y = 800;
float canvas_x = 20.0f;
float canvas_y = 16.0f;
float scale_x = window_x / canvas_x;
float scale_y = window_y / canvas_y;

struct M_Parbola {
  Eigen::Vector2f center_pos;
  float a_coefficient;
  float rotation_radian;
  Eigen::Matrix2f rotate;
  Eigen::Matrix2f rotate_inverse;
  Eigen::Matrix2f rotate_dir;
  Eigen::Matrix2f rotate_dir2;

  M_Parbola(float _center_pos_x = 0.0f, float _center_pos_y = 0.0f,
            float _a_coefficient = 1.0f, float _rotation_radian = 0.0f) {
    center_pos[0] = _center_pos_x;
    center_pos[1] = _center_pos_y;
    a_coefficient = _a_coefficient;
    rotation_radian = _rotation_radian;
    rotate << cosf(rotation_radian), -sinf(rotation_radian),
        sinf(rotation_radian), cosf(rotation_radian);
    rotate_inverse << cosf(rotation_radian), sinf(rotation_radian),
        -sinf(rotation_radian), cosf(rotation_radian);
    rotate_dir << cosf(rotation_radian + M_PI * 0.5f),
        -sinf(rotation_radian + M_PI * 0.5f),
        sinf(rotation_radian + M_PI * 0.5f),
        cosf(rotation_radian + M_PI * 0.5f);
    rotate_dir2 << cosf(rotation_radian + M_PI * -0.5f),
        -sinf(rotation_radian + M_PI * -0.5f),
        sinf(rotation_radian + M_PI * -0.5f),
        cosf(rotation_radian + M_PI * -0.5f);
  }
};

void paint_parbola(M_Parbola &parbola) {
  Eigen::Vector2f current_pos = parbola.center_pos;
  Eigen::Vector2f locel_pos;

  // one side
  int while_time_count = 0;
  while (true) {
    locel_pos = parbola.rotate_inverse * (current_pos - parbola.center_pos);
    // TODO:correct the pos

    // paint pixel
    glVertex2f(current_pos.x(), current_pos.y());
    locel_pos = parbola.rotate_inverse * (current_pos - parbola.center_pos);

    Eigen::Vector2f director =
        parbola.rotate_dir *
        Eigen::Vector2f(locel_pos.x() * parbola.a_coefficient, -0.5f);

    float normalize_scale =
        sqrtf(director.x() * director.x() * scale_x * scale_x +
              director.y() * director.y() * scale_y * scale_y);
    Eigen::Vector2f step_dir_v = director / normalize_scale;
    current_pos += step_dir_v;

    while_time_count++;
    if (current_pos.x() > canvas_x || current_pos.y() > canvas_y ||
        current_pos.x() < 0 || current_pos.y() < 0) {
      break;
    } else if (while_time_count > 10000) {
      break;
    }
  }
  // other side
  current_pos = parbola.center_pos;
  while_time_count = 0;
  while (true) {
    locel_pos = parbola.rotate_inverse * (current_pos - parbola.center_pos);
    // TODO:correct the pos

    // paint pixel
    glVertex2f(current_pos.x(), current_pos.y());
    locel_pos = parbola.rotate_inverse * (current_pos - parbola.center_pos);

    Eigen::Vector2f director =
        parbola.rotate_dir2 *
        Eigen::Vector2f(locel_pos.x() * parbola.a_coefficient, -0.5f);

    float normalize_scale =
        sqrtf(director.x() * director.x() * scale_x * scale_x +
              director.y() * director.y() * scale_y * scale_y);
    Eigen::Vector2f step_dir_v = director / normalize_scale;
    current_pos += step_dir_v;

    while_time_count++;
    if (current_pos.x() > canvas_x || current_pos.y() > canvas_y ||
        current_pos.x() < 0 || current_pos.y() < 0) {
      break;
    } else if (while_time_count > 10000) {
      break;
    }
  }
}

void display(void) {
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();
  glBegin(GL_POINTS); // GL_LINE_LOOP

  glColor3f(1.0f, 1.0f, 1.0f);

  M_Parbola parbola(10.0f, 8.0f, 0.2f, M_PI * 0.5f);
  paint_parbola(parbola);

  glEnd();
  glPopMatrix();
  glFlush();
}

void reshape(int w, int h) {
  glViewport(0, 0, (GLsizei)w, (GLsizei)h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, canvas_x, 0, canvas_y, -ZVALUE, ZVALUE);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

int main(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
  glutInitWindowSize(window_x, window_y);
  glutInitWindowPosition(50, 50);
  glutCreateWindow("paint_parbola");

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}
