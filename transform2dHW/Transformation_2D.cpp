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
#include <string.h>

#define ZVALUE 20.0f

int w_width = 600;  // 视口、窗口的宽度
int w_height = 600; // 视口、窗口的高度

float canvas_x = 600.0f;
float canvas_y = 600.0f;

float scale_x = w_width / canvas_x;
float scale_y = w_height / canvas_y;

float random_0_to_1() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);
  return dis(gen);
}

// 自定义欧氏平面点和向量
struct my_v_inhomogeneous {
  float x;
  float y;
};

// 齐次坐标下的点和向量
struct my_v_homogeneous {
  float x;
  float y;
  float ratio;
};

// 多边形顶点坐标
// 每条边都是直线
struct my_v_inhomogeneous rectangle[4];

// 绘制坐标系
void draw_coordinate() {
  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0); // 设置接下来显示的颜色
  glVertex2f(static_cast<GLfloat>(w_width), 0.0);
  glVertex2f(-static_cast<GLfloat>(w_width), 0.0);

  glColor3f(0.0, 1.0, 0.0);
  glVertex2f(0.0, static_cast<GLfloat>(w_width));
  glVertex2f(0.0, -static_cast<GLfloat>(w_width));
  glEnd();
}

struct M_Ellipse {
  Eigen::Vector2f center_pos;
  float x_len, x_len_2_inverse;
  float y_len, y_len_2_inverse;
  float rotation_radian;
  Eigen::Matrix2f rotate;
  Eigen::Matrix2f rotate_inverse;
  Eigen::Matrix2f rotate_dir;

  M_Ellipse(float _center_pos_x = 0.0f, float _center_pos_y = 0.0f,
            float _x_len = 1.0f, float _y_len = 1.0f,
            float _rotation_radian = 0.0f) {
    center_pos[0] = _center_pos_x;
    center_pos[1] = _center_pos_y;
    x_len = _x_len;
    x_len_2_inverse = 1.0f / (x_len * x_len);
    y_len = _y_len;
    y_len_2_inverse = 1.0f / (y_len * y_len);
    rotation_radian = _rotation_radian;
    rotate << cosf(rotation_radian), -sinf(rotation_radian),
        sinf(rotation_radian), cosf(rotation_radian);
    rotate_inverse << cosf(rotation_radian), sinf(rotation_radian),
        -sinf(rotation_radian), cosf(rotation_radian);
    rotate_dir << cosf(rotation_radian + static_cast<float>(M_PI) * 0.5f),
        -sinf(rotation_radian + static_cast<float>(M_PI) * 0.5f),
        sinf(rotation_radian + static_cast<float>(M_PI) * 0.5f),
        cosf(rotation_radian + static_cast<float>(M_PI) * 0.5f);
  }
};

void paint_ellipse(M_Ellipse &ellipse) {

  Eigen::Vector2f origin_pos =
      ellipse.center_pos + ellipse.rotate * Eigen::Vector2f(ellipse.x_len, 0);
  glVertex2f(origin_pos.x(), origin_pos.y());
  Eigen::Vector2i origin_pos_painted(origin_pos.x() * scale_x,
                                     origin_pos.y() * scale_y);
  Eigen::Vector2f current_pos = origin_pos;
  Eigen::Vector2f locel_pos;

  int while_time_count = 0;
  glBegin(GL_POINTS);
  while (true) {
    locel_pos = ellipse.rotate_inverse * (current_pos - ellipse.center_pos);
    // correct the pos
    current_pos = ellipse.center_pos +
                  (current_pos - ellipse.center_pos) /
                      (ellipse.x_len_2_inverse * locel_pos.x() * locel_pos.x() +
                       ellipse.y_len_2_inverse * locel_pos.y() * locel_pos.y());
    // paint pixel
    glVertex2f(current_pos.x(), current_pos.y());
    locel_pos = ellipse.rotate_inverse * (current_pos - ellipse.center_pos);

    Eigen::Vector2f director =
        ellipse.rotate_dir *
        Eigen::Vector2f(locel_pos.x() / (ellipse.x_len * ellipse.x_len),
                        locel_pos.y() / (ellipse.y_len * ellipse.y_len));

    float normalize_scale =
        sqrtf(director.x() * director.x() * scale_x * scale_x +
              director.y() * director.y() * scale_y * scale_y);
    Eigen::Vector2f step_dir_v = director / normalize_scale;
    current_pos += step_dir_v;

    while_time_count++;
    if (int(current_pos.x() * scale_x) == origin_pos_painted.x() &&
        int(current_pos.y() * scale_y) == origin_pos_painted.y()) {
      break;
    } else if (while_time_count > 10000) {
      break;
    }
  }
  glEnd();
}
struct Line2D {
  float point1_pos[2];
  float point2_pos[2];

  Line2D(const float p1x, const float p1y, const float p2x, const float p2y) {
    point1_pos[0] = p1x;
    point1_pos[1] = p1y;
    point2_pos[0] = p2x;
    point2_pos[1] = p2y;
  }
  Line2D() {
    point1_pos[0] = random_0_to_1() * canvas_x;
    point1_pos[1] = random_0_to_1() * canvas_y;
    point2_pos[0] = random_0_to_1() * canvas_x;
    point2_pos[1] = random_0_to_1() * canvas_y;
  }
};

Line2D lines[4];

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
    step_times = (int)(w_width / canvas_x * max_len);
  } else {
    step_times = (int)(w_height / canvas_y * max_len);
  }
  float step_dir_v[] = {dir_vector[0] / step_times, dir_vector[1] / step_times};
  float pos_x = line.point1_pos[0], pos_y = line.point1_pos[1];
  glBegin(GL_POINTS); // GL_LINE_LOOP
  for (int i = 0; i < step_times; ++i) {
    glVertex2f(pos_x, pos_y);
    pos_x += step_dir_v[0];
    pos_y += step_dir_v[1];
  }
  glEnd();
}

// 初始化正方形
void init(void) {
  rectangle[0].x = 0;
  rectangle[0].y = 0;

  rectangle[1].x = 160;
  rectangle[1].y = 0;

  rectangle[2].x = 160;
  rectangle[2].y = 80;

  rectangle[3].x = 0;
  rectangle[3].y = 80;
}

void flash_lines() {
  lines[0] =
      Line2D(rectangle[0].x, rectangle[0].y, rectangle[1].x, rectangle[1].y);
  lines[1] =
      Line2D(rectangle[1].x, rectangle[1].y, rectangle[2].x, rectangle[2].y);
  lines[2] =
      Line2D(rectangle[2].x, rectangle[2].y, rectangle[3].x, rectangle[3].y);
  lines[3] =
      Line2D(rectangle[3].x, rectangle[3].y, rectangle[0].x, rectangle[0].y);
}

void DrawLines() {
  for (int i = 0; i < 4; ++i) {
    paint_line(lines[i]);
  }
}

// 绘制内容
void display(void) {
  glClearColor(1.f, 1.f, 1.f, 0.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  draw_coordinate(); // 绘制坐标系

  glColor3f(157.0 / 256, 195.0 / 256, 230.0 / 256);

  flash_lines();
  DrawLines();

  M_Ellipse ellipse(rectangle[2].x, rectangle[2].y, 50.0f, 50.0f, M_PI * 0.0f);
  paint_ellipse(ellipse); // draw circle

  glutSwapBuffers();
}

//////////////////////////////////////////////非齐次平移变换-所需函数//////////////////////////////////////////////
void my_traslate_inhomogeneous(struct my_v_inhomogeneous *polygon,
                               int polygon_vertex_count, int tx, int ty) {
  for (int vIndex = 0; vIndex < polygon_vertex_count; vIndex++) {
    polygon[vIndex].x += tx;
    polygon[vIndex].y += ty;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////齐次平移变换-所需函数//////////////////////////////////////////////
// 矩阵向量相乘
struct my_v_homogeneous
matrix_multiply_vector(float matrix[][3], struct my_v_homogeneous input_v) {
  struct my_v_homogeneous translated_v;
  translated_v.x =
      matrix[0][0] * input_v.x + matrix[0][1] * input_v.y + matrix[0][2] * 1;
  translated_v.y =
      matrix[1][0] * input_v.x + matrix[1][1] * input_v.y + matrix[1][2] * 1;
  translated_v.ratio =
      matrix[2][0] * input_v.x + matrix[2][1] * input_v.y + matrix[2][2] * 1;
  return translated_v;
}

// 齐次平移变换
void my_traslate_homogeneous(struct my_v_inhomogeneous *polygon,
                             int polygon_vertex_count, float tx, float ty) {
  // 装配生成平移矩阵
  float translate_matrix[3][3];
  memset(translate_matrix, 0, sizeof(int) * 9);
  translate_matrix[0][0] = translate_matrix[1][1] = translate_matrix[2][2] =
      1; // 对角线赋值为1
  translate_matrix[0][2] = tx;
  translate_matrix[1][2] = ty;

  // 遍历并平移多边形每个顶点
  for (int vIndex = 0; vIndex < polygon_vertex_count; vIndex++) {
    struct my_v_homogeneous input_v;
    input_v.x = polygon[vIndex].x;
    input_v.y = polygon[vIndex].y;
    input_v.ratio = 1;
    input_v = matrix_multiply_vector(translate_matrix,
                                     input_v); // 平移矩阵作用到每个顶点
    polygon[vIndex].x = input_v.x;
    polygon[vIndex].y = input_v.y;
  }
}
// 齐次旋转变换
void my_rotate_homogeneous(struct my_v_inhomogeneous *polygon,
                           int polygon_vertex_count, float radian) {
  // 装配生成矩阵
  float translate_matrix[3][3] = {};
  memset(translate_matrix, 0, sizeof(int) * 9);
  translate_matrix[0][0] = cosf(radian);
  translate_matrix[0][1] = -sinf(radian);
  translate_matrix[1][0] = sinf(radian);
  translate_matrix[1][1] = cosf(radian);
  translate_matrix[2][2] = 1.0f;

  // 遍历多边形每个顶点
  for (int vIndex = 0; vIndex < polygon_vertex_count; vIndex++) {
    struct my_v_homogeneous input_v;
    input_v.x = polygon[vIndex].x;
    input_v.y = polygon[vIndex].y;
    input_v.ratio = 1;
    input_v = matrix_multiply_vector(translate_matrix,
                                     input_v); // 平移矩阵作用到每个顶点
    polygon[vIndex].x = input_v.x;
    polygon[vIndex].y = input_v.y;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////
// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case 'a':
  case 'A': {
    // 原点旋转
    my_rotate_homogeneous(rectangle, 4, 0.1f * static_cast<float>(M_PI));
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    // P1旋转
    float originX = rectangle[2].x, originY = rectangle[2].y;
    my_traslate_homogeneous(rectangle, 4, -originX, -originY);
    my_rotate_homogeneous(rectangle, 4, 0.1f * static_cast<float>(M_PI));
    my_traslate_homogeneous(rectangle, 4, originX, originY);
    glutPostRedisplay();
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
  /*if (w <= h)
          glOrtho(-16.0, 16.0,
  -16.0*(GLfloat)h/(GLfloat)w, 16.0*(GLfloat)h/(GLfloat)w, -ZVALUE, ZVALUE);
  else
          glOrtho(-16.0*(GLfloat)h/(GLfloat)w, 16.0*(GLfloat)h/(GLfloat)w,
  -16.0, 16.0, -ZVALUE, ZVALUE);*/

  if (w <= h)
    glOrtho(-0.5 * canvas_x, 0.5 * canvas_x,
            -0.5 * canvas_y * (GLfloat)canvas_y / (GLfloat)canvas_x,
            0.5 * canvas_y * (GLfloat)canvas_y / (GLfloat)canvas_x, -ZVALUE,
            ZVALUE);
  else
    glOrtho(-0.5 * canvas_x, 0.5 * canvas_x,
            -0.5 * canvas_y * (GLfloat)canvas_x / (GLfloat)canvas_y,
            0.5 * canvas_y * (GLfloat)canvas_x / (GLfloat)canvas_y, -ZVALUE,
            ZVALUE);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

// 主调函数
int main(int argc, char **argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("平移变换");

  init();

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMainLoop();
  return 0;
}
