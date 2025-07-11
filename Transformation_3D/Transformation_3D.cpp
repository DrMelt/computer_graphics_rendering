#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#include <iostream>
#include <math.h>
#include <random>
#include <string.h>
using namespace std;
using namespace Eigen;

int nearplane_width = 1;      // 视景体宽度
int nearplane_height = 1;     // 视景体高度
int nearplane_distance = 1;   // 视景体近平面与视点距离
int farplane_distance = 3000; // 视景体远平面与视点距离
int w_width = 600;            // 视口、窗口的宽度
int w_height = 600;           // 视口、窗口的高度
float eye_x = 100;

// 齐次坐标下的点和向量
struct my_v_homogeneous {
  float x;
  float y;
  float z;
  float ratio;
};

// box顶点坐标
// 每条边都是直线
struct my_v_homogeneous box[8];

// 初始化正方形顶点坐标
void init(void) {
  // 前面四个顶点
  box[0].x = 0;
  box[0].y = 0;
  box[0].z = 0;
  box[0].ratio = 1;

  box[1].x = 80;
  box[1].y = 0;
  box[1].z = 0;
  box[1].ratio = 1;

  box[2].x = 80;
  box[2].y = 40;
  box[2].z = 0;
  box[2].ratio = 1;

  box[3].x = 0;
  box[3].y = 40;
  box[3].z = 0;
  box[3].ratio = 1;

  // 后面四个顶点
  box[4].x = 0;
  box[4].y = 0;
  box[4].z = -50;
  box[4].ratio = 1;

  box[5].x = 80;
  box[5].y = 0;
  box[5].z = -50;
  box[5].ratio = 1;

  box[6].x = 80;
  box[6].y = 40;
  box[6].z = -50;
  box[6].ratio = 1;

  box[7].x = 0;
  box[7].y = 40;
  box[7].z = -50;
  box[7].ratio = 1;
}

// 绘制坐标系
void draw_coordinate() {
  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0); // 红色x轴
  glVertex3f(100, 0.0, 0.0);
  glVertex3f(-100, 0.0, 0.0);

  glColor3f(0.0, 1.0, 0.0); // 绿色y轴
  glVertex3f(0.0, 100, 0.0);
  glVertex3f(0.0, -100, 0.0);

  glColor3f(0.0, 0.0, 1.0); // 兰色z轴
  glVertex3f(0.0, 0.0, 100);
  glVertex3f(0.0, 0.0, -100);
  glEnd();
}

// 绘制内容
void display(void) {
  glClearColor(1.f, 1.f, 1.f, 0.f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  draw_coordinate(); // 绘制坐标系

  glColor3f(157.0 / 256, 195.0 / 256, 230.0 / 256);

  // 绘制BOX,默认顶点之间通过直线段相连
  glBegin(GL_LINES); // GL_LINE_LOOP
                     // 前面4条边
  glVertex3i((floor)(box[0].x + 0.5), (floor)(box[0].y + 0.5),
             (floor)(box[0].z + 0.5));
  glVertex3i((floor)(box[1].x + 0.5), (floor)(box[1].y + 0.5),
             (floor)(box[1].z + 0.5));

  glVertex3i((floor)(box[1].x + 0.5), (floor)(box[1].y + 0.5),
             (floor)(box[1].z + 0.5));
  glVertex3i((floor)(box[2].x + 0.5), (floor)(box[2].y + 0.5),
             (floor)(box[2].z + 0.5));

  glVertex3i((floor)(box[2].x + 0.5), (floor)(box[2].y + 0.5),
             (floor)(box[2].z + 0.5));
  glVertex3i((floor)(box[3].x + 0.5), (floor)(box[3].y + 0.5),
             (floor)(box[3].z + 0.5));

  glVertex3i((floor)(box[3].x + 0.5), (floor)(box[3].y + 0.5),
             (floor)(box[3].z + 0.5));
  glVertex3i((floor)(box[0].x + 0.5), (floor)(box[0].y + 0.5),
             (floor)(box[0].z + 0.5));

  // 后面4条边
  glVertex3i((floor)(box[4].x + 0.5), (floor)(box[4].y + 0.5),
             (floor)(box[4].z + 0.5));
  glVertex3i((floor)(box[5].x + 0.5), (floor)(box[5].y + 0.5),
             (floor)(box[5].z + 0.5));

  glVertex3i((floor)(box[5].x + 0.5), (floor)(box[5].y + 0.5),
             (floor)(box[5].z + 0.5));
  glVertex3i((floor)(box[6].x + 0.5), (floor)(box[6].y + 0.5),
             (floor)(box[6].z + 0.5));

  glVertex3i((floor)(box[6].x + 0.5), (floor)(box[6].y + 0.5),
             (floor)(box[6].z + 0.5));
  glVertex3i((floor)(box[7].x + 0.5), (floor)(box[7].y + 0.5),
             (floor)(box[7].z + 0.5));

  glVertex3i((floor)(box[7].x + 0.5), (floor)(box[7].y + 0.5),
             (floor)(box[7].z + 0.5));
  glVertex3i((floor)(box[4].x + 0.5), (floor)(box[4].y + 0.5),
             (floor)(box[4].z + 0.5));
  // 左面补两条边
  glVertex3i((floor)(box[0].x + 0.5), (floor)(box[0].y + 0.5),
             (floor)(box[0].z + 0.5));
  glVertex3i((floor)(box[4].x + 0.5), (floor)(box[4].y + 0.5),
             (floor)(box[4].z + 0.5));

  glVertex3i((floor)(box[7].x + 0.5), (floor)(box[7].y + 0.5),
             (floor)(box[7].z + 0.5));
  glVertex3i((floor)(box[3].x + 0.5), (floor)(box[3].y + 0.5),
             (floor)(box[3].z + 0.5));

  // 右面补两条边
  glVertex3i((floor)(box[1].x + 0.5), (floor)(box[1].y + 0.5),
             (floor)(box[1].z + 0.5));
  glVertex3i((floor)(box[5].x + 0.5), (floor)(box[5].y + 0.5),
             (floor)(box[5].z + 0.5));

  glVertex3i((floor)(box[6].x + 0.5), (floor)(box[6].y + 0.5),
             (floor)(box[6].z + 0.5));
  glVertex3i((floor)(box[2].x + 0.5), (floor)(box[2].y + 0.5),
             (floor)(box[2].z + 0.5));
  glEnd();
  glutSwapBuffers();
}

//////////////////////////////////////////////齐次平移变换-所需函数//////////////////////////////////////////////
// 矩阵向量相乘
struct my_v_homogeneous
matrix_multiply_vector(float matrix[][4], struct my_v_homogeneous input_v) {
  struct my_v_homogeneous translated_v;
  translated_v.x = matrix[0][0] * input_v.x + matrix[0][1] * input_v.y +
                   matrix[0][2] * input_v.z + matrix[0][3] * input_v.ratio;
  translated_v.y = matrix[1][0] * input_v.x + matrix[1][1] * input_v.y +
                   matrix[1][2] * input_v.z + matrix[1][3] * input_v.ratio;
  translated_v.z = matrix[2][0] * input_v.x + matrix[2][1] * input_v.y +
                   matrix[2][2] * input_v.z + matrix[2][3] * input_v.ratio;
  translated_v.ratio = matrix[3][0] * input_v.x + matrix[3][1] * input_v.y +
                       matrix[3][2] * input_v.z + matrix[3][3] * input_v.ratio;
  return translated_v;
}

// 齐次平移变换
void my_traslate_homogeneous(struct my_v_homogeneous *polygon, int vertex_count,
                             float tx, float ty, float tz) {
  // 装配生成平移矩阵
  float translate_matrix[4][4];
  memset(translate_matrix, 0, sizeof(int) * 16);
  translate_matrix[0][0] = translate_matrix[1][1] = translate_matrix[2][2] =
      translate_matrix[3][3] = 1; // 对角线赋值为1
  translate_matrix[0][3] = tx;
  translate_matrix[1][3] = ty;
  translate_matrix[2][3] = tz;

  // 遍历并平移多边形每个顶点
  for (int vIndex = 0; vIndex < vertex_count; vIndex++) {
    struct my_v_homogeneous input_v;
    input_v.x = polygon[vIndex].x;
    input_v.y = polygon[vIndex].y;
    input_v.z = polygon[vIndex].z;
    input_v.ratio = 1;
    input_v = matrix_multiply_vector(translate_matrix,
                                     input_v); // 平移矩阵作用到每个顶点
    polygon[vIndex].x = input_v.x;
    polygon[vIndex].y = input_v.y;
    polygon[vIndex].z = input_v.z;
    polygon[vIndex].ratio = input_v.ratio;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////

const float SPEED = 1.0f;
// 键盘交互事件
void keyboard(unsigned char key, int x, int y) {
  switch (key) {
  case 'w':
  case 'W': {
    my_traslate_homogeneous(box, 8, 0, SPEED, 0); // 向Y轴正方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 's':
  case 'S': {
    my_traslate_homogeneous(box, 8, 0, -SPEED, 0); // 向Y轴负方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 'a':
  case 'A': {
    my_traslate_homogeneous(box, 8, -SPEED, 0, 0); // 向X轴负方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 'd':
  case 'D': {
    my_traslate_homogeneous(box, 8, SPEED, 0, 0); // 向X轴正方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 'n':
  case 'N': {
    my_traslate_homogeneous(box, 8, 0, 0, SPEED); // 向z轴正方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 'f':
  case 'F': {
    my_traslate_homogeneous(box, 8, 0, 0, -SPEED); // 向z轴负方向移动1个单元
    glutPostRedisplay();
    break;
  }
  case 'p':
  case 'P': {
    float mat[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, mat);
    for (int i = 0; i < 16; ++i) {
      if (i % 4 == 0) {
        cout << endl;
      }
      cout << mat[i] << " ";
    }
    cout << endl;
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

  // if (w <= h)
  //	glOrtho(-0.5 * nearplane_width, 0.5 * nearplane_width, -0.5 *
  // nearplane_height * (GLfloat)nearplane_height / (GLfloat)nearplane_width,
  // 0.5
  //* nearplane_height * (GLfloat)nearplane_height / (GLfloat)nearplane_width,
  //		-nearplane_distance, farplane_distance); //相对于视点
  // else
  //	glOrtho(-0.5 * nearplane_width, 0.5 * nearplane_width, -0.5 *
  // nearplane_height * (GLfloat)nearplane_width / (GLfloat)nearplane_height,
  // 0.5
  //* nearplane_height * (GLfloat)nearplane_width / (GLfloat)nearplane_height,
  //		-nearplane_distance, farplane_distance);

  if (w <= h)
    glFrustum(-0.5 * nearplane_width, 0.5 * nearplane_width,
              -0.5 * nearplane_height * (GLfloat)nearplane_height /
                  (GLfloat)nearplane_width,
              0.5 * nearplane_height * (GLfloat)nearplane_height /
                  (GLfloat)nearplane_width,
              nearplane_distance, farplane_distance); // 相对于视点
  else
    glFrustum(-0.5 * nearplane_width, 0.5 * nearplane_width,
              -0.5 * nearplane_height * (GLfloat)nearplane_width /
                  (GLfloat)nearplane_height,
              0.5 * nearplane_height * (GLfloat)nearplane_width /
                  (GLfloat)nearplane_height,
              nearplane_distance, farplane_distance);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(eye_x, 50, 50, 0, 0, 0, 0, 1, 0);
}

// 鼠标交互事件
void mouse(int button, int state, int x, int y) {
  switch (button) {
  case GLUT_LEFT_BUTTON:
    if (state == GLUT_DOWN) {
      eye_x += SPEED;
      reshape(w_width, w_height);
      glutPostRedisplay();
    }
    break;
  case GLUT_RIGHT_BUTTON:
    if (state == GLUT_DOWN) {
      eye_x -= SPEED;
      reshape(w_width, w_height);
      glutPostRedisplay();
    }
    break;
  default:
    break;
  }
}

Matrix4f LookAtMatrix(float eyex, float eyey, float eyez, float centerx,
                      float centery, float centerz, float upx, float upy,
                      float upz) {
  Vector3f pos = {eyex, eyey, eyez}, lookAt = {centerx, centery, centerz},
           up = {upx, upy, upz};
  lookAt = (lookAt - pos).normalized();
  Vector3f normal = lookAt.cross(up).normalized();
  up = normal.cross(lookAt);
  Matrix4f transfer = Matrix4f::Identity();
  transfer.block<3, 1>(0, 3) = -pos;
  cout << transfer << endl;
  cout << endl;

  Matrix4f rotate = Matrix4f::Zero();
  rotate.block<1, 3>(0, 0) = lookAt;
  rotate.block<1, 3>(1, 0) = up;
  rotate.block<1, 3>(2, 0) = -normal;
  rotate(3, 3) = 1.0f;
  cout << rotate << endl;
  cout << endl;

  Matrix4f lookAtMatrix = rotate * transfer;

  cout << lookAtMatrix << endl;
  cout << endl;

  return lookAtMatrix;
}

Matrix4f OrthogonalProjection(float left, float right, float bottom, float top,
                              float zNear, float zFar) {
  Matrix4f orthogonalProjection = Matrix4f::Identity();
  orthogonalProjection(2, 2) = (zFar + zNear) / (zFar - zNear);
  orthogonalProjection(2, 3) = -2 * zFar * zNear / (zFar - zNear);

  cout << orthogonalProjection << endl;
  cout << endl;
  return orthogonalProjection;
}

// 主调函数
int main(int argc, char **argv) {
  LookAtMatrix(eye_x, 50, 50, 0, 0, 0, 0, 1, 0);

  OrthogonalProjection(
      -0.5 * nearplane_width, 0.5 * nearplane_width,
      -0.5 * nearplane_height * nearplane_height / nearplane_width,
      0.5 * nearplane_height * nearplane_height / nearplane_width,
      nearplane_distance, farplane_distance);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutInitWindowSize(w_width, w_height);
  glutInitWindowPosition(100, 100);
  glutCreateWindow("几何变换");

  init();

  glutReshapeFunc(reshape);
  glutDisplayFunc(display);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMainLoop();
  return 0;
}
