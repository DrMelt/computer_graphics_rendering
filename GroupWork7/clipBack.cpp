#define GLUT_DISABLE_ATEXIT_HACK
// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
// clang-format on
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define M_PI 3.14

// 定义一个向量结构体
typedef struct {
  float x, y, z;
  double length(Vector v) { return sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }
} Vector;

// 定义一个面结构体
typedef struct {
  Vector a, b, c;
} Face;

Vector operator-(const Vector &f1, const Vector &f2) {
  Vector result;
  result.x = f1.x - f2.x;
  result.y = f1.y - f2.y;
  result.z = f1.z - f2.z;
  return result;
}

// 计算两个向量的点积
float dotProduct(Vector v1, Vector v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

// 计算两个向量的叉积
Vector crossProduct(Vector v1, Vector v2) {
  Vector result;
  result.x = v1.y * v2.z - v1.z * v2.y;
  result.y = v1.z * v2.x - v1.x * v2.z;
  result.z = v1.x * v2.y - v1.y * v2.x;
  return result;
}

// 计算两个向量之间的夹角
float angleBetweenVectors(Vector v1, Vector v2) {
  float dot = dotProduct(v1, v2);
  float det = dotProduct(v1, crossProduct(v1, v2));
  return acos(dot / (v1.length(v1) * v2.length(v2)));
}

// 检查一个面是否在观察者的背面
int isBackfaceCulled(Face face, Vector observerPos) {
  Vector normal = crossProduct(face.b - face.a, face.c - face.a);
  float angle = angleBetweenVectors(normal, observerPos - face.a);
  return angle > M_PI / 3.0;
}

void Draw(Face face) {
  glBegin(GL_LINE_LOOP);
  glVertex3f(face.a.x * 1.0f, face.a.y * 1.0f, face.a.z * 1.0f);
  glVertex3f(face.b.x * 1.0f, face.b.y * 1.0f, face.b.z * 1.0f);
  glVertex3f(face.c.x * 1.0f, face.c.y * 1.0f, face.c.z * 1.0f);
  glEnd();
}

int main() {
  // 定义一些面
  Face faces[] = {{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}},
                  {{0, 0, 0}, {1, 1, 0}, {0, 1, 0}},
                  {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}},
                  {{0, 0, 1}, {1, 1, 1}, {0, 1, 1}}};
  Vector observerPos = {0, 0, 0}; // 观察者的位置

  // 遍历每个面，检查是否在观察者的背面
  for (int i = 0; i < 4; i++) {
    if (isBackfaceCulled(faces[i], observerPos)) {
      // printf("Face %d is backface culled\n", i);
      Draw(faces[i]);
    } else {
      // printf("Face %d is visible\n", i);
    }
  }

  return 0;
}
