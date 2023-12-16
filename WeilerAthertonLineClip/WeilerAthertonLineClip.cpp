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
#include <list>
#include <math.h>
#include <random>
#include <string.h>
using namespace std;
using namespace Eigen;

#define ZVALUE 20.0f
int window_x = 900;
int window_y = 900;

float canvas_x = 16.0f;
float canvas_y = 16.0f;

// Eigen::Matrix3f rotate_m = Rotate2dH(0.05f * static_cast<float>(M_PI));

struct Line2D {
  Line2D(const Vector3f v1, const Vector3f v2) : vertex1(v1), vertex2(v2) {}

  Vector3f GetVector() const { return vertex2 - vertex1; }

  Vector3f Sample(const float at) const { return vertex1 + GetVector() * at; }

  Vector3f Intersect(const Line2D &otherLine) const {
    auto ab = IntersectAt(otherLine);
    return vertex1 + GetVector() * ab.x();
  }
  bool IsIntersectWithRay(const Line2D ray) const {
    auto ab = IntersectAt(ray);
    if (ab.x() > 0 && ab.x() < 1 && ab.y() > 0) {
      return true;
    } else {
      return false;
    }
  }

  // return the sample on lineA and lineB
  Vector2f IntersectAt(const Line2D &otherLine) const {
    Vector3f vec1 = GetVector(), vec2 = otherLine.GetVector();
    Matrix2f m;
    m(0, 0) = vec1.x();
    m(1, 0) = vec1.y();
    m(0, 1) = -vec2.x();
    m(1, 1) = -vec2.y();
    Vector2f deltaP(otherLine.vertex1[0] - vertex1[0],
                    otherLine.vertex1[1] - vertex1[1]);
    Vector2f ab = m.inverse() * deltaP;
    return ab;
  }

  bool PointInLeft(const Vector3f pointPos) {
    Vector3f vec1 = vertex2 - vertex1, vec2 = pointPos - vertex1;
    return (vec1.cross(vec2).z() > 0);
  }

  Vector3f vertex1, vertex2;
};

class Primitive2D {
public:
  Vector3f PointPosition(const uint32_t ind) const { return points[ind]; }

  list<uint32_t>::iterator InsertPoint(const Vector3f newPoint,
                                       const uint32_t insertInd) {
    auto itor = pointInds.begin();
    advance(itor, insertInd);
    return InsertPoint(newPoint, itor);
  }
  list<uint32_t>::iterator InsertPoint(const Vector3f newPoint) {
    auto itor = pointInds.end();
    return InsertPoint(newPoint, itor);
  }
  list<uint32_t>::iterator
  InsertPoint(const Vector3f newPoint,
              const list<uint32_t>::iterator &insertItor) {
    points.push_back(newPoint);
    return pointInds.insert(insertItor, points.size() - 1);
  }

  void RemovePoint(const uint32_t insertInd = 0) {
    auto &itor = pointInds.begin();
    advance(itor, insertInd);
    RemovePoint(itor);
  }

  void RemovePoint(const list<uint32_t>::iterator &insertItor) {
    uint32_t pInd = *insertItor;
    pointInds.erase(insertItor);

    for (auto &ind : pointInds) {
      if (ind > pInd) {
        ind--;
      }
    }

    auto pItor = points.begin();
    advance(pItor, pInd);
    points.erase(pItor);
  }

  void DrawPoints() {
    glBegin(GL_LINE_STRIP); // GL_LINE_LOOP
    uint32_t originInd = *pointInds.begin();
    for (auto pointInd : pointInds) {
      glVertex2f(points[pointInd].x(), points[pointInd].y());
    }
    glVertex2f(points[originInd].x(), points[originInd].y());
    glEnd();
  }

  bool IsPointInside(const Vector3f pos) const {
    uint32_t intersectCount = 0;
    auto &itorPre = pointInds.begin();
    auto &itorNext = pointInds.begin();
    itorNext++;

    Vector3f dir(random_0_to_1(), 1, 0);
    Line2D lineDir(pos, pos + dir);

    while (itorPre != pointInds.end()) {
      Line2D line(points[*itorPre], points[*itorNext]);
      if (line.IsIntersectWithRay(lineDir)) {
        intersectCount++;
      }

      itorPre++;
      itorNext++;
      if (itorNext == pointInds.end()) {
        itorNext = pointInds.begin();
      }
    }

    if (intersectCount % 2 != 0) {
      return true;
    } else {
      return false;
    }
  }

  struct IntersectPoint {
    Vector3f pos;
    uint32_t clippedLineInd;
    float clippedAt;
    uint32_t clipLineInd;
    float clipAt;

    static bool CompareClipAt(const IntersectPoint &p1,
                              const IntersectPoint &p2) {
      return p1.clipAt < p2.clipAt;
    }
    static bool CompareClippedAt(const IntersectPoint &p1,
                                 const IntersectPoint &p2) {
      return p1.clippedAt < p2.clippedAt;
    }
  };

  /*
    遍历被裁减边
        遍历裁减多边形边
         找到交点 记录信息：坐标，在原来的哪条边上，位置
    重构结构
    建立被裁减结果
  */
  static vector<Primitive2D> WALineClip(const Primitive2D beClippedPrimitive,
                                        const Primitive2D clipPrimitive) {

    vector<IntersectPoint> intersectPoints;

    // Get Intersect Point
    auto clippedPrimitiveItorPre = beClippedPrimitive.pointInds.begin();
    auto clippedPrimitiveItorNext = beClippedPrimitive.pointInds.begin();
    clippedPrimitiveItorNext++;
    uint32_t clippedInd = 0;
    while (clippedPrimitiveItorPre != beClippedPrimitive.pointInds.end()) {
      Line2D line1(beClippedPrimitive.points[*clippedPrimitiveItorPre],
                   beClippedPrimitive.points[*clippedPrimitiveItorNext]);

      auto clipPTmpItorPre = clipPrimitive.pointInds.begin();
      auto clipPTmpItorNext = clipPrimitive.pointInds.begin();
      clipPTmpItorNext++;
      uint32_t clipInd = 0;

      while (clipPTmpItorPre != clipPrimitive.pointInds.end()) {
        Line2D line2(clipPrimitive.points[*clipPTmpItorPre],
                     clipPrimitive.points[*clipPTmpItorNext]);

        // is itersected
        auto ab = line1.IntersectAt(line2);
        if (ab.x() > 0 && ab.x() < 1) {
          if (ab.y() > 0 && ab.y() < 1) {
            IntersectPoint intersectPoint;
            intersectPoint.pos = line1.Sample(ab.x());
            intersectPoint.clippedLineInd = clippedInd;
            intersectPoint.clippedAt = ab.x();
            intersectPoint.clipLineInd = clipInd;
            intersectPoint.clipAt = ab.y();

            intersectPoints.push_back(intersectPoint);
          }
        }

        clipPTmpItorPre++;
        clipPTmpItorNext++;
        if (clipPTmpItorNext == clipPrimitive.pointInds.end()) {
          clipPTmpItorNext = clipPrimitive.pointInds.begin();
        }
        clipInd++;
      }

      clippedPrimitiveItorPre++;
      clippedPrimitiveItorNext++;
      if (clippedPrimitiveItorNext == beClippedPrimitive.pointInds.end()) {
        clippedPrimitiveItorNext = beClippedPrimitive.pointInds.begin();
      }
      clippedInd++;
    }

    // if no itersect point
    if (intersectPoints.size() == 0) {
      vector<Primitive2D> resultPrimitives;
      if (clipPrimitive.IsPointInside(beClippedPrimitive.points[0])) {
        resultPrimitives.push_back(beClippedPrimitive);
      }
      return resultPrimitives;
    }

    // rebuild prim
    vector<Vector3f> entryPos;
    vector<Vector3f> outPos;
    // rebiud clipped
    vector<Vector3f> clippedPoints;
    uint32_t clippedLineCount = 0;
    for (auto &pInd : beClippedPrimitive.pointInds) {
      clippedPoints.push_back(beClippedPrimitive.points[pInd]);
      vector<IntersectPoint> pointsOnLine;
      for (auto &iterP : intersectPoints) {
        if (iterP.clippedLineInd == clippedLineCount) {
          pointsOnLine.push_back(iterP);
        }
      }
      sort(pointsOnLine.begin(), pointsOnLine.end(),
           IntersectPoint::CompareClippedAt);
      for (auto &iterP : pointsOnLine) {
        clippedPoints.push_back(iterP.pos);
      }

      clippedLineCount++;
    }

    // is entry or out
    for (uint32_t ind = 0; ind < clippedPoints.size(); ind++) {
      uint32_t ind_pre =
                   (ind - 1 + clippedPoints.size()) % clippedPoints.size(),
               ind_next = (ind + 1) % clippedPoints.size();
      Vector3f preMidPos = (clippedPoints[ind] + clippedPoints[ind_pre]) * 0.5f,
               nextMidPos =
                   (clippedPoints[ind] + clippedPoints[ind_next]) * 0.5f;
      bool preInClip = clipPrimitive.IsPointInside(preMidPos),
           nextInClip = clipPrimitive.IsPointInside(nextMidPos);
      if (preInClip && !nextInClip) {
        outPos.push_back(clippedPoints[ind]);
      } else if (!preInClip && nextInClip) {
        entryPos.push_back(clippedPoints[ind]);
      }
    }

    // rebiud clip
    vector<Vector3f> clipPoints;
    uint32_t clipLineCount = 0;
    for (auto &pInd : clipPrimitive.pointInds) {
      clipPoints.push_back(clipPrimitive.points[pInd]);
      vector<IntersectPoint> pointsOnLine;
      for (auto &iterP : intersectPoints) {
        if (iterP.clipLineInd == clipLineCount) {
          pointsOnLine.push_back(iterP);
        }
      }
      sort(pointsOnLine.begin(), pointsOnLine.end(),
           IntersectPoint::CompareClipAt);
      for (auto &iterP : pointsOnLine) {
        clipPoints.push_back(iterP.pos);
      }

      clipLineCount++;
    }

    //////////////Test//////////////
    // glColor3f(0.5f, 0.3f, 1.0f);
    // glBegin(GL_LINE_STRIP); // GL_LINE_LOOP
    // for (auto &pointPos : clippedPoints) {
    //   glVertex2f(pointPos.x(), pointPos.y());
    // }
    // glVertex2f(clippedPoints[0].x(), clippedPoints[0].y());
    // glEnd();
    // glColor3f(1.0f, 0.3f, 0.5f);
    // glBegin(GL_LINE_STRIP); // GL_LINE_LOOP
    // for (auto &pointPos : clipPoints) {
    //   glVertex2f(pointPos.x(), pointPos.y());
    // }
    // glVertex2f(clipPoints[0].x(), clipPoints[0].y());
    // glEnd();
    // return vector<Primitive>();
    /////////////TestEnd///////////

    // Build clipped primitives
    vector<Primitive2D> resultPrimitives;
    vector<uint32_t> usedEntryPos;

    for (auto &pos : entryPos) {
      // find a entry point
      uint32_t ind = 0;
      for (ind = 0; ind < clippedPoints.size(); ind++) {
        // not be used
        if (find(usedEntryPos.begin(), usedEntryPos.end(), ind) ==
            usedEntryPos.end())
          // is a entry point
          if (clippedPoints[ind].isApprox(pos)) {
            usedEntryPos.push_back(ind);
            break;
          }
      }
      // if no unused entry point
      if (ind > clippedPoints.size() - 1) {
        break;
      }

      Primitive2D newPrim;
      bool isOnClipped = true;
      Vector3f startPos = clippedPoints[ind], currentPos = clippedPoints[ind];
      do {
        newPrim.InsertPoint(currentPos);

        if (isOnClipped) {
          ind = (ind + 1) % clippedPoints.size();
          currentPos = clippedPoints[ind];
          for (auto &outP : outPos) {
            // is out point
            if (outP.isApprox(currentPos)) {
              // find out point on clip
              for (uint32_t clipPInd = 0; clipPInd < clipPoints.size();
                   clipPInd++) {
                if (outP.isApprox(clipPoints[clipPInd])) {
                  ind = clipPInd;
                  isOnClipped = false;

                  break;
                }
              }

              break;
            }
          }
        }
        // on clip
        else {
          ind = (ind + 1) % clipPoints.size();
          currentPos = clipPoints[ind];
          for (auto &entryP : entryPos) {
            // is out point
            if (entryP.isApprox(currentPos)) {
              // find out point on clip
              for (uint32_t clippedPInd = 0; clippedPInd < clippedPoints.size();
                   clippedPInd++) {
                if (entryP.isApprox(clippedPoints[clippedPInd])) {
                  ind = clippedPInd;
                  usedEntryPos.push_back(ind);
                  isOnClipped = true;

                  break;
                }
              }

              break;
            }
          }
        }

      } while (!startPos.isApprox(currentPos));

      resultPrimitives.push_back(newPrim);
    }

    return resultPrimitives;
  }

protected:
  list<uint32_t> pointInds;
  vector<Vector3f> points;
};

Primitive2D primitive1, primitive2;
vector<Primitive2D> primsReturn;

void InitPrimives() {
  primitive1.InsertPoint(Vector3f(3, 3, 1));
  primitive1.InsertPoint(Vector3f(14, 3, 1));
  primitive1.InsertPoint(Vector3f(6, 6, 1));
  primitive1.InsertPoint(Vector3f(14, 8, 1));
  primitive1.InsertPoint(Vector3f(8, 11, 1));
  primitive1.InsertPoint(Vector3f(3, 14, 1));

  // primitive2.InsertPoint(Vector3f(0, 0, 1));
  // primitive2.InsertPoint(Vector3f(canvas_x, 0, 1));
  // primitive2.InsertPoint(Vector3f(canvas_x, canvas_y, 1));
  // primitive2.InsertPoint(Vector3f(0, canvas_y, 1));

  primitive2.InsertPoint(Vector3f(2, 2, 1));
  primitive2.InsertPoint(Vector3f(13, 2, 1));
  primitive2.InsertPoint(Vector3f(13, 13, 1));
  primitive2.InsertPoint(Vector3f(2, 13, 1));
}

void TestIntersect() {
  Line2D line1(Vector3f(10, 10, 1), Vector3f(20, 10, 1));
  Line2D line2(Vector3f(15, 10, 1), Vector3f(15, 20, 1));

  auto pos = line1.Intersect(line2);
  cout << pos << endl;
}

void PaintPrims() {
  glLineWidth(2.5f);
  glColor3f(0.5f, 0.3f, 0.3f);
  for (auto &prim : primsReturn) {
    glColor3f(random_0_to_1(), random_0_to_1(), random_0_to_1());
    prim.DrawPoints();
  }
  glLineWidth(1.0f);
  glPushAttrib(GL_ENABLE_BIT);
  glLineStipple(1, 0xAAAA);
  glEnable(GL_LINE_STIPPLE);
  glColor3f(0.5f, 0.3f, 1.0f);
  primitive1.DrawPoints();
  glColor3f(1.0f, 0.3f, 0.3f);
  primitive2.DrawPoints();
  glPopAttrib();
}

// 绘制内容
void display(void) {
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  glPushMatrix();

  // primsReturn = Primitive::WALineClip(primitive1, primitive2); // test
  PaintPrims();

  glPopMatrix();
  glFlush();
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
  // TestIntersect();
  InitPrimives();
  primsReturn = Primitive2D::WALineClip(primitive1, primitive2);

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