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

float deltaPrePixel = canvas_x / window_x;

Vector3f eyePos = {0, 50, 5};
Vector3f eyeUp = {0, 0, 1};
Vector3f lookAtPos = {0, 0, 0};

const float rotateSpeed = 0.1;

class AttributeBase {};

struct Point {
  Vector3f pos;
  vector<AttributeBase> attributes;
};

struct Line2D {
  Line2D(const Vector3f v1, const Vector3f v2) : vertex1(v1), vertex2(v2) {}

  void Draw() {
    glBegin(GL_POINTS);
    Vector2f dir_vector = (vertex2 - vertex1).block<2, 1>(0, 0);

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
    Vector2f pos = vertex1.block<2, 1>(0, 0);
    for (int i = 0; i < step_times; ++i) {
      glVertex2f(pos.x(), pos.y());
      pos += step_dir_v;
    }

    glEnd();
  }

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

  void DrawWithoutTrans() {
    auto &itorPre = pointInds.begin();
    auto &itorNext = pointInds.begin();
    itorNext++;

    while (itorPre != pointInds.end()) {
      Line2D line(points[*itorPre], points[*itorNext]);
      line.Draw();

      itorPre++;
      itorNext++;
      if (itorNext == pointInds.end()) {
        itorNext = pointInds.begin();
      }
    }
  }

  void DrawLines() {
    auto indItor = pointInds.begin();
    auto prePos = points[*indItor], currentPos = prePos;
    indItor++;

    while (indItor != pointInds.end()) {
      currentPos = points[*indItor];
      Line2D line(prePos, currentPos);
      line.Draw();
      prePos = currentPos;
      indItor++;
    }

    Line2D line(points[*pointInds.begin()], points[*(--pointInds.end())]);
    line.Draw();
  }

  // convex
  void Fill() {
    if (pointInds.size() < 3) {
      return;
    }

    auto minYPoint = pointInds.begin();
    auto pointItor = minYPoint;
    pointItor++;
    float yMin = points[*minYPoint].y();
    float yMax = points[*minYPoint].y();
    while (pointItor != pointInds.end()) {
      if (points[*pointItor].y() < yMin) {
        yMin = points[*pointItor].y();
        minYPoint = pointItor;
      } else if (points[*pointItor].y() > yMax) {
        yMax = points[*pointItor].y();
      }

      pointItor++;
    }

    if (yMax - yMin <= deltaPrePixel) {
      return;
    }

    struct ScanPoint {
      float nextY;
      float currentX;
      float deltaX;

      ScanPoint(const bool isLeft, list<uint32_t>::iterator startItor,
                list<uint32_t> *pointInds, vector<Vector3f> *points)
          : isLeft(isLeft), nextPItor(startItor), pointInds(pointInds),
            points(points) {
        NextLine();
      }

      bool isLeft;
      list<uint32_t>::iterator nextPItor;
      list<uint32_t> *pointInds;
      vector<Vector3f> *points;

      void MoveItor() {
        if (isLeft) {
          if (nextPItor == (*pointInds).begin()) {
            nextPItor = (*pointInds).end();
          }
          nextPItor--;
        } else {
          nextPItor++;
          if (nextPItor == (*pointInds).end()) {
            nextPItor = (*pointInds).begin();
          }
        }
      }

      void Step() { currentX += deltaX; }

      // Will chage itor
      void NextLine() {
        Vector3f p1, p2, line = Vector3f::Zero();
        while (line.y() < deltaPrePixel) {
          p1 = (*points)[*nextPItor];
          currentX = p1.x();
          MoveItor();
          p2 = (*points)[*nextPItor];
          nextY = p2.y();
          line = p2 - p1;
        }
        deltaX = line.x() / line.y() * deltaPrePixel;
      }
    };

    ScanPoint spL(true, minYPoint, &pointInds, &points),
        spR(false, minYPoint, &pointInds, &points);

    float currentY = yMin;
    currentY += deltaPrePixel;
    while (currentY < yMax) {
      spL.Step();
      if (spL.nextY < currentY) {
        spL.NextLine();
      }

      spR.Step();
      if (spR.nextY < currentY) {
        spR.NextLine();
      }

      if (spL.currentX < spR.currentX) {
        Line2D scanLine(Vector3f(spL.currentX, currentY, 1.0f),
                        Vector3f(spR.currentX, currentY, 1.0f));
        scanLine.Draw();
      }

      currentY += deltaPrePixel;
    }
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

  static vector<Primitive2D> WALineClip(const Primitive2D beClippedPrimitive,
                                        const Primitive2D clipPrimitive) {
    /*
    遍历被裁减边
        遍历裁减多边形边
         找到交点 记录信息：坐标，在原来的哪条边上，位置
    重构结构
    建立被裁减结果
    */
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

class Triangle {};

class Primitive {
public:
  void Transform(const Matrix4f &tranM) {
    for (auto &pointInd : pointInds) {
      points[pointInd] = TransformForPos(points[pointInd], tranM);
    }
  }

  void Draw(const Matrix4f &MVP, const Primitive2D &clipPrim) {
    points_trans = points;
    for (auto &point : points_trans) {
      point = TransformForPos(point, MVP);
    }

    if (pointInds.size() < 3) {
      return;
    }

    // is front
    auto itorInd = pointInds.begin();
    auto p1 = points_trans[*itorInd];
    itorInd++;
    auto p2 = points_trans[*itorInd];
    itorInd++;
    auto p3 = points_trans[*itorInd];
    auto line1 = p2 - p1;
    auto line2 = p3 - p1;
    auto crossResult = line1.cross(line2);
    if (crossResult.z() < 0) {
      return;
    }

    ////Draw Origin
    // auto itorPre = pointInds.begin();
    // auto itorNext = pointInds.begin();
    // itorNext++;
    // while (itorPre != pointInds.end()) {
    //   auto p1 = points_trans[*itorPre], p2 = points_trans[*itorNext];
    //   p1.z() = 1;
    //   p2.z() = 1;
    //   Line2D line(p1, p2);
    //   line.Draw();

    //  itorPre++;
    //  itorNext++;
    //  if (itorNext == pointInds.end()) {
    //    itorNext = pointInds.begin();
    //  }
    //}

    // Clip Draw Fill
    Primitive2D prim2d;
    for (auto &pInd : pointInds) {
      prim2d.InsertPoint(points_trans[pInd]);
    }
    auto prims = Primitive2D::WALineClip(prim2d, clipPrim);
    for (auto &prim : prims) {
      prim.DrawLines();
      prim.Fill();
    }
  }

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

protected:
  list<uint32_t> pointInds;
  vector<Vector3f> points;
  vector<Vector3f> points_trans;
};

class Geometry {
public:
  Primitive GetPrimitive(const uint32_t ind) const {
    return primitives.at(ind);
  }

  void AddPrimitive(const Primitive &prim) { primitives.push_back(prim); }

  void Draw(const Matrix4f &MVP, const Primitive2D &clipPrim) {
    for (auto &primitive : primitives) {
      primitive.Draw(MVP, clipPrim);
    }
  }

  void Transform(const Matrix4f tranM) {
    origin = TransformForPos(origin, tranM);
    originDir = TransformForVector(originDir, tranM);

    for (auto &primitive : primitives) {
      primitive.Transform(tranM);
    }
  }

  Vector3f origin;
  Vector3f originDir;

protected:
  vector<Primitive> primitives;
};

Geometry cube1, cube2;
Primitive2D clipPrim2D;

void Init() {
  clipPrim2D.InsertPoint(Vector3f(0.1, 0.1, 1));
  clipPrim2D.InsertPoint(Vector3f(0.9, 0.1, 1));
  clipPrim2D.InsertPoint(Vector3f(0.9, 0.9, 1));
  clipPrim2D.InsertPoint(Vector3f(0.1, 0.9, 1));

  // cube1
  cube1.origin = Vector3f(0, 0, 5);
  cube1.originDir = Vector3f(0, 0, 1);
  // 上面
  Primitive prim1;
  prim1.InsertPoint(Vector3f(0, 0, 10));
  prim1.InsertPoint(Vector3f(0, -10, 10));
  prim1.InsertPoint(Vector3f(-20, -10, 10));
  prim1.InsertPoint(Vector3f(-20, 0, 10));
  cube1.AddPrimitive(prim1);

  // 侧面
  Primitive prim2;
  prim2.InsertPoint(Vector3f(0, 0, 10));
  prim2.InsertPoint(Vector3f(-20, 0, 10));
  prim2.InsertPoint(Vector3f(-20, 0, 0));
  prim2.InsertPoint(Vector3f(0, 0, 0));
  cube1.AddPrimitive(prim2);

  Primitive prim3;
  prim3.InsertPoint(Vector3f(0, 0, 10));
  prim3.InsertPoint(Vector3f(0, 0, 0));
  prim3.InsertPoint(Vector3f(0, -10, 0));
  prim3.InsertPoint(Vector3f(0, -10, 10));
  cube1.AddPrimitive(prim3);

  Primitive prim4;
  prim4.InsertPoint(Vector3f(-20, -10, 10));
  prim4.InsertPoint(Vector3f(-20, -10, 0));
  prim4.InsertPoint(Vector3f(-20, 0, 0));
  prim4.InsertPoint(Vector3f(-20, 0, 10));
  cube1.AddPrimitive(prim4);

  Primitive prim5;
  prim5.InsertPoint(Vector3f(-20, -10, 10));
  prim5.InsertPoint(Vector3f(0, -10, 10));
  prim5.InsertPoint(Vector3f(0, -10, 0));
  prim5.InsertPoint(Vector3f(-20, -10, 0));
  cube1.AddPrimitive(prim5);
  // 下面
  Primitive prim6;
  prim6.InsertPoint(Vector3f(0, 0, 0));
  prim6.InsertPoint(Vector3f(-20, 0, 0));
  prim6.InsertPoint(Vector3f(-20, -10, 0));
  prim6.InsertPoint(Vector3f(0, -10, 0));
  cube1.AddPrimitive(prim6);

  // cube2
  cube2.origin = Vector3f(0, 0, 5);
  cube2.originDir = Vector3f(0, 0, 1);
  // 上面
  Primitive prim1_2;
  prim1_2.InsertPoint(Vector3f(0, 0, 10));
  prim1_2.InsertPoint(Vector3f(0, 10, 10));
  prim1_2.InsertPoint(Vector3f(20, 10, 10));
  prim1_2.InsertPoint(Vector3f(20, 0, 10));
  cube2.AddPrimitive(prim1_2);
  // 侧面
  Primitive prim2_2;
  prim2_2.InsertPoint(Vector3f(20, 0, 10));
  prim2_2.InsertPoint(Vector3f(20, 0, 0));
  prim2_2.InsertPoint(Vector3f(0, 0, 0));
  prim2_2.InsertPoint(Vector3f(0, 0, 10));
  cube2.AddPrimitive(prim2_2);

  Primitive prim3_2;
  prim3_2.InsertPoint(Vector3f(0, 0, 0));
  prim3_2.InsertPoint(Vector3f(0, 10, 0));
  prim3_2.InsertPoint(Vector3f(0, 10, 10));
  prim3_2.InsertPoint(Vector3f(0, 0, 10));
  cube2.AddPrimitive(prim3_2);

  Primitive prim4_2;
  prim4_2.InsertPoint(Vector3f(20, 10, 0));
  prim4_2.InsertPoint(Vector3f(20, 0, 0));
  prim4_2.InsertPoint(Vector3f(20, 0, 10));
  prim4_2.InsertPoint(Vector3f(20, 10, 10));
  cube2.AddPrimitive(prim4_2);

  Primitive prim5_2;
  prim5_2.InsertPoint(Vector3f(0, 10, 10));
  prim5_2.InsertPoint(Vector3f(0, 10, 0));
  prim5_2.InsertPoint(Vector3f(20, 10, 0));
  prim5_2.InsertPoint(Vector3f(20, 10, 10));
  cube2.AddPrimitive(prim5_2);
  // 下面
  Primitive prim6_2;
  prim6_2.InsertPoint(Vector3f(20, 0, 0));
  prim6_2.InsertPoint(Vector3f(20, 10, 0));
  prim6_2.InsertPoint(Vector3f(0, 10, 0));
  prim6_2.InsertPoint(Vector3f(0, 0, 0));
  cube2.AddPrimitive(prim6_2);
}

void rotateCube2() {
  Vector3f center = cube2.origin;
  Vector3f dir = cube2.originDir;

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

  // Draw ClipPrim
  glColor3f(0.3f, 0.8f, 1.0f);
  clipPrim2D.DrawLines();

  glColor3f(0.5f, 0.3f, 0.5f);
  cube1.Draw(MVP, clipPrim2D);
  cube2.Draw(MVP, clipPrim2D);

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
  Init();

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