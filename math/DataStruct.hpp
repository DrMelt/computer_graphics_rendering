#ifndef _DATA_STRUCT_HPP_
#define _DATA_STRUCT_HPP_

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
#include <stack>
#include <string>

using namespace std;
using namespace Eigen;

struct Axis {
  Vector3f origin = Vector3f(0.0f, 0.0f, 0.0f); // Axis position
  Vector3f originX = Vector3f(1.0f, 0.0f, 0.0f);
  Vector3f originY = Vector3f(0.0f, 1.0f, 0.0f);
  Vector3f originZ = Vector3f(0.0f, 0.0f, 1.0f); // Axis z dir

  Axis() = default;
  Axis(const Vector3f &origin,
       const Vector3f &originX = Vector3f(1.0f, 0.0f, 0.0f),
       const Vector3f &originZ = Vector3f(0.0f, 0.0f, 1.0f))
      : origin(origin), originX(originX.normalized()),
        originZ(originZ.normalized()), originY(originZ.cross(originX)) {}

  void Transform(const Matrix4f tranM) {
    origin = TransformForPos(origin, tranM);
    originX = TransformForVector(originX, tranM);
    originY = TransformForVector(originY, tranM);
    originZ = TransformForVector(originZ, tranM);
  }
};

template <class T> struct Attribute {
  Attribute() = default;
  Attribute(const string &name) : name(name) {}
  ~Attribute() = default;

  bool IsName(const string &name) const { return this->name == name; }
  string Name() const { return name; }

  T value;

protected:
  string name;
};

struct AttributeSet {
  Vector3f color = {1.0f, 1.0f, 1.0f};
  Vector3f normal;

  vector<Attribute<Vector3f>> vec3fAttributes;
  vector<Attribute<float>> floatAttributes;

  vector<Attribute<int>> intAttributes; // not be varying

  AttributeSet()
      : color((System3D::GetSystem()->_ColorState()).block<3, 1>(0, 0)) {}
  virtual ~AttributeSet() = default;

  virtual AttributeSet Average(const AttributeSet &other) const {
    AttributeSet newAttr(*this);
    newAttr += other;
    newAttr *= 0.5f;
    return newAttr;
  }

  static AttributeSet Average(const vector<AttributeSet *> &attributeSets) {
    AttributeSet newAttr(*attributeSets[0]);
    for (uint32_t ind = 1; ind < attributeSets.size(); ind++) {
      newAttr += *attributeSets[ind];
    }
    newAttr *= 1.0f / attributeSets.size();
    return newAttr;
  }

  virtual AttributeSet operator+(const AttributeSet &other) const {
    AttributeSet newAttr(*this);
    newAttr += other;
    return newAttr;
  }

  virtual AttributeSet &operator+=(const AttributeSet &other) {
    color += other.color;
    normal += other.normal;

    for (auto &attr : vec3fAttributes) {
      attr.value += other.GetAttributeValue<Vector3f>(attr.Name());
    }
    for (auto &attr : floatAttributes) {
      attr.value += other.GetAttributeValue<float>(attr.Name());
    }

    return *this;
  }

  virtual AttributeSet operator*(const float scale) const {
    AttributeSet newAttr(*this);
    newAttr *= scale;
    return newAttr;
  }

  virtual AttributeSet &operator*=(const float scale) {
    color *= scale;
    normal *= scale;

    for (auto &attr : vec3fAttributes) {
      attr.value *= scale;
    }
    for (auto &attr : floatAttributes) {
      attr.value *= scale;
    }

    return *this;
  }

  template <class Type> Type GetAttributeValue(const string &name) const {
    const vector<Attribute<Type>> *attributes;
    if constexpr (is_same<Type, float>::value) {
      attributes = &floatAttributes;
    } else if constexpr (is_same<Type, int>::value) {
      attributes = &intAttributes;
    } else if constexpr (is_same<Type, Vector3f>::value) {
      attributes = &vec3fAttributes;
    } else {
      static_assert("Unsupported Type");
    }

    auto mycomp = [&name](const Attribute<Type> &attr) -> bool {
      return attr.IsName(name);
    };
    auto itor = find_if(attributes->begin(), attributes->end(), mycomp);

    if (itor == attributes->end()) {
      return Type();
    } else {
      return itor->value;
    }
  }
};

struct PrimitiveAttributeSet : public AttributeSet {
  bool isShaderFlat = false;

  PrimitiveAttributeSet() = default;
  PrimitiveAttributeSet(const AttributeSet &attr) : AttributeSet(attr) {}
};

struct PointAttributeSet : public AttributeSet {
  PointAttributeSet() = default;
  PointAttributeSet(const AttributeSet &attr) : AttributeSet(attr) {}
};

struct VertexAttributeSet : public AttributeSet {
  Vector2f uv = Vector2f::Zero();

  VertexAttributeSet() = default;
  VertexAttributeSet(const AttributeSet &attr) : AttributeSet(attr) {}
  VertexAttributeSet(const PointAttributeSet &pas) : AttributeSet(pas) {}
};

struct Vertex;

struct Point {
  Vector3f pos;
  vector<Vertex *> vertices;
  PointAttributeSet attributes;

  Point() = default;
  Point(const Vector3f &pos) : pos(pos) {}
  Point(const Vector3f &pos, const AttributeSet &attrSet)
      : pos(pos), attributes(attrSet) {}
  Point(const Point &other) : pos(other.pos), attributes(other.attributes) {}
  Point(const Point *p1, const Point *p2)
      /*
        center point from two points
            varying attribute
      */
      : pos((p1->pos + p2->pos) * 0.5f),
        attributes(p1->attributes.Average(p2->attributes)) {}

  void AverageNormalFromVertices();

  ~Point();

  void FlipAxis(const uint32_t ind);

  Vertex *NewVertex();
  void Merge(Point *&other);
};

struct Vertex {
  Point *point;
  VertexAttributeSet attributes;

  Vertex() = default;
  Vertex(Point *point) : point(point), attributes(point->attributes) {
    point->vertices.push_back(this);
  }

  Vertex(const Vertex *other) {
    attributes = other->attributes;
    other->point->vertices.push_back(this);
  }

  Vertex(const Vertex &other) {
    attributes = other.attributes;
    other.point->vertices.push_back(this);
  }

  Vector3f GetPos() const { return point->pos; }
  void SetPos(const Vector3f &pos) { point->pos = pos; }

  ~Vertex() {
    auto &vertices = point->vertices;
    auto itor = find(vertices.begin(), vertices.end(), this);
    if (itor != vertices.end()) {
      vertices.erase(itor);
    }
  }
};

struct Line2D {
  Line2D(const Vector2f v1, const Vector2f v2) : vertex1(v1), vertex2(v2) {}

  void Draw() const {
    Vector2f dir_vector = vertex2 - vertex1;

    bool max_is_x = true;
    float max_len = 0.0f;
    if (abs(dir_vector[0]) > abs(dir_vector[1])) {
      max_len = abs(dir_vector[0]);
    } else {
      max_len = abs(dir_vector[1]);
      max_is_x = false;
    }

    float deltaPrePixel = System3D::GetSystem()->_DeltaPrePixel();
    auto &colorState = System3D::GetSystem()->_ColorState();
    uint32_t step_times;
    if (max_is_x) {
      step_times = (uint32_t)(max_len / deltaPrePixel);
    } else {
      step_times = (uint32_t)(max_len / deltaPrePixel);
    }
    Vector2f step_dir_v = dir_vector / step_times;
    Vector2f pos = vertex1;
    for (uint32_t i = 0; i < step_times; ++i) {
      System3D::SetBufferColor(pos.x(), pos.y(), colorState);
      pos += step_dir_v;
    }
  }

  Vector2f GetVector() const { return vertex2 - vertex1; }

  Vector2f Sample(const float at) const { return vertex1 + GetVector() * at; }

  Vector2f Intersect(const Line2D &otherLine) const {
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
    Vector2f vec1 = GetVector(), vec2 = otherLine.GetVector();
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

  bool PointInLeft(const Vector2f pointPos) {
    Vector2f vec1 = vertex2 - vertex1, vec2 = pointPos - vertex1;
    return (vec1.x() * vec2.y() - vec2.x() * vec1.y() > 0);
  }

  Vector2f vertex1, vertex2;
};

struct Primitive2D {
public:
  Vector2f PointPosition(const uint32_t ind) const { return points[ind]; }

  list<uint32_t>::iterator InsertPoint(const Vector2f newPoint,
                                       const uint32_t insertInd) {
    auto itor = pointInds.begin();
    advance(itor, insertInd);
    return InsertPoint(newPoint, itor);
  }
  list<uint32_t>::iterator InsertPoint(const Vector2f newPoint) {
    auto itor = pointInds.end();
    return InsertPoint(newPoint, itor);
  }
  list<uint32_t>::iterator
  InsertPoint(const Vector2f newPoint,
              const list<uint32_t>::iterator &insertItor) {
    points.push_back(newPoint);
    return pointInds.insert(insertItor, points.size() - 1);
  }

  void RemovePoint(const uint32_t insertInd = 0) {
    auto itor = pointInds.begin();
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

  void DrawWithoutTrans() const {
    auto itorPre = pointInds.begin();
    auto itorNext = pointInds.begin();
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

  void DrawLines() const {
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

    auto deltaPrePixel = System3D::GetSystem()->_DeltaPrePixel();
    if (yMax - yMin <= deltaPrePixel) {
      return;
    }

    struct ScanPoint {
      float nextY;
      float currentX;
      float deltaX;

      bool isLeft;
      list<uint32_t>::iterator nextPItor;
      list<uint32_t> *pointInds;
      vector<Vector2f> *points;

      ScanPoint(const bool isLeft, const list<uint32_t>::iterator &startItor,
                list<uint32_t> *pointInds, vector<Vector2f> *points)
          : isLeft(isLeft), nextPItor(startItor), pointInds(pointInds),
            points(points) {
        NextLine();
      }

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
        auto deltaPrePixel = System3D::GetSystem()->_DeltaPrePixel();
        Vector2f p1, p2, line = Vector2f::Zero();
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
        Line2D scanLine(Vector2f(spL.currentX, currentY),
                        Vector2f(spR.currentX, currentY));
        scanLine.Draw();
      }

      currentY += deltaPrePixel;
    }
  }

  bool IsPointInside(const Vector2f pos) const {
    uint32_t intersectCount = 0;
    auto itorPre = pointInds.begin();
    auto itorNext = pointInds.begin();
    itorNext++;

    Vector2f dir(random_0_to_1(), 1);
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
    Vector2f pos = {0.0f, 0.0f};
    uint32_t clippedLineInd = 0;
    float clippedAt = 0.0f;
    uint32_t clipLineInd = 0;
    float clipAt = 0.0f;

    IntersectPoint() = default;

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
    vector<Vector2f> entryPos;
    vector<Vector2f> outPos;
    // rebiud clipped
    vector<Vector2f> clippedPoints;
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
      Vector2f preMidPos = (clippedPoints[ind] + clippedPoints[ind_pre]) * 0.5f,
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
    vector<Vector2f> clipPoints;
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
      Vector2f startPos = clippedPoints[ind], currentPos = clippedPoints[ind];
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
  vector<Vector2f> points;
};


Vertex *Point::NewVertex() {
  /*
    Used when create new Primitive
  */
  Vertex *vex = new Vertex;
  vex->point = this;
  vex->attributes = attributes;

  vertices.push_back(vex);

  return vex;
}

void Point::Merge(Point *&other) {
  for (auto verP : other->vertices) {
    verP->point = this;
  }
  other->vertices.clear();
  delete other;
  other = nullptr;
}

void Point::AverageNormalFromVertices() {
  auto verNum = vertices.size();
  if (verNum > 0) {
    Vector3f normal = Vector3f::Zero();
    for (auto verP : vertices) {
      normal += verP->attributes.normal;
    }
    normal /= verNum;
  }
}

Point::~Point() {
  for (auto vexP : vertices) {
    delete vexP;
  }
}

// Only support type that inhert from Primitive
template <class Prim>
Vector2<Vector3f> GetPrimsBoundBox(const vector<const Prim *> &prims) {
  if constexpr (is_same<Prim, Primitive>::value) {
  } else if constexpr (is_same<Prim, Triangle>::value) {
  } else if constexpr (is_same<Prim, Spline>::value) {
  } else {
    assert("Only support type that inhert from Primitive ");
  }

  const Vector3f minConst = {numeric_limits<float>::infinity(),
                             numeric_limits<float>::infinity(),
                             numeric_limits<float>::infinity()},
                 maxConst = {-numeric_limits<float>::infinity(),
                             -numeric_limits<float>::infinity(),
                             -numeric_limits<float>::infinity()};
  Vector2<Vector3f> minMax = {minConst, maxConst};
  for (auto primP : prims) {
    auto primBound = primP->GetBoundBox();
    minMax[0] = minMax[0].cwiseMin(primBound[0]);
    minMax[1] = minMax[1].cwiseMax(primBound[1]);
  }

  return minMax;
}

void Point::FlipAxis(const uint32_t ind) {
  pos[ind] = -pos[ind];
  attributes.normal[ind] = -attributes.normal[ind];
  for (auto verP : vertices) {
    verP->attributes.normal[ind] = -verP->attributes.normal[ind];
  }
}

#endif // !_DATA_STRUCT_HPP_
