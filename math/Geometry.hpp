#pragma once

class Geometry {
public:
  ~Geometry() {
    for (auto primP : primitives) {
      delete primP;
    }

    for (auto pointP : points) {
      delete pointP;
    }
  }

  Geometry() = default;
  Geometry(const Axis axis) : axis(axis) {}

  Geometry(const Geometry &other) = delete;

  Axis axis;

protected:
  vector<Primitive *> primitives;
  vector<Point *> points;

public:
  Primitive *GetPrimitive(const uint32_t ind) const {
    return primitives.at(ind);
  }

  Point *AddPoint(const Vector3f &pos) {
    auto newPoint = new Point;
    newPoint->pos = pos;

    points.push_back(newPoint);

    return newPoint;
  }

  Point *GetPoint(const uint32_t ind) const { return points.at(ind); }

  void PushPrimsToSystem() {
    for (auto primP : primitives) {
      auto triaP = dynamic_cast<Triangle *>(primP);
      if (triaP != nullptr) {
        System3D::PushTriangleRef(triaP);
      }

      auto splineP = dynamic_cast<Spline *>(primP);
      if (splineP != nullptr) {
        System3D::PushSplineRef(splineP);
      }
    }
  }

  void ComputeTrianglesNormal() {
    for (auto primP : primitives) {
      auto triaP = dynamic_cast<Triangle *>(primP);
      if (triaP != nullptr) {
        triaP->ComputeNormal();
      }
    }
    RefreshPointsNormal();
  }

  void RefreshPointsNormal() {
    for (auto ptP : points) {
      ptP->AverageNormalFromVertices();
    }
  }

  void AddPrimitive(Primitive *prim) { primitives.push_back(prim); }

  void AddPrimitive(const Primitive &prim) {
    auto newPrimP = new Primitive(prim);
    primitives.push_back(newPrimP);
  }

  void Draw(const Primitive2D &clipPrim) {

    for (auto &primitive : primitives) {
      auto meshP = dynamic_cast<Mesh *>(primitive);
      if (meshP != nullptr) {
        meshP->Draw(clipPrim);
      }

      auto bezierP = dynamic_cast<Bezier *>(primitive);
      if (bezierP != nullptr) {
        bezierP->DrawToBuffer();
      }
    }
  }

  void DrawPointsToBuffer() {
    auto MVP = System3D::GetMVP();
    auto system = System3D::GetSystem();
    auto deltaPrePixel = system->_DeltaPrePixel();
    Vector4f color = {1.0f, 1.0f, 1.0f, 1.0f};
    for (auto pointP : points) {
      auto pos = TransformForPos(pointP->pos, MVP);
      color.block<3, 1>(0, 0) = pointP->attributes.color;
      for (int i = -3; i < 4; i++) {
        for (int j = -3; j < 4; j++) {
          System3D::SetBufferColor(pos.x() + i * deltaPrePixel,
                                   pos.y() + j * deltaPrePixel, color);
        }
      }
    }
  }

  void TransformWorld(const Matrix4f tranM) {
    axis.Transform(tranM);

    for (auto &point : points) {
      point->pos = TransformForPos(point->pos, tranM);
    }
  }

  void CleanPoints() {
    /*
      clear unused points
      merge close points
    */
    auto itor = points.begin();
    for (; itor != points.end(); itor++) {
      if ((*itor)->vertices.size() < 1) {
        delete *itor; // delete point in memery

        itor = points.erase(itor);
      }
    }

    const float distanceThreshold = 1e-6;
    for (uint32_t ind = 0; ind < points.size(); ++ind) {
      // Search near points
      auto pos = points[ind]->pos;
      vector<uint32_t> nearPointsInd;
      for (uint32_t indSearch = ind + 1; indSearch < points.size();
           ++indSearch) {
        if ((points[indSearch]->pos - pos).norm() < distanceThreshold) {
          nearPointsInd.push_back(indSearch);
        }
      }

      // Merge
      for (auto nearPInd : nearPointsInd) {
        points[ind]->Merge(points[nearPInd]);
      }

      // Delete
      auto itor = points.begin();
      for (; itor != points.end(); ++itor) {
        if ((*itor) == nullptr) {
          itor = points.erase(itor);
        }
      }
    }
  }

  void SubdivideMesh() {
    vector<Primitive *> newPrims;
    for (auto primP : primitives) {
      vector<Point *> newPts;
      auto verNum = primP->VerticesNum();

      {
        for (uint32_t ind = 1; ind < verNum; ind++) {
          auto newP = new Point(primP->GetVertex(ind)->point,
                                primP->GetVertex(ind - 1)->point);
          newPts.push_back(newP);
        }
        Point *newP = new Point(primP->GetVertex(0)->point,
                                primP->GetVertex(verNum - 1)->point);
        newPts.push_back(newP);
      }

      {
        Vector3f centerPos = Vector3f::Zero();
        vector<AttributeSet *> attriSets;
        for (uint32_t ind = 0; ind < verNum; ind++) {
          centerPos += primP->GetVertex(ind)->GetPos();
          attriSets.push_back(&primP->GetVertex(ind)->attributes);
        }
        centerPos /= static_cast<float>(verNum);
        Point *newP = new Point(centerPos, AttributeSet::Average(attriSets));
        newPts.push_back(newP);
      }

      // Biuld new Triangles
      {
        for (uint32_t origonVerP = 1; origonVerP < verNum; origonVerP++) {
          auto pt1 = primP->GetVertex(origonVerP)->point;
          auto pt2 = newPts[origonVerP];
          auto pt3 = newPts[origonVerP - 1];
          Triangle *newTri = new Triangle(pt1, pt2, pt3);
          newPrims.push_back(newTri);
        }
        auto pt1 = primP->GetVertex(0)->point;
        auto pt2 = newPts[verNum - 1];
        auto pt3 = newPts[0];
        Triangle *newTri = new Triangle(pt1, pt2, pt3);
        newPrims.push_back(newTri);
      }

      {
        for (uint32_t origonVerP = 1; origonVerP < verNum; origonVerP++) {
          auto pt1 = newPts[verNum];
          auto pt2 = newPts[origonVerP];
          auto pt3 = newPts[origonVerP - 1];
          Triangle *newTri = new Triangle(pt1, pt2, pt3);
          newPrims.push_back(newTri);
        }
        auto pt1 = newPts[verNum];
        auto pt2 = newPts[verNum - 1];
        auto pt3 = newPts[0];
        Triangle *newTri = new Triangle(pt1, pt2, pt3);
        newPrims.push_back(newTri);
      }

      for (auto ptP : newPts) {
        points.push_back(ptP);
      }
    }

    for (auto primP : primitives) {
      delete primP;
    }
    primitives = newPrims;

    CleanPoints();
  }

  void ToShpere() {
    Vector3f centerPos = Vector3f::Zero();

    for (auto &ptP : points) {
      centerPos += ptP->pos;
    }

    float averageDistance = 0.0f;
    centerPos /= points.size();
    for (auto &ptP : points) {
      averageDistance += (ptP->pos - centerPos).norm();
    }
    averageDistance /= points.size();

    for (auto &ptP : points) {
      ptP->pos = averageDistance * (ptP->pos - centerPos).normalized();
    }
  }

  void FlipX() {
    for (auto ptP : points) {
      ptP->FlipAxis(0);
    }
  }

  void AssignMaterial(Material *const material) {
    for (auto &primP : primitives) {
      auto meshP = dynamic_cast<Mesh *>(primP);
      if (meshP != nullptr) {
        meshP->material = material;
      }
    }
  }
};
