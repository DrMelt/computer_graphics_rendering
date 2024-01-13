#pragma once
struct Primitive {
public:
  Primitive() = default;
  Primitive(const Primitive &other)
      : verticesTrans(other.verticesTrans), attributes(other.attributes) {
    for (auto vexP : other.vertices) {
      vertices.push_back(vexP->point->NewVertex());
    }
  }

  virtual ~Primitive() {
    for (auto &verP : vertices) {
      delete verP;
      verP = nullptr;
    }
  }

public:
  void Transform(const Matrix4f &tranM) const {
    for (auto &vex : vertices) {
      vex->SetPos(TransformForPos(vex->GetPos(), tranM));
    }
  }

  Vertex *GetVertex(const uint32_t ind) const { return vertices.at(ind); }

  uint32_t VerticesNum() const { return vertices.size(); }

  Vector3f VerticesCenter() const {
    Vector3f counter = Vector3f::Zero();
    for (auto verP : vertices) {
      counter += verP->GetPos();
    }
    return counter / VerticesNum();
  }

  virtual vector<Vertex *>::iterator InsertVertex(Point *newPoint,
                                                  const uint32_t insertInd) {
    auto itor = vertices.begin();
    advance(itor, insertInd);
    return InsertVertex(newPoint, itor);
  }

  virtual void PushBack(Point *newPoint) {
    vertices.push_back(newPoint->NewVertex());
  }

  virtual vector<Vertex *>::iterator
  InsertVertex(Point *newPoint, const vector<Vertex *>::iterator &insertItor) {
    vertices.push_back(newPoint->NewVertex());
    return vertices.insert(insertItor, newPoint->NewVertex());
  }

  virtual void RemoveVertex(const uint32_t insertInd = 0) {
    auto itor = vertices.begin();
    advance(itor, insertInd);
    vertices.erase(itor);
  }

  virtual Vector2<Vector3f> GetBoundBox() const {
    const Vector3f minConst = {numeric_limits<float>::infinity(),
                               numeric_limits<float>::infinity(),
                               numeric_limits<float>::infinity()},
                   maxConst = {-numeric_limits<float>::infinity(),
                               -numeric_limits<float>::infinity(),
                               -numeric_limits<float>::infinity()};
    Vector2<Vector3f> minMax = {minConst, maxConst};

    for (auto verP : vertices) {
      auto pos = verP->GetPos();
      minMax[0] = minMax[0].cwiseMin(pos);
      minMax[1] = minMax[1].cwiseMax(pos);
    }

    return minMax;
  }

protected:
  PrimitiveAttributeSet attributes;
  vector<Vertex *> vertices;
  vector<Vector3f> verticesTrans;
};

struct Spline : public Primitive {
public:
  Spline() = default;

public:
  virtual Vector3f Sample(const float t) const = 0;
  virtual void DrawToBuffer(const uint32_t parts = 1000) const = 0;
};

struct Bezier : public Spline {
public:
  Bezier() = default;

public:
  virtual Vector3f Sample(const float t) const {
    vector<Vector3f> *pos1 = new vector<Vector3f>, *pos2 = new vector<Vector3f>;
    for (auto verP : vertices) {
      pos1->push_back(verP->GetPos());
    }

    while (pos1->size() > 1) {
      pos2->clear();
      for (uint32_t i = 1; i < pos1->size(); ++i) {
        auto pos = t * (*pos1)[i - 1] + (1 - t) * (*pos1)[i];
        pos2->push_back(pos);
      }

      auto tmp = pos1;
      pos1 = pos2;
      pos2 = tmp;
    }

    auto result = (*pos1)[0];
    delete pos1;
    delete pos2;
    return result;
  }

  virtual void DrawToBuffer(const uint32_t parts = 1000) const {
    float tStep = 1.0f / parts;
    float t = 0.0f;
    for (uint32_t i = 0; i < parts; ++i) {
      auto pos = Sample(t);
      t += tStep;

      pos = TransformForPos(pos, System3D::GetMVP());

      System3D::SetBufferColor(pos.x(), pos.y());
    }
  }
};

struct BSpline : public Spline {
public:
  uint32_t K;
  vector<float> knotVector;

  BSpline(const uint32_t K = 3) : Spline(), K(K) {}

public:
  void PushBack(Point *newPoint) override {
    if (knotVector.size() > 0) {
      knotVector.push_back(knotVector[knotVector.size() - 1] + 1.0f);
    } else {
      knotVector.push_back(0.0f);
    }
    vertices.push_back(newPoint->NewVertex());
  }

  virtual void PushBack(Point *newPoint, const float knot) {
    knotVector.push_back(knot);
    vertices.push_back(newPoint->NewVertex());
  }

public:
  virtual Vector3f Sample(const float t) const {
    uint32_t iEnd = 0;
    while (knotVector[iEnd] < t) {
      iEnd += 1;
    }
    Vector3f result = Vector3f::Zero();

    for (uint32_t ind = iEnd - K; ind < iEnd; ++ind) {
      result += vertices[ind]->GetPos() * GetWeight(t, ind);
    }

    return result;
  }

  virtual void DrawToBuffer(const uint32_t parts = 1000) const {
    if (vertices.size() < 2 * K + 1) {
      return;
    }

    float tStep =
        (knotVector[knotVector.size() - K] - knotVector[K - 1] - 1e-6) / parts;
    float t = knotVector[K - 1];
    for (uint32_t i = 0; i < parts; ++i) {
      auto pos = Sample(t);
      t += tStep;

      pos = TransformForPos(pos, System3D::GetMVP());

      System3D::SetBufferColor(pos.x(), pos.y());
    }
  }

protected:
  float GetWeight(const float t, const uint32_t i) const {
    if (i > vertices.size() - 1) {
      return 0.0f;
    }

    struct NFunc {
      float weightAcc = 1.0f;
      uint32_t i;
      uint32_t K;
    };

    stack<NFunc> NStack;
    float weightAcc = 0.0f;

    NFunc n0 = {1.0f, i, K};
    NStack.push(n0);

    while (NStack.size() > 0) {
      auto n = NStack.top();
      NStack.pop();

      if (n.K > 1) {
        NFunc n1 = {n.weightAcc * (t - knotVector[n.i]) /
                        (knotVector[n.i + n.K - 1] - knotVector[n.i]),
                    n.i, n.K - 1};
        NStack.push(n1);

        NFunc n2 = {n.weightAcc * (knotVector[n.i + n.K] - t) /
                        (knotVector[n.i + n.K] - knotVector[n.i + 1]),
                    n.i + 1, n.K - 1};
        NStack.push(n2);
      } else {
        if (t >= knotVector[n.i] && t < knotVector[n.i + 1]) {
          weightAcc += n.weightAcc;
        }
      }
    }

    return weightAcc;
  }
};

struct Mesh : public Primitive {
public:
  const Material *material = nullptr;

public:
  Mesh() = default;

  Mesh(const Mesh &other) : Primitive(other) {}

public:
  virtual void Draw(const Primitive2D &clipPrim) {
    auto MVP = System3D::GetMVP();

    verticesTrans.clear();
    verticesTrans.resize(vertices.size());
    for (uint32_t i = 0; i < vertices.size(); ++i) {
      verticesTrans[i] = vertices[i]->GetPos();
    }
    for (auto &point : verticesTrans) {
      point = TransformForPos(point, MVP);
    }

    if (vertices.size() < 3) {
      return;
    }

    // is front
    auto p1 = verticesTrans[0];
    auto p2 = verticesTrans[1];
    auto p3 = verticesTrans[2];
    auto line1 = p2 - p1;
    auto line2 = p3 - p1;
    auto crossResult = line1.cross(line2);
    if (crossResult.z() < 0) {
      return;
    }

    Primitive2D prim2d;
    for (auto &vex : verticesTrans) {
      prim2d.InsertPoint(vex.block<2, 1>(0, 0));
    }
    auto prims = Primitive2D::WALineClip(prim2d, clipPrim);
    for (auto &prim : prims) {
      prim.DrawLines();
      prim.Fill();
    }
  }
};

struct Triangle : public Mesh {
public:
  Triangle(Point *p1, Point *p2, Point *p3) {
    Primitive::PushBack(p1);
    Primitive::PushBack(p2);
    Primitive::PushBack(p3);
  }

  Triangle(Vertex *v1, Vertex *v2, Vertex *v3) {
    vertices.push_back(v1);
    vertices.push_back(v2);
    vertices.push_back(v3);
  }

public:
  void PushBack(Point *newPoint) override {
    throw string("Triangle cannot add vertex");
  };

  vector<Vertex *>::iterator InsertVertex(Point *newPoint,
                                          const uint32_t insertInd) override {
    throw string("Triangle cannot add vertex");
  }

  vector<Vertex *>::iterator
  InsertVertex(Point *newPoint,
               const vector<Vertex *>::iterator &insertItor) override {
    throw string("Triangle cannot add vertex");
  }

  void RemoveVertex(const uint32_t insertInd = 0) override {
    throw string("Triangle cannot move vertex");
  }

public:
  float Area() const {
    const Vector3f v1 = vertices[1]->GetPos() - vertices[0]->GetPos();
    const Vector3f v2 = vertices[2]->GetPos() - vertices[0]->GetPos();

    // cout << vertices[2]->GetPos() << endl
    //      << endl
    //      << vertices[1]->GetPos() << endl
    //      << endl
    //      << vertices[0]->GetPos() << endl
    //      << endl;

    return (v1.cross(v2)).norm() * 0.5f;
  }

  Vector3f ScatterPoint() const {
    const auto v1 = vertices[1]->GetPos() - vertices[0]->GetPos(),
               v2 = vertices[2]->GetPos() - vertices[0]->GetPos();

    const float weight1 = random_0_to_1(),
                weight2 = (1.0f - weight1) * random_0_to_1();

    return v1 * weight1 + v2 * weight2 + vertices[0]->GetPos();
  }

  void ComputeNormal() {
    Vector3f v1 = vertices[1]->GetPos() - vertices[0]->GetPos(),
             v2 = vertices[2]->GetPos() - vertices[0]->GetPos();
    Vector3f normal = v1.cross(v2).normalized();
    vertices[0]->attributes.normal = normal;
    vertices[1]->attributes.normal = normal;
    vertices[2]->attributes.normal = normal;

    attributes.normal = normal;
  }

  Vector4f HitColor(Ray &ray) const {
    Vector3f intersect = RayIntersect(ray);
    auto varyingColor = VaryingColor(intersect);
    Vector4f color = Vector4f::Zero();
    color.block<3, 1>(0, 0) = varyingColor;
    color.w() = 1.0f;

    return color;
  }

  // Return Triangle position (x+y+z = 1)
  // alpha, beta, gamma.
  // Will change t(deep) in ray
  // if no intersection ray, deep will be setted infinity
  Vector3f RayIntersect(Ray &ray) const {

    const Vector3f cPos = vertices[2]->GetPos();
    Matrix3f pr;
    pr.block<3, 1>(0, 0) = vertices[0]->GetPos() - cPos;
    pr.block<3, 1>(0, 1) = vertices[1]->GetPos() - cPos;
    pr.block<3, 1>(0, 2) = -ray.dir;

    const Vector3f solve = pr.inverse() * (ray.origin - cPos);

    const float alpha = solve.x(), beta = solve.y(),
                gamma = 1.0f - alpha - beta;

    ray.deep = solve.z();
    // if not in triangle or behind the origin
    if (alpha < 0.0f || beta < 0.0f || gamma < 0.0f || ray.deep < 1e-6f ||
        isnan(ray.deep)) {
      ray.deep = numeric_limits<float>::infinity();
    }

    return Vector3f(alpha, beta, gamma);
  }

  Vector3f VaryingNormal(const Vector3f &triaPos) const {
    return vertices[0]->attributes.normal * triaPos.x() +
           vertices[1]->attributes.normal * triaPos.y() +
           vertices[2]->attributes.normal * triaPos.z();
  }

  Vector3f VaryingPos(const Vector3f &triaPos) const {
    return vertices[0]->GetPos() * triaPos.x() +
           vertices[1]->GetPos() * triaPos.y() +
           vertices[2]->GetPos() * triaPos.z();
  }
  Vector3f VaryingColor(const Vector3f &triaPos) const {
    return vertices[0]->attributes.color * triaPos.x() +
           vertices[1]->attributes.color * triaPos.y() +
           vertices[2]->attributes.color * triaPos.z();
  }

  Vector2f VaryingUV(const Vector3f &triaPos) const {
    return vertices[0]->attributes.uv * triaPos.x() +
           vertices[1]->attributes.uv * triaPos.y() +
           vertices[2]->attributes.uv * triaPos.z();
  }
};

vector<Triangle *> ConvertMeshToTriangles(Mesh *mesh) {
  vector<Triangle *> result;
  auto vertice0 = mesh->GetVertex(0);
  auto verNum = mesh->VerticesNum();
  auto preVer = mesh->GetVertex(1);
  for (uint32_t i = 2; i < verNum; ++i) {
    auto currentVer = mesh->GetVertex(i);

    result.push_back(
        new Triangle(vertice0->point, preVer->point, currentVer->point));
    preVer = currentVer;
  }
  return result;
}