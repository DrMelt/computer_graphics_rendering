#pragma once

enum RayType {
  CAMERA,
  DIFFUSE,
  SPECULAR,
  REFLECTION,
  REFRACTION,
  EMISSION,
  SKYBOX,
  ALPHA
};

struct SamplePointInfo {

  SamplePointInfo() = default;
  SamplePointInfo(const SamplePointInfo &other) = default;

  RayType rayType = RayType::CAMERA;

  System3D::SampleDeep sampleDeep = {0, 0}; // times of ray bounced

  uint32_t ind = 0;
  uint32_t fromInd = 0;

  float weight = 1.0f; // effected weight from accColor

  Vector3f color = {1.0f, 1.0f, 1.0f};
  Vector3f emition = {0.0f, 0.0f, 0.0f};
  Vector3f accColor = {0.0f, 0.0f,
                       0.0f}; // the light shot on this point (irradiance)

  const Material *material = nullptr;
  const Triangle *triaP = nullptr;

  Vector3f pos = Vector3f::Zero();
  Vector3f normal = Vector3f::Zero();
  Vector3f inDir = Vector3f::Zero();
  Vector2f uv = Vector2f::Zero();
  // Vector3f outDir;
};

struct SamplePointInfos {
public:
  vector<SamplePointInfo> sampleInfos;

public:
  Vector3f AccColor() const {
    vector<SamplePointInfo> sampleInfoCopy = sampleInfos;

    long ind = sampleInfoCopy.size() - 1;

    while (ind > -1) {
      if (ind > 0) {
        auto &info = sampleInfoCopy[ind];
        sampleInfoCopy[info.fromInd].accColor +=
            info.weight *
            (info.emition + info.accColor.cwiseProduct(info.color));
      } else if (ind > -1) {
        auto &info = sampleInfoCopy[0];

        return info.accColor;
      } else {
        return Vector3f::Zero();
      }

      ind--;
    }

    return Vector3f::Zero();
  }

  System3D::SampleDeep ThisRayPathDeep() const {
    return (sampleInfos)[sampleInfos.size() - 1].sampleDeep;
  }

  void PushSamplePointInfo(SamplePointInfo &info, Ray &ray,
                           const RayType rayType);

  const SamplePointInfo &LastInfo() const {
    return (sampleInfos)[sampleInfos.size() - 1];
  }

  Vector3f HitNormal(const uint32_t hit) const {
    if (sampleInfos.size() > hit) {
      return (sampleInfos)[hit].normal;
    } else {
      return Vector3f::Zero();
    }
  }

  Vector3f HitNormal(const RayType rayType = RayType::DIFFUSE) const {
    Vector3f result = Vector3f::Zero();

    uint32_t ind = 0;
    while (ind < sampleInfos.size()) {
      if (sampleInfos[ind].rayType == rayType) {
        result = sampleInfos[ind].normal;
        break;
      }
      ind++;
    }

    return result;
  }

  Vector3f HitAlbedo(const uint32_t hit) const {
    if (sampleInfos.size() > hit) {
      return (sampleInfos)[hit].color;
    } else {
      return Vector3f::Zero();
    }
  }

  Vector3f HitAlbedo(const RayType rayType = RayType::DIFFUSE) const {
    Vector3f result = Vector3f::Zero();

    uint32_t ind = 0;
    while (ind < sampleInfos.size()) {
      if (sampleInfos[ind].rayType == rayType) {
        result = sampleInfos[ind].color;
        break;
      }
      ind++;
    }

    return result;
  }
};

struct Ray {
public:
  Vector3f origin = Vector3f::Zero();
  Vector3f dir = Vector3f(0, 0, 1.0f); // should be normalized
  float deep = numeric_limits<float>::infinity();

  SamplePointInfo sampleInfo;

public:
  Ray() = default;
  Ray(const Ray &other) = default;

public:
  bool IsThisRayPathHit() const {
    return deep < numeric_limits<float>::infinity() && deep > 1e-6f;
  }

  void Transform(const Matrix4f &transM) {
    origin = TransformForPos(origin, transM);
    dir = TransformForVector(dir, transM);
  }
};

void SamplePointInfos::PushSamplePointInfo(SamplePointInfo &info, Ray &ray,
                                           const RayType rayType) {

  info.rayType = rayType;
  info.ind = sampleInfos.size();
  info.fromInd = ray.sampleInfo.ind;
  ray.sampleInfo = info;

  sampleInfos.push_back(info);
}
