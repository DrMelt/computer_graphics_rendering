#pragma once

struct SamplePointInfo {
  SamplePointInfo() = default;
  SamplePointInfo(const SamplePointInfo &other) = default;

  uint32_t sampleDeep = 0;

  uint32_t ind = 0;
  uint32_t fromInd = 0;

  float weight = 1.0f; // effected weight from accColor

  Vector3f color = {1.0f, 1.0f, 1.0f};
  Vector3f emition = {0.0f, 0.0f, 0.0f};
  Vector3f accColor = {0.0f, 0.0f,
                       0.0f}; // the light shot on this point (irradiance)

  const Material *material = nullptr;

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

    int ind = sampleInfoCopy.size() - 1;

    while (true) {
      if (ind > 0) {
        auto &info = sampleInfoCopy[ind];
        sampleInfoCopy[info.fromInd].accColor +=
            info.emition + info.weight * info.accColor.cwiseProduct(info.color);
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

  bool IsThisRayPathContinue(const uint32_t maxTimes) const {
    if (sampleInfos.size() < 1) {
      return true;
    } else {
      return (sampleInfos)[sampleInfos.size() - 1].sampleDeep < maxTimes;
    }
  }

  uint32_t ThisRayPathDeep() const {
    return (sampleInfos)[sampleInfos.size() - 1].sampleDeep;
  }

  void PushSamplePointInfo(SamplePointInfo &info, Ray &ray);

  const SamplePointInfo &LastInfo() const {
    return (sampleInfos)[sampleInfos.size() - 1];
  }

  Vector3f HitNormal(const uint32_t hit = 0) const {
    if (sampleInfos.size() > hit) {
      return (sampleInfos)[hit].normal;
    } else {
      return Vector3f::Zero();
    }
  }

  Vector3f HitAlbedo(const uint32_t hit = 0) const {
    if (sampleInfos.size() > hit) {
      return (sampleInfos)[hit].color;
    } else {
      return Vector3f::Zero();
    }
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
  bool IsThisRayPathHitted() const {
    return deep < numeric_limits<float>::infinity();
  }

  void Transform(const Matrix4f &transM) {
    origin = TransformForPos(origin, transM);
    dir = TransformForVector(dir, transM);
  }
};

void SamplePointInfos::PushSamplePointInfo(SamplePointInfo &info, Ray &ray) {
  info.sampleDeep = ray.sampleInfo.sampleDeep + 1;

  info.ind = sampleInfos.size();
  info.fromInd = ray.sampleInfo.ind;
  ray.sampleInfo = info;

  sampleInfos.push_back(info);
}
