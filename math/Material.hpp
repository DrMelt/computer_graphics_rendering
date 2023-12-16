#ifndef _MATERIAL_HPP_
#define _MATERIAL_HPP_
// clang-format off
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stack>
#include <string>
#include <vector>
// clang-format on
#include "Texture.hpp"

using namespace std;
using namespace Eigen;

struct Material {
public:
  Vector3f diffuseColor = {0.8f, 0.8f, 0.8f};
  Vector3f specularColor = {0.8f, 0.8f, 0.8f};
  Vector3f emitionColor = Vector3f::Zero();

  float roughness = 1.0f;

  float transparency = 0.0f;
  float IOR = 1.5f;
  Vector3f transmissionColor = {1.0f, 1.0f, 1.0f};

public:
  virtual Vector3f
  SampleEmitionColor(const Vector3f normal, const Vector3f inDir,
                     const Vector3f outDir,
                     const Vector2f uv = Vector2f::Zero()) const {
    return emitionColor;
  }

  virtual Vector3f
  SampleDiffuseColor(const Vector3f normal, const Vector3f inDir,
                     const Vector3f outDir,
                     const Vector2f uv = Vector2f::Zero()) const {
    return diffuseColor;
  }

  virtual Vector3f
  SampleTransmissionColor(const Vector3f normal, const Vector3f inDir,
                          const Vector3f outDir,
                          const Vector2f uv = Vector2f::Zero()) const {
    return transmissionColor;
  }

  // Will change 'weight'
  virtual Vector3f SampleDiffuseOutDir(const Vector3f &normal,
                                       const Vector3f &inDir,
                                       float &weight) const {
    const float x_1 = random_0_to_1(), x_2 = random_0_to_1();
    const float theta = M_PI_2 - acosf(x_1);
    const auto rateZ = sinf(theta);
    weight *= rateZ * (1.0f - transparency);

    const float r = std::sqrt(1.0f - rateZ * rateZ), phi = 2 * M_PI * x_2;
    const Vector3f localRay(r * std::cos(phi), r * std::sin(phi), rateZ);
    auto toLookAt = ToLookAt(normal, inDir);

    const auto newDir = (toLookAt * localRay).normalized();

    return newDir;
  }

  virtual Vector3f SampleTransmissionOutDir(const Vector3f &normal,
                                            const Vector3f &inDir,
                                            float &weight) const {
    weight *= transparency;

    const auto inDirProject = normal.dot(inDir);
    const auto dirVertical = inDir - normal * inDirProject;
    const float sinIn = dirVertical.norm();
    float sinOut;
    // to air
    if (inDirProject > 0.0f) {
      sinOut = sinIn * IOR;
    }
    // to inner
    else {
      sinOut = sinIn / IOR;
    }

    Vector3f newDir;
    // total reflection
    if (sinOut > 1.0f) {
      newDir = -inDir + 2 * dirVertical;
    }
    // refrection
    else {
      newDir =
          (normal * inDirProject).normalized() * sqrt(1 - sinOut * sinOut) +
          dirVertical.normalized() * sinOut;
      newDir = newDir.normalized();
    }

    return newDir;
  }
};

struct SkyBox : public Material {

  Texture<Vector3f> texture;

  Vector3f SampleSkyBoxColor(const Vector3f &inDir) const {
    auto texturePos = MapDir2TextureInCylinder(inDir);
    auto color = texture.Sample(texturePos.x(), texturePos.y());

    return color;
  }
};

#endif // !_MATERIAL_HPP_
