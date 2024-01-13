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

using namespace std;
using namespace Eigen;

/*
    Fresnel approximation
    normal and inDir should be normalized
*/
float Fresnel(const float f0, const float inDirProject,
              const float IOR = 1.0f) {
  return f0 + (1 - f0) * powf(inDirProject * IOR, 4);
}

struct Material {
public:
  string name;

  Vector3f diffuseColor = {0.8f, 0.8f, 0.8f};
  Vector3f specularColor = {0.8f, 0.8f, 0.8f};
  Vector3f emissionColor = Vector3f::Zero();
  Vector3f transmissionColor = {1.0f, 1.0f, 1.0f};
  float alpha = 1.0f;

  Texture<Vector3f> *diffuseTexture = nullptr;
  Texture<Vector3f> *specularTexture = nullptr;
  Texture<Vector3f> *emissionTexture = nullptr;
  Texture<Vector3f> *transmissionTexture = nullptr;
  Texture<float> *alphaTexture = nullptr;

  float roughness = 1.0f;

  float transparency = 0.0f;
  float IOR = 1.5f;

public:
  Material() = default;
  virtual ~Material() {
    if (diffuseTexture != nullptr) {
      delete diffuseTexture;
      diffuseTexture = nullptr;
    }
    if (specularTexture != nullptr) {
      delete specularTexture;
      specularTexture = nullptr;
    }
    if (emissionTexture != nullptr) {
      delete emissionTexture;
      emissionTexture = nullptr;
    }
    if (transmissionTexture != nullptr) {
      delete transmissionTexture;
      transmissionTexture = nullptr;
    }
  }

  Material(const Material &mt)
      : name(mt.name), roughness(mt.roughness), transparency(mt.transparency),
        IOR(mt.IOR) {
    if (mt.diffuseTexture != nullptr) {
      diffuseTexture = new Texture<Vector3f>(*mt.diffuseTexture);
    } else {
      diffuseTexture = nullptr;
    }
    if (mt.specularTexture != nullptr) {
      specularTexture = new Texture<Vector3f>(*mt.specularTexture);
    } else {
      specularTexture = nullptr;
    }
    if (mt.emissionTexture != nullptr) {
      emissionTexture = new Texture<Vector3f>(*mt.emissionTexture);
    } else {
      emissionTexture = nullptr;
    }
    if (mt.transmissionTexture != nullptr) {
      transmissionTexture = new Texture<Vector3f>(*mt.transmissionTexture);
    } else {
      transmissionTexture = nullptr;
    }
  }

public:
  virtual Vector3f
  SampleEmissionColor(const Vector3f normal, const Vector3f inDir,
                      const Vector2f uv = Vector2f::Zero()) const {
    if (emissionTexture != nullptr) {
      return emissionTexture->Sample(uv.x(), uv.y());
    } else {
      return emissionColor;
    }
  }

  virtual Vector3f
  SampleSpecularColor(const Vector3f normal, const Vector3f inDir,
                      const Vector2f uv = Vector2f::Zero()) const {
    if (specularTexture != nullptr) {
      return specularTexture->Sample(uv.x(), uv.y());
    } else {
      return specularColor;
    }
  }

  virtual Vector3f
  SampleDiffuseColor(const Vector3f normal, const Vector3f inDir,
                     const Vector2f uv = Vector2f::Zero()) const {
    if (diffuseTexture != nullptr) {
      return diffuseTexture->Sample(uv.x(), uv.y());
    } else {
      return diffuseColor;
    }
  }

  virtual Vector3f
  SampleTransmissionColor(const Vector3f normal, const Vector3f inDir,
                          const Vector2f uv = Vector2f::Zero()) const {
    if (transmissionTexture != nullptr) {
      return transmissionTexture->Sample(uv.x(), uv.y());
    } else {
      return transmissionColor;
    }
  }

  virtual float SampleAlpha(const Vector3f normal, const Vector3f inDir,
                            const Vector2f uv = Vector2f::Zero()) const {
    if (alphaTexture != nullptr) {
      return alphaTexture->Sample(uv.x(), uv.y());
    } else {
      return alpha;
    }
  }

  virtual float SampleAlpha(const Vector2f uv) const {
    if (alphaTexture != nullptr) {
      return alphaTexture->Sample(uv.x(), uv.y());
    } else {
      return alpha;
    }
  }
  // Will change 'weight'
  virtual Vector3f SampleDiffuseOutDir(const Vector3f &normal,
                                       const Vector3f &inDir,
                                       float &weight) const {
    const float x_1 = random_0_to_1();
    const float x_2 = random_0_to_1();
    const float theta = M_PI_2 - acosf(x_1);
    const float rateZ = sinf(theta);
    // weight = rateZ * (1.0f - transparency) * static_cast<float>(M_PI * 2);
    weight = rateZ * (1.0f - transparency);

    const float r = sqrtf(1.0f - rateZ * rateZ), phi = 2 * M_PI * x_2;
    const Vector3f localRay(r * cosf(phi), r * sinf(phi), rateZ);

    const float sideSign = normal.dot(inDir) < 0.0f ? 1.0f : -1.0f;
    const Matrix3f toLookAt = ToLookAt(normal * sideSign, inDir);

    const Vector3f newDir = (toLookAt * localRay).normalized();

    return newDir;
  }

  virtual Vector3f SampleTransmissionOutDir(const Vector3f &normal,
                                            const Vector3f &inDir,
                                            float &weight) const {
    weight = transparency;

    const auto inDirProject = normal.dot(inDir);
    const auto dirVertical = inDir - normal * inDirProject;
    const float sinIn = dirVertical.norm();
    float sinOut;
    // to air
    if (inDirProject > 0.0f) {
      sinOut = sinIn * IOR;
      weight *= Fresnel(0.1f, inDirProject, IOR);
    }
    // to inner
    else {
      sinOut = sinIn / IOR;
      weight *= Fresnel(0.1f, -inDirProject, 1.0f);
    }

    Vector3f newDir;
    // total reflection
    if (sinOut > 1.0f) {
      weight = 0.0f;
    }
    // refraction
    else {
      newDir =
          (normal * inDirProject).normalized() * sqrt(1 - sinOut * sinOut) +
          dirVertical.normalized() * sinOut;
      newDir = newDir.normalized();
    }

    return newDir;
  }

  virtual Vector3f SampleReflectionOutDir(const Vector3f &normal,
                                          const Vector3f &inDir,
                                          float &weight) const {

    const auto inDirProject = normal.dot(inDir);
    const auto dirVertical = inDir - normal * inDirProject;

    // in
    if (inDirProject > 0.0f) {
      weight = (1.0f - Fresnel(0.1f, inDirProject, IOR));
    }
    // out
    else {
      weight = (1.0f - Fresnel(0.1f, -inDirProject, 1.0f));
    }

    Vector3f newDir;
    newDir = -inDir + 2 * dirVertical;

    return newDir;
  }
};

struct SkyBox : public Material {
public:
  Texture<Vector3f> *texture = nullptr;

public:
  virtual ~SkyBox() {
    if (texture != nullptr) {
      delete texture;
      texture = nullptr;
    }
  }

public:
  Vector3f SampleSkyBoxColor(const Vector3f &inDir) const {
    if (texture != nullptr) {
      auto texturePos = MapDir2TextureInCylinder(inDir);
      auto color = texture->Sample(texturePos.x(), texturePos.y());

      return color;
    } else {
      return emissionColor;
    }
  }
};

class MaterialManager {
protected:
  vector<Material *> materials;

public:
  MaterialManager() {
    auto defaultMaterial = new Material;
    defaultMaterial->name = "defualt";
    materials.push_back(defaultMaterial);
  }

  virtual ~MaterialManager() {
    for (auto &mP : materials) {
      delete mP;
    }
  }

  void PushMaterial(Material *const materialP) {
    materials.push_back(materialP);
  }

  Material *const GetDefaultMaterial() { return materials[0]; }

  Material *const GetMaterial(const uint32_t ind) { return materials[ind]; }

  Material *const GetMaterialByName(const string &name) {
    auto itor =
        find_if(materials.begin(), materials.end(),
                [name](Material *mt) -> bool { return mt->name == name; });
    return *itor;
  }
};

#endif // !_MATERIAL_HPP_
