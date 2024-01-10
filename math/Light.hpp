#ifndef _LIGHT_HPP_
#define _LIGHT_HPP_

// clang-format off
#include "Texture.hpp"
#include "DataStruct.hpp"
// clang-format on

struct Light {
public:
  Vector3f intensity = {5.0f, 5.0f, 5.0f};
  Axis axis;
  Texture<float> *shadowMap = nullptr;

  uint32_t shadowMapHeight = 512;

  Light() = default;
  virtual ~Light() {
    if (shadowMap != nullptr) {
      delete shadowMap;
    }
  }

  void ShowShadowMapToBuffer(const float vaulesScale = 0.2f) {
    System3D::ShowTextureToBuffer(*shadowMap, vaulesScale);
  }

  virtual bool CanLighting(const Vector3f &pos,
                           const float epsilon = 1e-2) const {
    if (shadowMap != nullptr) {
      auto vec = pos - axis.origin;
      auto length = vec.norm();
      auto imagePos = MapDir2TextureInBox(vec);
      auto deep = shadowMap->Sample(imagePos.x(), imagePos.y());

      return ((length - epsilon * deep) < deep);
    } else {
      return true;
    }
  }

  virtual void RefreshShadowMap() {
    if (shadowMap == nullptr) {
      shadowMap = new Texture<float>(shadowMapHeight, shadowMapHeight * 6);
    }

    shadowMap->Fill(numeric_limits<float>::infinity());

    auto system = System3D::GetSystem();

    const auto shadowMapSize = shadowMap->Size();
    const float xInverse = 1.0f / shadowMapSize.x(),
                yInverse = 1.0f / shadowMapSize.y();
    Ray ray;
    ray.origin = axis.origin;
    for (uint32_t x = 0; x < shadowMapSize.x(); ++x) {
      for (uint32_t y = 0; y < shadowMapSize.y(); ++y) {
        const Vector2f imagePos(x * xInverse, y * yInverse); // [0,1]
        ShadowMapPixel(imagePos, ray, x, y);
      }
    }
  }

  virtual void RefreshShadowMapMultiThreads() {
    if (shadowMap == nullptr) {
      shadowMap = new Texture<float>(shadowMapHeight, shadowMapHeight * 6);
    }

    shadowMap->Fill(numeric_limits<float>::infinity());

    uint32_t runningThreads = 0;
    const uint32_t maxThreads = 8;
    vector<thread *> threads;

    const uint32_t yStep = 64;
    uint32_t yCount = yStep;
    const auto shadowMapSize = shadowMap->Size();

    while (yCount < shadowMapSize.y()) {
      if (runningThreads < maxThreads) {
        auto tP = new thread(&Light::ShadowMapPart, this,
                             Vector2i(yCount - yStep, yCount), &runningThreads);
        runningThreads++;
        threads.push_back(tP);
        yCount += yStep;
      } else {
        this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    auto tP = new thread(&Light::ShadowMapPart, this,
                         Vector2i(yCount - yStep, shadowMapSize.y()),
                         &runningThreads);
    threads.push_back(tP);

    for (auto &tP : threads) {
      tP->join();
      delete tP;
    }
  }

protected:
  virtual void ShadowMapPart(const Vector2i &yRange, uint32_t *runningThreads) {
    const auto shadowMapSize = shadowMap->Size();
    const float xInverse = 1.0f / shadowMapSize.x(),
                yInverse = 1.0f / shadowMapSize.y();
    Ray ray;
    ray.origin = axis.origin;
    for (uint32_t x = 0; x < shadowMapSize.x(); ++x) {
      for (uint32_t y = yRange.x(); y < yRange.y(); ++y) {
        const Vector2f imagePos(x * xInverse, y * yInverse); // [0,1]
        ShadowMapPixel(imagePos, ray, x, y);
      }
    }

    (*runningThreads)--;
  }

  virtual void ShadowMapPixel(const Vector2f &imagePos, Ray &ray,
                              const uint32_t x, const uint32_t y) {
    auto dir = MapTexture2DirInBox(imagePos);
    ray.dir = dir;
    auto trias = System3D::GetSystem()->GetTriasFromBVH(ray);
    for (auto triaP : trias) {
      auto triaPos = triaP->RayIntersect(ray);
      if (ray.deep < shadowMap->Sample(imagePos.x(), imagePos.y())) {
        shadowMap->SetData(x, y, ray.deep);
      }
    }
  }
};

struct PointLight : public Light {
public:
};

#endif // !_LIGHT_HPP_