#ifndef _SYSTEM3D_IMPLEMENTATION_HPP_
#define _SYSTEM3D_IMPLEMENTATION_HPP_

using namespace std;
using namespace Eigen;

bool BVHNode::IsIntersection(const Ray &ray) const {
  Vector3<Vector2f> minMaxT = {
      {-numeric_limits<float>::infinity(), numeric_limits<float>::infinity()},
      {-numeric_limits<float>::infinity(), numeric_limits<float>::infinity()},
      {-numeric_limits<float>::infinity(), numeric_limits<float>::infinity()}};

  for (uint32_t i = 0; i < 3; ++i) {
    float t1 = 0.0f, t2 = 0.0f;

    auto rayStep = ray.dir[i];
    if (abs(rayStep) > 1e-6f) {
      auto rayStart = ray.origin[i];
      auto rayStep_invert = 1.0f / rayStep;

      t1 = (boundBoxMin[i] - rayStart) * rayStep_invert;
      t2 = (boundBoxMax[i] - rayStart) * rayStep_invert;

      if (t1 > t2) {
        swap(t1, t2);
      }
      minMaxT[i][0] = t1; // min t
      minMaxT[i][1] = t2; // max t
    } else {
      // out of range
      if (ray.origin[i] > boundBoxMax[i] || ray.origin[i] < boundBoxMin[i]) {
        swap(minMaxT[i][0], minMaxT[i][1]);
        break;
      }
    }
  }

  auto maxT = min(min(minMaxT[0][1], minMaxT[1][1]), minMaxT[2][1]),
       minT = max(max(minMaxT[0][0], minMaxT[1][0]), minMaxT[2][0]);

  if (maxT + 1e-6f > minT && maxT > 0) {
    return true;
  } else {
    return false;
  }
}

void BVHNode::DrawBVHFramwork() const {
  auto MVP = System3D::GetMVP();
  auto projectedMax = TransformForPos(boundBoxMax, MVP);
  auto projectedMin = TransformForPos(boundBoxMin, MVP);

  Line2D line1(projectedMin.block<2, 1>(0, 0), projectedMax.block<2, 1>(0, 0));
  line1.Draw();
}

System3D::~System3D() {
  delete[] zBuffer;
  delete[] colorBuffer;

  if (activeCamera != nullptr) {
    delete activeCamera;
  }

  if (bvhRoot != nullptr) {
    delete bvhRoot;
  }
}

void System3D::_RefreshShadowMap() {
  for (auto lightP : lightsRef) {
    lightP->RefreshShadowMapMultiThreads();
  }
}

void System3D::_BuildBVH() {
  if (bvhRoot != nullptr) {
    delete bvhRoot;
  }

  struct Part {
    vector<const Triangle *> trias;
    BVHNode *currentNode = nullptr;
  };

  stack<Part> partsStack;
  bvhRoot = new BVHNode;
  partsStack.push({trianglesRef, bvhRoot});

  while (!partsStack.empty()) {
    Part part = partsStack.top();
    partsStack.pop();

    // Compute current node bounding box
    auto minMax = GetPrimsBoundBox(part.trias);
    part.currentNode->SetBoundBox(minMax);

    auto triasNum = part.trias.size();
    if (triasNum > 1) {
      vector<const Triangle *> trias1, trias2;
      uint32_t trias1Num = static_cast<uint32_t>(triasNum / 2);

      auto boxSize = minMax[1] - minMax[0];
      int maxIndex = 0;
      double maxEdge = boxSize.maxCoeff(&maxIndex);
      auto sortFunc = [maxIndex](const Triangle *tria1,
                                 const Triangle *tria2) -> bool {
        return tria1->GetBoundBox()[1][maxIndex] <
               tria2->GetBoundBox()[1][maxIndex];
      };
      sort(part.trias.begin(), part.trias.end(), sortFunc);
      for (uint32_t i1 = 0; i1 < trias1Num; ++i1) {
        trias1.push_back(part.trias[i1]);
      }
      for (uint32_t i2 = trias1Num; i2 < triasNum; ++i2) {
        trias2.push_back(part.trias[i2]);
      }

      BVHNode *next1 = new BVHNode, *next2 = new BVHNode;
      part.currentNode->SetNextNode1(next1);
      part.currentNode->SetNextNode2(next2);
      partsStack.push({trias1, next1});
      partsStack.push({trias2, next2});

    } else {
      part.currentNode->SetContainedTriaRef(part.trias[0]);
    }
  }
}

vector<const Triangle *> System3D::_GetTriasFromBVH(const Ray &ray) const {
  vector<const BVHNode *> intersectedNodes;
  stack<const BVHNode *> nodes;
  nodes.push(bvhRoot);
  while (!nodes.empty()) {
    const auto node = nodes.top();
    nodes.pop();
    if (node->IsIntersection(ray)) {
      if (node->IsLeaf()) {
        intersectedNodes.push_back(node);
      } else {
        nodes.push(node->NextNode1());
        nodes.push(node->NextNode2());
      }
    }
  }

  auto rayOrigin = ray.origin;
  auto sortFunc = [rayOrigin](const BVHNode *node1,
                              const BVHNode *node2) -> bool {
    return (node1->Center() - rayOrigin).squaredNorm() <
           (node2->Center() - rayOrigin).squaredNorm();
  };
  sort(intersectedNodes.begin(), intersectedNodes.end(), sortFunc);

  vector<const Triangle *> trias;
  for (auto nodeP : intersectedNodes) {
    trias.push_back(nodeP->ContainedTriaRef());
  }

  return trias;
}

vector<const Triangle *> System3D::_GetTriasWithEmission() const {
  vector<const Triangle *> trias;
  for (const auto triaP : trianglesRef) {
    if (triaP->material->emissionColor.squaredNorm() > 1e-6f) {
      trias.push_back(triaP);
    }
  }
  return trias;
}

void System3D::_DrawTrianglesInOnePixel(const Matrix4f &toView,
                                        const float current_x,
                                        const float current_y, const uint32_t x,
                                        const uint32_t y) {
  Ray ray;
  ray.dir = Vector3f(current_y, current_x, activeCamera->nearplane_distance)
                .normalized();
  ray.Transform(toView);

  for (auto triaP : trianglesRef) {
    auto color = triaP->HitColor(ray);

    System3D::SetBufferColorWithZ(x, y, ray.deep, color);
  }
}

// Rasterization
void System3D::_SampleTrianglesInOnePixelWithMaterial(const Matrix4f &toView,
                                                      const float current_x,
                                                      const float current_y,
                                                      const uint32_t x,
                                                      const uint32_t y) {
  Ray ray;
  ray.dir = Vector3f(-current_y, current_x, activeCamera->nearplane_distance)
                .normalized();
  ray.Transform(toView);

  auto skyBoxColor = skyBox.SampleSkyBoxColor(ray.dir);
  System3D::SetBufferColor(
      x, y, Vector4f(skyBoxColor.x(), skyBoxColor.y(), skyBoxColor.z(), 1.0f));

#pragma region struct TransTriaInfo
  struct TransTriaInfo {
    TransTriaInfo(const TransTriaInfo &other)
        : triaP(other.triaP), alpha(other.alpha), deep(other.deep),
          rayDir(other.rayDir), hitPos(other.hitPos), uv(other.uv),
          normal(other.normal){};

    TransTriaInfo() = default;

    TransTriaInfo(const Triangle *const triaP, const float alpha,
                  const float deep, const Vector3f rayDir,
                  const Vector3f hitPos, const Vector2f uv,
                  const Vector3f normal)
        : triaP(triaP), alpha(alpha), deep(deep), rayDir(rayDir),
          hitPos(hitPos), uv(uv), normal(normal){};

    bool operator>(const TransTriaInfo &other) const {
      return deep > other.deep;
    }
    bool operator<(const TransTriaInfo &other) const {
      return deep < other.deep;
    }
    bool operator==(const TransTriaInfo &other) const {
      return deep == other.deep;
    }

    static bool CompareFunc(const TransTriaInfo &item1,
                            const TransTriaInfo &item2) {
      return item1.deep > item2.deep;
    };

    const Triangle *triaP;
    float alpha;
    float deep;
    Vector3f rayDir;
    Vector3f hitPos;
    Vector2f uv;
    Vector3f normal;
  };
#pragma endregion

  auto trias = _GetTriasFromBVH(ray);
  vector<TransTriaInfo> transTriainfos;
  // pass 1
  for (auto triaP : trias) {
    auto triaPos = triaP->RayIntersect(ray);
    if (ray.deep < System3D::ZBuffer(x, y)) {
      const auto pos = ray.deep * ray.dir + ray.origin;
      const auto normal = triaP->VaryingNormal(triaPos);
      const auto uv = triaP->VaryingUV(triaPos);

      auto alpha = triaP->material->SampleAlpha(normal, ray.dir, uv);
      // if not transmission
      if (alpha > 1.0f - 1e-6f) {
        auto diffuseColor =
            triaP->material->SampleDiffuseColor(normal, ray.dir, uv);
        auto specularColor =
            triaP->material->SampleSpecularColor(normal, ray.dir, uv);

        Vector3f color =
            triaP->material->SampleEmissionColor(normal, ray.dir, uv);

        for (auto light : lightsRef) {
          // if not in shadow
          if (light->CanLighting(pos)) {
            auto lightRay = pos - light->axis.origin;
            auto lightRayLen = lightRay.norm();
            auto lightRayDir = lightRay / lightRayLen;

            auto cosNormalLight = -lightRayDir.dot(normal);
            if (cosNormalLight > 0) {
              auto lightLuminance =
                  light->intensity / (lightRayLen * lightRayLen);
              // diffuse
              color +=
                  cosNormalLight * diffuseColor.cwiseProduct(lightLuminance);
              // specular
              color +=
                  pow(-(lightRayDir + ray.dir).normalized().dot(normal), 60) *
                  specularColor.cwiseProduct(lightLuminance);
            }
          }
        }

        color += diffuseColor.cwiseProduct(skyBox.emissionColor);

        // Set pixel color
        System3D::SetBufferColor(
            x, y, Vector4f(color.x(), color.y(), color.z(), 1.0f));
        System3D::SetZBuffer(x, y, ray.deep);

        system->albedoBuffer[IndFromXY(x, y)] = Vector4f(
            diffuseColor.x(), diffuseColor.y(), diffuseColor.z(), 1.0f);
        system->normalBuffer[IndFromXY(x, y)] =
            Vector4f(normal.x(), normal.y(), normal.z(), 1.0f);

      } else {
        transTriainfos.emplace_back(triaP, alpha, ray.deep, ray.dir, pos, uv,
                                    normal);
      }
    }
  }

  // pass 2: transmission
  std::sort(transTriainfos.begin(), transTriainfos.end(),
            TransTriaInfo::CompareFunc);
  for (const auto &transTriainfo : transTriainfos) {
    if (transTriainfo.deep < System3D::ZBuffer(x, y)) {

      const auto triaP = transTriainfo.triaP;
      const auto normal = transTriainfo.normal;
      const auto uv = transTriainfo.uv;
      const auto pos = transTriainfo.hitPos;

      const Vector3f diffuseColor =
          triaP->material->SampleDiffuseColor(normal, transTriainfo.rayDir, uv);
      const Vector3f specularColor = triaP->material->SampleSpecularColor(
          normal, transTriainfo.rayDir, uv);

      Vector3f color = Vector3f::Zero();

      for (auto light : lightsRef) {
        // if not in shadow
        if (light->CanLighting(pos)) {
          auto lightRay = pos - light->axis.origin;
          auto lightRayLen = lightRay.norm();
          auto lightRayDir = lightRay / lightRayLen;

          const float cosNormalLight = -lightRayDir.dot(normal);
          if (cosNormalLight > 0) {
            const Vector3f lightLuminance =
                light->intensity / (lightRayLen * lightRayLen);
            // diffuse
            color +=
                cosNormalLight * (diffuseColor.cwiseProduct(lightLuminance));
            // specular
            color +=
                pow(-(lightRayDir + ray.dir).normalized().dot(normal), 60) *
                specularColor.cwiseProduct(lightLuminance);
          }
        }
      }

      color += diffuseColor.cwiseProduct(skyBox.emissionColor);

      const float alpha = transTriainfo.alpha;
      const Vector4f originColor = System3D::BufferColor(x, y);
      const Vector4f newColor = Vector4f(color.x(), color.y(), color.z(), 1.0f);
      Vector4f emitionColor = Vector4f::Zero();
      emitionColor.block<3, 1>(0, 0) = triaP->material->SampleEmissionColor(
          normal, transTriainfo.rayDir, uv);
      const Vector4f blendColor =
          originColor * (1.0f - alpha) + newColor * alpha + emitionColor;

      System3D::SetBufferColor(x, y, blendColor);
    }
  }
}

void System3D::_RaySample(Ray &ray) {
  const Triangle *closestTriaP = nullptr;
  Vector3f closestTriaPos;
  float closestTriaDeep = numeric_limits<float>::infinity();
  const auto trias = _GetTriasFromBVH(ray);
  // get closest triangle
  for (const Triangle *const triaP : trias) {
    if (triaP != ray.sampleInfo.triaP) {
      const Vector3f triaPos = triaP->RayIntersect(ray);
      if (ray.deep < closestTriaDeep) {
        closestTriaP = triaP;
        closestTriaPos = triaPos;
        closestTriaDeep = ray.deep;
      }
    }
  }

  ray.deep = closestTriaDeep;
  ray.sampleInfo.triaP = closestTriaP;

  // Sample on triangle
  if (closestTriaP != nullptr) {
    // const auto pos = closestTriaDeep * ray.dir + ray.origin;
    const auto pos = closestTriaP->VaryingPos(closestTriaPos);
    const auto normal = closestTriaP->VaryingNormal(closestTriaPos);
    const auto uv = closestTriaP->VaryingUV(closestTriaPos);

    ray.sampleInfo.material = closestTriaP->material;

    ray.sampleInfo.uv = uv;
    ray.sampleInfo.inDir = ray.dir;
    ray.sampleInfo.normal = normal;
    // be careful pos may be replaced in release
    ray.sampleInfo.pos = pos;
  }
}

// Path tracing
void System3D::_SampleTrianglesInOnePixelRayTracing(
    const Matrix4f &toView, const float current_x, const float current_y,
    const uint32_t x, const uint32_t y, const float nearplane_height_step,
    const float nearplane_width_step) {

  Vector3f color = Vector3f::Zero();
  Vector3f normalColor = Vector3f::Zero();
  Vector3f albedoColor = Vector3f::Zero();
  for (uint32_t sampleTime = 0; sampleTime < pixelSampleTimes; sampleTime++) {
    Ray originRay;

    if constexpr (IS_PERSPECTIVE_PROJECT) {
      originRay.dir =
          Vector3f(-(current_y + nearplane_width_step * random_0_to_1()),
                   current_x + nearplane_height_step * random_0_to_1(),
                   activeCamera->nearplane_distance)
              .normalized();
    } else {
      originRay.dir = {0.0f, 0.0f, 1.0f};
      originRay.origin = {-(current_y + nearplane_width_step * random_0_to_1()),
                          current_x + nearplane_height_step * random_0_to_1(),
                          0.0f};
    }
    originRay.Transform(toView);

    // Ray bounce
    SamplePointInfos infos;
    infos.sampleInfos.push_back(SamplePointInfo());
    stack<Ray> rays;
    rays.push(originRay);

    while (!rays.empty()) {
      auto ray = rays.top();
      rays.pop();

      if (ray.sampleInfo.sampleDeep < pixelSampleDeep &&
          ray.sampleInfo.weight > 1e-6f) {
        _RaySample(ray);
        // const Ray ray = ray;

        // ray hit mesh And deep is not infinity
        if (ray.IsThisRayPathHit()) {
          // spaw new rays

          // get alpha
          const auto alpha = ray.sampleInfo.material->SampleAlpha(
              ray.sampleInfo.normal, ray.sampleInfo.inDir, ray.sampleInfo.uv);

          // alpha
          if (alpha < 1.0f - 1e-6f) {
            Ray rayCopy = ray; // new ray
            SamplePointInfo alphaRayInfo = rayCopy.sampleInfo;

            rayCopy.origin = alphaRayInfo.pos;
            rayCopy.dir = alphaRayInfo.inDir;

            alphaRayInfo.weight = (1.0f - alpha);
            alphaRayInfo.color = Vector3f::Ones();
            alphaRayInfo.emition = Vector3f::Zero();

            alphaRayInfo.sampleDeep.alphaSample += 1;
            infos.PushSamplePointInfo(alphaRayInfo, rayCopy, RayType::ALPHA);
            if (alphaRayInfo.sampleDeep.alphaSample <
                pixelSampleDeep.alphaSample) {
              rays.push(rayCopy);
            }
          }

          if (alpha > 1e-6f) {
            // emission
            {
              Ray rayCopy = ray;
              SamplePointInfo emissionRayInfo = rayCopy.sampleInfo;

              emissionRayInfo.emition =
                  emissionRayInfo.material->SampleEmissionColor(
                      emissionRayInfo.normal, emissionRayInfo.inDir,
                      emissionRayInfo.uv);
              emissionRayInfo.color = Vector3f::Zero();

              emissionRayInfo.weight = alpha;

              infos.PushSamplePointInfo(emissionRayInfo, rayCopy,
                                        RayType::EMISSION);
            }

            // reflection
            {
              Ray rayCopy = ray;
              SamplePointInfo reflectionRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              float newWeight = 0.0f;
              auto newDir = reflectionRayInfo.material->SampleReflectionOutDir(
                  reflectionRayInfo.normal, reflectionRayInfo.inDir, newWeight);
              reflectionRayInfo.weight = newWeight * alpha;

              rayCopy.origin = reflectionRayInfo.pos;
              rayCopy.dir = newDir;

              reflectionRayInfo.color =
                  reflectionRayInfo.material->SampleSpecularColor(
                      reflectionRayInfo.normal, reflectionRayInfo.inDir,
                      reflectionRayInfo.uv);

              reflectionRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(reflectionRayInfo, rayCopy,
                                        RayType::REFLECTION);
              if (reflectionRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }

            // diffuse
            if (ray.sampleInfo.material->transparency < 1.0f) {
              Ray rayCopy = ray;
              SamplePointInfo diffuseRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              auto newWeight = 0.0f;
              auto newDir = diffuseRayInfo.material->SampleDiffuseOutDir(
                  diffuseRayInfo.normal, diffuseRayInfo.inDir, newWeight);
              // diffuseRayInfo.weight = newWeight * alpha;

              rayCopy.origin = diffuseRayInfo.pos;
              rayCopy.dir = newDir;

              diffuseRayInfo.color =
                  diffuseRayInfo.material->SampleDiffuseColor(
                      diffuseRayInfo.normal, diffuseRayInfo.inDir,
                      diffuseRayInfo.uv);

              diffuseRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(diffuseRayInfo, rayCopy,
                                        RayType::DIFFUSE);
              if (diffuseRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }

            // transmision
            if (ray.sampleInfo.material->transparency > 1e-6f) {
              Ray rayCopy = ray;
              auto transmisionRayInfo = rayCopy.sampleInfo;

              auto newWeight = 0.0f;
              auto newDir =
                  transmisionRayInfo.material->SampleTransmissionOutDir(
                      transmisionRayInfo.normal, transmisionRayInfo.inDir,
                      newWeight);
              transmisionRayInfo.weight = newWeight * alpha;

              rayCopy.origin = transmisionRayInfo.pos;
              rayCopy.dir = newDir;

              transmisionRayInfo.color =
                  transmisionRayInfo.material->SampleTransmissionColor(
                      transmisionRayInfo.normal, transmisionRayInfo.inDir,
                      transmisionRayInfo.uv);

              transmisionRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(transmisionRayInfo, rayCopy,
                                        RayType::REFRACTION);
              if (transmisionRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }
          }
        }

        // hit sky
        else {
          Ray rayCopy = ray;
          auto skyRayInfo = rayCopy.sampleInfo;
          skyRayInfo.material = &skyBox;
          skyRayInfo.emition = skyBox.SampleSkyBoxColor(rayCopy.dir);
          skyRayInfo.color = skyRayInfo.emition;

          skyRayInfo.weight = 1.0f;

          infos.PushSamplePointInfo(skyRayInfo, rayCopy, RayType::SKYBOX);
        }
      }
    }

    // Ray color
    normalColor += infos.HitNormal();
    albedoColor += infos.HitAlbedo();
    color += infos.AccColor();

  }
  color /= static_cast<float>(pixelSampleTimes);
  normalColor /= static_cast<float>(pixelSampleTimes);
  albedoColor /= static_cast<float>(pixelSampleTimes);

  // Set pixel color
  System3D::SetBufferColor(x, y,
                           Vector4f(color.x(), color.y(), color.z(), 1.0f));

  system->albedoBuffer[IndFromXY(x, y)] =
      Vector4f(albedoColor.x(), albedoColor.y(), albedoColor.z(), 1.0f);
  system->normalBuffer[IndFromXY(x, y)] =
      Vector4f(normalColor.x(), normalColor.y(), normalColor.z(), 1.0f);
}

// Path tracing with light sample
void System3D::_SampleTrianglesInOnePixelPathTracingLightSample(
    const Matrix4f &toView, const float current_x, const float current_y,
    const uint32_t x, const uint32_t y, const float nearplane_height_step,
    const float nearplane_width_step) {

  Vector3f color = Vector3f::Zero();
  Vector3f normalColor = Vector3f::Zero();
  Vector3f albedoColor = Vector3f::Zero();
  for (uint32_t sampleTime = 0; sampleTime < pixelSampleTimes; sampleTime++) {
    Ray originRay;

    if constexpr (IS_PERSPECTIVE_PROJECT) {
      originRay.dir =
          Vector3f(-(current_y + nearplane_width_step * random_0_to_1()),
                   current_x + nearplane_height_step * random_0_to_1(),
                   activeCamera->nearplane_distance)
              .normalized();
    } else {
      originRay.dir = {0.0f, 0.0f, 1.0f};
      originRay.origin = {-(current_y + nearplane_width_step * random_0_to_1()),
                          current_x + nearplane_height_step * random_0_to_1(),
                          0.0f};
    }
    originRay.Transform(toView);

    // Ray bounce
    SamplePointInfos infos;
    infos.sampleInfos.push_back(SamplePointInfo());
    stack<Ray> rays;
    rays.push(originRay);

    while (!rays.empty()) {
      auto ray = rays.top();
      rays.pop();

      if (ray.sampleInfo.sampleDeep < pixelSampleDeep &&
          ray.sampleInfo.weight > 1e-6f) {
        _RaySample(ray);
        // const Ray ray = ray;

        // ray hit mesh And deep is not infinity
        if (ray.IsThisRayPathHit()) {
          // spaw new rays

          // get alpha
          const auto alpha = ray.sampleInfo.material->SampleAlpha(
              ray.sampleInfo.normal, ray.sampleInfo.inDir, ray.sampleInfo.uv);

          // alpha
          if (alpha < 1.0f - 1e-6f) {
            Ray rayCopy = ray; // new ray
            SamplePointInfo alphaRayInfo = rayCopy.sampleInfo;

            rayCopy.origin = alphaRayInfo.pos;
            rayCopy.dir = alphaRayInfo.inDir;

            alphaRayInfo.weight = (1.0f - alpha);
            alphaRayInfo.color = Vector3f::Ones();
            alphaRayInfo.emition = Vector3f::Zero();

            alphaRayInfo.sampleDeep.alphaSample += 1;
            infos.PushSamplePointInfo(alphaRayInfo, rayCopy, RayType::ALPHA);
            if (alphaRayInfo.sampleDeep.alphaSample <
                pixelSampleDeep.alphaSample) {
              rays.push(rayCopy);
            }
          }

          if (alpha > 0.0f) {

            // emission
            if (ray.sampleInfo.rayType != RayType::DIFFUSE) {
              Ray rayCopy = ray;
              SamplePointInfo emissionRayInfo = rayCopy.sampleInfo;

              emissionRayInfo.emition =
                  emissionRayInfo.material->SampleEmissionColor(
                      emissionRayInfo.normal, emissionRayInfo.inDir,
                      emissionRayInfo.uv);
              emissionRayInfo.color = Vector3f::Zero();

              emissionRayInfo.weight = alpha;

              infos.PushSamplePointInfo(emissionRayInfo, rayCopy,
                                        RayType::EMISSION);
            }

            // light sample
            Vector3f emissionAcc = Vector3f::Zero();
            {
              Ray rayCopy = ray;

              for (const auto triaP : trianglesEmission) {
                const auto samplePos = triaP->ScatterPoint();

                Ray sampleEmissionRay = rayCopy;

                const Vector3f vec = samplePos - rayCopy.sampleInfo.pos;
                sampleEmissionRay.dir = vec.normalized();
                sampleEmissionRay.origin = rayCopy.sampleInfo.pos;

                // inDir and smaple Dir is one side
                if (rayCopy.sampleInfo.normal.dot(rayCopy.dir) *
                        rayCopy.sampleInfo.normal.dot(sampleEmissionRay.dir) <
                    0.0f) {
                  _RaySample(sampleEmissionRay);
                }

                if (sampleEmissionRay.IsThisRayPathHit() &&
                    sampleEmissionRay.sampleInfo.triaP == triaP) {

                  const float cosTheta =
                      abs(rayCopy.sampleInfo.normal.dot(sampleEmissionRay.dir));

                  const float cosTheta2 =
                      abs(sampleEmissionRay.sampleInfo.normal.dot(
                          sampleEmissionRay.dir));

                  const Vector3f diffColor =
                      rayCopy.sampleInfo.material->SampleDiffuseColor(
                          rayCopy.sampleInfo.normal, rayCopy.sampleInfo.inDir,
                          rayCopy.sampleInfo.uv);

                  const Vector3f emissionColor =
                      triaP->material->SampleEmissionColor(
                          sampleEmissionRay.sampleInfo.normal,
                          sampleEmissionRay.sampleInfo.inDir,
                          sampleEmissionRay.sampleInfo.uv);

                  emissionAcc += triaP->material->alpha * cosTheta * cosTheta2 *
                                 triaP->Area() *
                                 emissionColor.cwiseProduct(diffColor) /
                                 vec.squaredNorm();
                }
              }
            }

            // reflection
            {
              Ray rayCopy = ray;
              SamplePointInfo reflectionRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              float newWeight = 0.0f;
              auto newDir = reflectionRayInfo.material->SampleReflectionOutDir(
                  reflectionRayInfo.normal, reflectionRayInfo.inDir, newWeight);
              reflectionRayInfo.weight = newWeight * alpha;

              rayCopy.origin = reflectionRayInfo.pos;
              rayCopy.dir = newDir;

              reflectionRayInfo.color =
                  reflectionRayInfo.material->SampleSpecularColor(
                      reflectionRayInfo.normal, reflectionRayInfo.inDir,
                      reflectionRayInfo.uv);

              reflectionRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(reflectionRayInfo, rayCopy,
                                        RayType::REFLECTION);
              if (reflectionRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }

            // diffuse
            if (ray.sampleInfo.material->transparency < 1.0f) {
              Ray rayCopy = ray;
              SamplePointInfo diffuseRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              auto newWeight = 0.0f;
              auto newDir = diffuseRayInfo.material->SampleDiffuseOutDir(
                  diffuseRayInfo.normal, diffuseRayInfo.inDir, newWeight);
              diffuseRayInfo.weight = newWeight * alpha;

              rayCopy.origin = diffuseRayInfo.pos;
              rayCopy.dir = newDir;

              diffuseRayInfo.color =
                  diffuseRayInfo.material->SampleDiffuseColor(
                      diffuseRayInfo.normal, diffuseRayInfo.inDir,
                      diffuseRayInfo.uv);

              diffuseRayInfo.emition = emissionAcc;

              diffuseRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(diffuseRayInfo, rayCopy,
                                        RayType::DIFFUSE);
              if (diffuseRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }

            // transmision
            if (ray.sampleInfo.material->transparency > 1e-6f) {
              Ray rayCopy = ray;
              auto transmisionRayInfo = rayCopy.sampleInfo;

              auto newWeight = 0.0f;
              auto newDir =
                  transmisionRayInfo.material->SampleTransmissionOutDir(
                      transmisionRayInfo.normal, transmisionRayInfo.inDir,
                      newWeight);
              transmisionRayInfo.weight = newWeight * alpha;

              rayCopy.origin = transmisionRayInfo.pos;
              rayCopy.dir = newDir;

              transmisionRayInfo.color =
                  transmisionRayInfo.material->SampleTransmissionColor(
                      transmisionRayInfo.normal, transmisionRayInfo.inDir,
                      transmisionRayInfo.uv);
              transmisionRayInfo.emition = Vector3f::Zero();

              transmisionRayInfo.sampleDeep.bsdfSample += 1;
              infos.PushSamplePointInfo(transmisionRayInfo, rayCopy,
                                        RayType::REFRACTION);
              if (transmisionRayInfo.sampleDeep.bsdfSample <
                  pixelSampleDeep.bsdfSample) {
                rays.push(rayCopy);
              }
            }
          }
        }

        // hit sky
        else {
          Ray rayCopy = ray;
          auto skyRayInfo = rayCopy.sampleInfo;
          skyRayInfo.material = &skyBox;
          skyRayInfo.emition = skyBox.SampleSkyBoxColor(rayCopy.dir);
          skyRayInfo.color = skyRayInfo.emition;

          skyRayInfo.weight = 1.0f;

          infos.PushSamplePointInfo(skyRayInfo, rayCopy, RayType::SKYBOX);
        }
      }
    }

    // Ray color
    color += infos.AccColor();
    normalColor += infos.HitNormal();
    albedoColor += infos.HitAlbedo();
  }
  color /= static_cast<float>(pixelSampleTimes);
  normalColor /= static_cast<float>(pixelSampleTimes);
  albedoColor /= static_cast<float>(pixelSampleTimes);

  // Set pixel color
  System3D::SetBufferColor(x, y,
                           Vector4f(color.x(), color.y(), color.z(), 1.0f));

  system->albedoBuffer[IndFromXY(x, y)] =
      Vector4f(albedoColor.x(), albedoColor.y(), albedoColor.z(), 1.0f);
  system->normalBuffer[IndFromXY(x, y)] =
      Vector4f(normalColor.x(), normalColor.y(), normalColor.z(), 1.0f);
}

void System3D::_DrawSplines() {
  for (auto splineP : splinesRef) {
    splineP->DrawToBuffer();
  }
}

void System3D::_DrawTrianglesInRangePixel(const Vector2i &xRange,
                                          const Vector2i &yRange,
                                          int *const runninghreadCount) {

  const auto toView = System3D::ToView();
  const float window_x_i = 1.0f / window_x, window_y_i = 1.0f / window_y;

  const float nearplane_width_step = activeCamera->nearplane_width * window_y_i,
              nearplane_height_step =
                  activeCamera->nearplane_height * window_x_i;
  float current_x = activeCamera->nearplane_height * -0.5f,
        start_y = activeCamera->nearplane_width * -0.5f;

  current_x += xRange.x() * nearplane_height_step;
  start_y += yRange.x() * nearplane_width_step;
  for (int x = xRange.x(); x < xRange.y(); ++x) {
    auto current_y = start_y;
    for (int y = yRange.x(); y < yRange.y(); ++y) {

      // Sample
      if constexpr (IS_RAY_TRACING) {
        //_SampleTrianglesInOnePixelRayTracing(toView, current_x, current_y, x,
        // y,
        //                                     nearplane_height_step,
        //                                     nearplane_width_step);
        _SampleTrianglesInOnePixelPathTracingLightSample(
            toView, current_x, current_y, x, y, nearplane_height_step,
            nearplane_width_step);
      } else {
        _SampleTrianglesInOnePixelWithMaterial(toView, current_x, current_y, x,
                                               y);
      }

      current_y += nearplane_width_step;
    }
    current_x += nearplane_height_step;
  }

  // cout << "end Thread: " << (*runninghreadCount) << "  xCount: " <<
  // xRange.y()
  //      << endl;
  (*runninghreadCount)--;
}

void System3D::_DrawTrianglesInClip(const Primitive2D &clip) {
  auto toView = System3D::ToView();
  float window_x_i = 1.0f / window_x, window_y_i = 1.0f / window_y;

  float nearplane_width_step = activeCamera->nearplane_width * window_x_i,
        nearplane_height_step = activeCamera->nearplane_height * window_y_i;
  float current_x = activeCamera->nearplane_width * -0.5f,
        start_y = activeCamera->nearplane_height * -0.5f;

  for (uint32_t x = 0; x < window_x; ++x) {
    auto current_y = start_y;
    for (uint32_t y = 0; y < window_y; ++y) {
      if (clip.IsPointInside(Vector2f(x * window_x_i, y * window_y_i))) {
        _DrawTrianglesInOnePixel(toView, current_x, current_y, x, y);
      }

      current_y += nearplane_height_step;
    }
    current_x += nearplane_width_step;
  }
}

#endif // !_SYSTEM3D_IMPLEMENTATION_HPP_