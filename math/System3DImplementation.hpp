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
    auto node = nodes.top();
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
  // if (!intersectedNodes.empty()) {
  //   trias.push_back(intersectedNodes[0]->ContainedTriaRef());
  // }
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
  ray.dir = Vector3f(current_y, current_x, activeCamera->nearplane_distance)
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
            triaP->material->SampleEmitionColor(normal, ray.dir, uv);

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

        color += diffuseColor.cwiseProduct(skyBox.emitionColor);

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

      color += diffuseColor.cwiseProduct(skyBox.emitionColor);

      const float alpha = transTriainfo.alpha;
      const Vector4f originColor = System3D::BufferColor(x, y);
      const Vector4f newColor = Vector4f(color.x(), color.y(), color.z(), 1.0f);
      Vector4f emitionColor = Vector4f::Zero();
      emitionColor.block<3, 1>(0, 0) =
          triaP->material->SampleEmitionColor(normal, transTriainfo.rayDir, uv);
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
  auto trias = _GetTriasFromBVH(ray);
  for (auto triaP : trias) {
    auto triaPos = triaP->RayIntersect(ray);
    if (ray.deep < closestTriaDeep) {
      closestTriaP = triaP;
      closestTriaPos = triaPos;
      closestTriaDeep = ray.deep;
    }
  }
  ray.deep = closestTriaDeep;

  // Sample color
  if (closestTriaP != nullptr) {
    const auto pos = ray.deep * ray.dir + ray.origin;
    const auto normal = closestTriaP->VaryingNormal(closestTriaPos);
    const auto uv = closestTriaP->VaryingUV(closestTriaPos);

    ray.sampleInfo.uv = uv;
    ray.sampleInfo.material = closestTriaP->material;
    ray.sampleInfo.inDir = ray.dir;
    ray.sampleInfo.normal = normal;
    ray.sampleInfo.pos = pos;
  }
}

// Ray tracing
void System3D::_SampleTrianglesInOnePixelRayTracing(
    const Matrix4f &toView, const float current_x, const float current_y,
    const uint32_t x, const uint32_t y, const float nearplane_height_step,
    const float nearplane_width_step) {

  Vector3f color = Vector3f::Zero();
  Vector3f normalColor = Vector3f::Zero();
  Vector3f albedoColor = Vector3f::Zero();
  for (uint32_t i = 0; i < pixelSampleTimes; i++) {
    Ray originRay;

    if constexpr (IS_PERSPECTIVE_PROJECT) {
      originRay.dir =
          Vector3f(current_y + nearplane_width_step * random_0_to_1(),
                   current_x + nearplane_height_step * random_0_to_1(),
                   activeCamera->nearplane_distance)
              .normalized();
    } else {
      originRay.dir = {0.0f, 0.0f, 1.0f};
      originRay.origin = {current_y + nearplane_width_step * random_0_to_1(),
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
          ray.sampleInfo.weight > 0.0f) {
        _RaySample(ray);
        // const Ray ray = ray;

        // ray hitted mesh And deep is not infinity
        if (ray.IsThisRayPathHitted()) {
          // spaw new rays

          // get alpha
          const auto alpha = ray.sampleInfo.material->SampleAlpha(
              ray.sampleInfo.normal, ray.sampleInfo.inDir, ray.sampleInfo.uv);

          // alpha
          if (alpha < 1.0f - 1e-8f) {
            Ray rayCopy = ray; // new ray
            SamplePointInfo alphaRayInfo = rayCopy.sampleInfo;

            rayCopy.origin = alphaRayInfo.pos;
            rayCopy.dir = alphaRayInfo.inDir;

            alphaRayInfo.weight *= (1.0f - alpha);
            alphaRayInfo.color = Vector3f::Ones();
            alphaRayInfo.emition = Vector3f::Zero();

            alphaRayInfo.sampleDeep -= 1;
            infos.PushSamplePointInfo(alphaRayInfo, rayCopy);
            rays.push(rayCopy);
          }

          if (alpha > 0.0f) {
            // emission
            {
              Ray rayCopy = ray;
              SamplePointInfo emissionRayInfo = rayCopy.sampleInfo;

              emissionRayInfo.emition =
                  emissionRayInfo.material->SampleEmitionColor(
                      emissionRayInfo.normal, emissionRayInfo.inDir,
                      emissionRayInfo.uv);

              infos.PushSamplePointInfo(emissionRayInfo, rayCopy);
            }

            // reflection
            {
              Ray rayCopy = ray;
              SamplePointInfo reflectionRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              auto newWeight = reflectionRayInfo.weight;
              auto newDir = reflectionRayInfo.material->SampleReflectionOutDir(
                  reflectionRayInfo.normal, reflectionRayInfo.inDir, newWeight);
              reflectionRayInfo.weight = newWeight * alpha;

              rayCopy.origin = reflectionRayInfo.pos;
              rayCopy.dir = newDir;

              reflectionRayInfo.color =
                  reflectionRayInfo.material->SampleSpecularColor(
                      reflectionRayInfo.normal, reflectionRayInfo.inDir,
                      reflectionRayInfo.uv);

              infos.PushSamplePointInfo(reflectionRayInfo, rayCopy);
              rays.push(rayCopy);
            }

            // diffuse
            if (ray.sampleInfo.material->transparency < 1.0f) {
              Ray rayCopy = ray;
              SamplePointInfo diffuseRayInfo = rayCopy.sampleInfo;

              // sample new ray dir
              auto newWeight = diffuseRayInfo.weight;
              auto newDir = diffuseRayInfo.material->SampleDiffuseOutDir(
                  diffuseRayInfo.normal, diffuseRayInfo.inDir, newWeight);
              diffuseRayInfo.weight = newWeight * alpha;

              rayCopy.origin = diffuseRayInfo.pos;
              rayCopy.dir = newDir;

              diffuseRayInfo.color =
                  diffuseRayInfo.material->SampleDiffuseColor(
                      diffuseRayInfo.normal, diffuseRayInfo.inDir,
                      diffuseRayInfo.uv);

              infos.PushSamplePointInfo(diffuseRayInfo, rayCopy);
              rays.push(rayCopy);
            }

            // transmision
            if (ray.sampleInfo.material->transparency > 0.0f) {
              Ray rayCopy = ray;
              auto transmisionRayInfo = rayCopy.sampleInfo;

              auto newWeight = transmisionRayInfo.weight;
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

              infos.PushSamplePointInfo(transmisionRayInfo, rayCopy);
              rays.push(rayCopy);
            }
          }
        }

        // hitted sky
        else {
          Ray rayCopy = ray;
          auto skyRayInfo = rayCopy.sampleInfo;
          skyRayInfo.material = &skyBox;
          skyRayInfo.emition = skyBox.SampleSkyBoxColor(rayCopy.dir);
          skyRayInfo.color = skyRayInfo.emition;

          infos.PushSamplePointInfo(skyRayInfo, rayCopy);
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
  (*runninghreadCount)++;

  auto toView = System3D::ToView();
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
        _SampleTrianglesInOnePixelRayTracing(toView, current_x, current_y, x, y,
                                             nearplane_height_step,
                                             nearplane_width_step);
      } else {
        _SampleTrianglesInOnePixelWithMaterial(toView, current_x, current_y, x,
                                               y);
      }

      current_y += nearplane_width_step;
    }
    current_x += nearplane_height_step;
  }

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