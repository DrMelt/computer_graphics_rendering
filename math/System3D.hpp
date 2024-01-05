#ifndef _SYSTEM_3D_HPP_
#define _SYSTEM_3D_HPP_

// clang-format off
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <Eigen/Geometry>
#include <OpenImageDenoise/oidn.hpp>
// clang-format on
#include "Texture.hpp"
#include "Transform.hpp"
#include <iostream>
#include <math.h>
#include <random>
#include <string.h>
#include <thread>

using namespace std;
using namespace Eigen;

struct Triangle;
struct Spline;
struct Primitive2D;
struct Light;
struct Ray;
struct SamplePointInfo;

struct Camera {
  Vector3f eyePos = {0, 50, 5};
  Vector3f eyeUp = {0, 0, 1};
  Vector3f lookAtPos = {0, 0, 0};

  float nearplane_width = 1.0f;      // 视景体宽度
  float nearplane_height = 1.0f;     // 视景体高度
  float nearplane_distance = 1.0f;   // 视景体近平面与视点距离
  float farplane_distance = 3000.0f; // 视景体远平面与视点距离
};

struct BVHNode {
protected:
  Vector3f boundBoxMin = Vector3f::Zero();
  Vector3f boundBoxMax = Vector3f::Zero();
  Vector3f boundBoxCenter = Vector3f::Zero();
  const BVHNode *nextNode1 = nullptr, *nextNode2 = nullptr;
  // leaf node if containedTriaRef != nullptr
  const Triangle *containedTriaRef = nullptr;

  void UpdateCenter() {
    boundBoxCenter = boundBoxMin + boundBoxMax;
    boundBoxCenter *= 0.5;
  }

public:
  BVHNode() = default;
  ~BVHNode() {
    delete nextNode1;
    delete nextNode2;
  }
  bool IsLeaf() const { return (containedTriaRef != nullptr); }
  bool IsIntersection(const Ray &ray) const;
  Vector3f Center() const { return boundBoxCenter; }
  void SetBoundBoxMin(const Vector3f &boundBoxMin) {
    this->boundBoxMin = boundBoxMin;
    UpdateCenter();
  }
  void SetBoundBoxMax(const Vector3f &boundBoxMax) {
    this->boundBoxMax = boundBoxMax;
    UpdateCenter();
  }
  void SetBoundBox(const Vector2<Vector3f> &boundBox) {
    this->boundBoxMin = boundBox[0];
    this->boundBoxMax = boundBox[1];
    UpdateCenter();
  }
  void SetContainedTriaRef(const Triangle *containedTriaRef) {
    this->containedTriaRef = containedTriaRef;
  }
  const Triangle *ContainedTriaRef() const { return containedTriaRef; }
  void SetNextNode1(const BVHNode *node) { nextNode1 = node; }
  const BVHNode *NextNode1() const { return nextNode1; }
  void SetNextNode2(const BVHNode *node) { nextNode2 = node; }
  const BVHNode *NextNode2() const { return nextNode2; }

  void DrawBVHFramwork() const;
};

class System3D {
public:
  ~System3D();

  class System3DDeleter {
  public:
    System3DDeleter() = default;
    ~System3DDeleter() { delete System3D::system; }
  };

public:
  static System3D *InitSystem(const uint32_t window_x = 900,
                              const uint32_t window_y = 900) {
    if (system != nullptr) {
      delete system;
    }

    system = new System3D(window_x, window_y);
    return system;
  }

  static System3D *GetSystem() {
    if (system == nullptr) {
      assert("system is not inited");
    }
    return system;
  }

  // Project to
  // x -> [0,1]
  // y -> [0,1]
  // z -> [-1,1]
  static Matrix4f GetMVP() {
    auto activeCamera = system->activeCamera;
    return PerspectiveProjection(
               -0.5f * activeCamera->nearplane_width,
               0.5f * activeCamera->nearplane_width,
               -0.5f * activeCamera->nearplane_height,
               0.5f * activeCamera->nearplane_height,
               static_cast<float>(activeCamera->nearplane_distance),
               static_cast<float>(activeCamera->farplane_distance)) *
           LookAtMatrix(activeCamera->eyePos, activeCamera->lookAtPos,
                        activeCamera->eyeUp);
  }

  static Matrix4f GetV() {
    auto activeCamera = system->activeCamera;
    return LookAtMatrix(activeCamera->eyePos, activeCamera->lookAtPos,
                        activeCamera->eyeUp);
  }

  // rotate z axis to view lookAt
  static Matrix4f ToView() {
    auto activeCamera = system->activeCamera;
    return ToLookAtMatrix(activeCamera->eyePos, activeCamera->lookAtPos,
                          activeCamera->eyeUp);
  }

  static uint32_t IndFromXY(const uint32_t x, const uint32_t y) {
    return system->_IndFromXY(x, y);
  }

  static void DenoiseForBuffer() {
    // Create an Open Image Denoise device
    oidn::DeviceRef device = oidn::newDevice(); // CPU or GPU if available
    device.commit();

    // Create buffers for input/output images accessible by both host (CPU) and
    // device (CPU/GPU)
    oidn::BufferRef colorBuf = device.newBuffer(
        system->window_y * system->window_x * 3 * sizeof(float));
    oidn::BufferRef albedoBuf = device.newBuffer(
        system->window_y * system->window_x * 3 * sizeof(float));
    oidn::BufferRef normalBuf = device.newBuffer(
        system->window_y * system->window_x * 3 * sizeof(float));
    // Create a filter for denoising a beauty (color) image using optional
    // auxiliary images too This can be an expensive operation, so try no to
    // create a new filter for every image!
    oidn::FilterRef filter =
        device.newFilter("RT"); // generic ray tracing filter
    filter.setImage("color", colorBuf, oidn::Format::Float3, system->window_y,
                    system->window_x); // beauty
    filter.setImage("albedo", albedoBuf, oidn::Format::Float3, system->window_y,
                    system->window_x); // auxiliary
    filter.setImage("normal", normalBuf, oidn::Format::Float3, system->window_y,
                    system->window_x); // auxiliary
    filter.setImage("output", colorBuf, oidn::Format::Float3, system->window_y,
                    system->window_x); // denoised beauty
    filter.set("hdr", true);           // beauty image is HDR
    filter.commit();

    // Fill the input image buffers
    float *colorPtr = (float *)colorBuf.getData();
    float *albedoPtr = (float *)albedoBuf.getData();
    float *normalPtr = (float *)normalBuf.getData();
    for (uint32_t x = 0; x < system->window_x; ++x) {
      for (uint32_t y = 0; y < system->window_y; ++y) {
        const auto ind = IndFromXY(x, y);
        const auto indOIDN = x * system->window_y + y;
        colorPtr[indOIDN * 3] = system->colorBuffer[ind].x();
        colorPtr[indOIDN * 3 + 1] = system->colorBuffer[ind].y();
        colorPtr[indOIDN * 3 + 2] = system->colorBuffer[ind].z();

        albedoPtr[indOIDN * 3] = system->albedoBuffer[ind].x();
        albedoPtr[indOIDN * 3 + 1] = system->albedoBuffer[ind].y();
        albedoPtr[indOIDN * 3 + 2] = system->albedoBuffer[ind].z();

        normalPtr[indOIDN * 3] = system->normalBuffer[ind].x();
        normalPtr[indOIDN * 3 + 1] = system->normalBuffer[ind].y();
        normalPtr[indOIDN * 3 + 2] = system->normalBuffer[ind].z();
      }
    }

    // Filter the beauty image
    filter.execute();

    // Check for errors
    const char *errorMessage;
    if (device.getError(errorMessage) != oidn::Error::None) {
      std::cout << "Error: " << errorMessage << std::endl;
    }

    for (uint32_t x = 0; x < system->window_x; ++x) {
      for (uint32_t y = 0; y < system->window_y; ++y) {
        auto ind = IndFromXY(x, y);
        auto indOIDN = x * system->window_y + y;
        system->colorBuffer[ind].x() = colorPtr[indOIDN * 3];
        system->colorBuffer[ind].y() = colorPtr[indOIDN * 3 + 1];
        system->colorBuffer[ind].z() = colorPtr[indOIDN * 3 + 2];
      }
    }
  }

  static void ShowBuffer() {
    auto window_x = system->window_x, window_y = system->window_y;
    // float window_x_invert = 1.0 / window_x, window_y_invert = 1.0 / window_y;
    auto colorBufferP = system->colorBuffer;

    float *pixels = new float[window_x * window_y * 4];
    for (uint32_t x = 0; x < window_x; ++x) {
      for (uint32_t y = 0; y < window_y; ++y) {
        const Vector4f color = colorBufferP[IndFromXY(x, y)];
        const auto pixelInd = (x * window_y + y) * 4;
        pixels[pixelInd] = color.x();
        pixels[pixelInd + 1] = color.y();
        pixels[pixelInd + 2] = color.z();
        pixels[pixelInd + 3] = color.w();
      }
    }

    glDrawPixels(window_y, window_x, GL_RGBA, GL_FLOAT, pixels);
    delete[] pixels;
  }

  static void DrawAlbedoToBuffer() {
    auto window_x = system->window_x, window_y = system->window_y;
    float window_x_invert = 1.0 / window_x, window_y_invert = 1.0 / window_y;
    auto colorBufferP = system->colorBuffer;
    auto albedoBufferP = system->albedoBuffer;

    for (uint32_t x = 0; x < window_x; ++x) {
      for (uint32_t y = 0; y < window_y; ++y) {
        const auto ind = IndFromXY(x, y);
        colorBufferP[ind] = albedoBufferP[ind];
      }
    }
  }

  static void DrawNormalToBuffer() {
    auto window_x = system->window_x, window_y = system->window_y;
    float window_x_invert = 1.0 / window_x, window_y_invert = 1.0 / window_y;
    auto colorBufferP = system->colorBuffer;
    auto normalBufferP = system->normalBuffer;

    for (uint32_t x = 0; x < window_x; ++x) {
      for (uint32_t y = 0; y < window_y; ++y) {
        const auto ind = IndFromXY(x, y);
        colorBufferP[ind] = normalBufferP[ind];
      }
    }
  }

  static void ClearBuffer() { system->_ClearBuffer(); }

  static Vector4f BufferColor(const uint32_t x, const uint32_t y) {
    if (x >= system->window_x) {
      return Vector4f::Zero();
    }
    if (y >= system->window_y) {
      return Vector4f::Zero();
    }

    return system->colorBuffer[IndFromXY(x, y)];
  }

  static Vector4f BufferColor(const float x, const float y) {
    return BufferColor(static_cast<uint32_t>(system->window_x * x),
                       static_cast<uint32_t>(system->window_y * y));
  }

  static void SetBufferColor(const uint32_t w, const uint32_t h,
                             const Vector4f &value) {
    if (w >= system->window_x) {
      return;
    }
    if (h >= system->window_y) {
      return;
    }

    auto colorBuffer = system->colorBuffer;
    colorBuffer[IndFromXY(w, h)] = value;
  }

  static void SetBufferColor(const uint32_t w, const uint32_t h,
                             const Vector3f &value) {
    if (w >= system->window_x) {
      return;
    }
    if (h >= system->window_y) {
      return;
    }

    auto colorBuffer = system->colorBuffer;
    colorBuffer[IndFromXY(w, h)].block<3, 1>(0, 0) = value;
    colorBuffer[IndFromXY(w, h)].w() = 1.0f;
  }

  // Will use system status color
  static void SetBufferColor(const uint32_t w, const uint32_t h) {
    if (w >= system->window_x) {
      return;
    }
    if (h >= system->window_y) {
      return;
    }

    auto colorBuffer = system->colorBuffer;
    colorBuffer[IndFromXY(w, h)] = system->colorState;
  }

  // w,h [0,1]
  static void SetBufferColor(const float x, const float y,
                             const Vector4f &value) {
    System3D::SetBufferColor(static_cast<uint32_t>(system->window_x * x),
                             static_cast<uint32_t>(system->window_y * y),
                             value);
  }
  // w,h [0,1]
  static void SetBufferColor(const float x, const float y,
                             const Vector3f &value) {
    System3D::SetBufferColor(static_cast<uint32_t>(system->window_x * x),
                             static_cast<uint32_t>(system->window_y * y),
                             value);
  }

  // w,h [0,1]
  static void SetBufferColor(const float x, const float y) {
    System3D::SetBufferColor(static_cast<uint32_t>(system->window_x * x),
                             static_cast<uint32_t>(system->window_y * y));
  }

  static void SetBufferColorWithZ(const uint32_t x, const uint32_t y,
                                  const float z, const Vector4f &value) {
    auto zBuffer = system->zBuffer;
    auto ind = IndFromXY(x, y);
    if (z < zBuffer[ind] && z > 0) {
      zBuffer[ind] = z;
      System3D::SetBufferColor(x, y, value);
    } else {
      return;
    }
  }

  static void SetBufferColorWithZ(const uint32_t x, const uint32_t y,
                                  const float deep, const Vector3f &value) {
    auto zBuffer = system->zBuffer;
    auto ind = IndFromXY(x, y);
    if (deep < zBuffer[ind] && deep > 0) {
      zBuffer[ind] = deep;
      System3D::SetBufferColor(x, y, value);
    } else {
      return;
    }
  }

  static void SetBufferColorWithZ(const float x, const float y,
                                  const float deep, const Vector4f &value) {

    System3D::SetBufferColorWithZ(static_cast<uint32_t>(system->window_x * x),
                                  static_cast<uint32_t>(system->window_y * y),
                                  deep, value);
  }

  static float ZBuffer(const uint32_t x, const uint32_t y) {
    if (x >= system->window_x) {
      return 0.0f;
    }
    if (y >= system->window_y) {
      return 0.0f;
    }
    auto zBuffer = system->zBuffer;
    return zBuffer[IndFromXY(x, y)];
  }

  static void SetZBuffer(const uint32_t x, const uint32_t y,
                         const float value) {
    if (x >= system->window_x) {
      return;
    }
    if (y >= system->window_y) {
      return;
    }

    auto zBuffer = system->zBuffer;
    zBuffer[IndFromXY(x, y)] = value;
  }

  static void SetZBuffer(const float x, const float y, const float &value) {
    System3D::SetZBuffer(static_cast<uint32_t>(system->window_x * x),
                         static_cast<uint32_t>(system->window_y * y), value);
  }

protected:
  System3D(const uint32_t window_x = 900, const uint32_t window_y = 900)
      : window_x(window_x), window_y(window_y) {

    _ClearBuffer();
  }

  static System3D *system;
  static System3DDeleter system3DDeleter;

public:
  Camera *activeCamera;

  float _DeltaPrePixel() const { return deltaPrePixel; }
  Vector4f _ColorState() const { return colorState; }
  void _ColorStateSet(const Vector4f &rgba) { colorState = rgba; }
  void _SetWindowSize(const uint32_t x, const uint32_t y) {
    window_x = x;
    window_y = y;
    deltaPrePixelX = canvas_x / window_x;
    deltaPrePixelY = canvas_y / window_y;
    deltaPrePixel = deltaPrePixelX;

    _ClearBuffer();
  }

protected:
  float canvas_x = 1.0f; // height
  float canvas_y = 1.0f; // width

  uint32_t window_x = 900; // height
  uint32_t window_y = 900; // width

  float deltaPrePixelX = canvas_x / window_x;
  float deltaPrePixelY = canvas_y / window_y;
  float deltaPrePixel = deltaPrePixelX;

  Vector4f colorState = {1.0f, 1.0f, 1.0f, 1.0f};

protected:
  float *zBuffer = nullptr;
  Vector4f *colorBuffer = nullptr;
  Vector4f *normalBuffer = nullptr;
  Vector4f *albedoBuffer = nullptr;

public:
  // Ray tracing
  uint32_t pixelSampleTimes = 4;
  uint32_t pixelSampleDeep = 3;

  uint32_t drawThreads = 8;
  uint32_t clipX = 1;
  SkyBox skyBox;

protected:
  MaterialManager materialManager;

public:
  static void PushMaterial(Material *const material) {
    system->materialManager.PushMaterial(material);
  }

  static Material *const GetMaterial(const uint32_t ind) {
    return system->materialManager.GetMaterial(ind);
  }

  static Material *const GetMaterialByName(const string &name) {
    return system->materialManager.GetMaterialByName(name);
  }

  static Material *const GetDefaultMaterial() {
    return system->materialManager.GetDefaultMaterial();
  }

protected:
  vector<const Triangle *> trianglesRef;
  vector<const Spline *> splinesRef;
  vector<Light *> lightsRef;
  BVHNode *bvhRoot = nullptr;

public:
  static void SetPixelSampleTimes(const uint32_t times) {
    system->pixelSampleTimes = times;
  }
  static void SetPixelSampleDeep(const uint32_t deep) {
    system->pixelSampleDeep = deep;
  }
  static void SetThreads(const uint32_t threads) {
    system->drawThreads = threads;
  }

  static Vector2i WindowSize() {
    Vector2i size(system->window_x, system->window_y);
    return size;
  }

  static void ClearRef() {
    system->trianglesRef.clear();
    system->splinesRef.clear();
  }

  static void PushLightRef(Light *light) { system->lightsRef.push_back(light); }

  static void PushTriangleRef(const Triangle *prim) {
    system->trianglesRef.push_back(prim);
  }

  static void PushSplineRef(const Spline *spline) {
    system->splinesRef.push_back(spline);
  }

  static void BuildBVH() { system->_BuildBVH(); }

  static void RefreshShadowMap() { system->_RefreshShadowMap(); }

  static vector<const Triangle *> GetTriasFromBVH(const Ray &ray) {
    return system->_GetTriasFromBVH(ray);
  }

  static void DrawBVHsFramwork() { system->_DrawBVHsFramwork(); }

  static void DrawSplines() { system->_DrawSplines(); }

  static void DrawTriangles() { system->_DrawTriangles(); }

  static void DrawTrianglesInClip(const Primitive2D &clip) {
    system->_DrawTrianglesInClip(clip);
  }
  static void DrawTrianglesInRange(const Vector2f xPercent,
                                   const Vector2f yPercent) {
    int temp = 0;
    system->_DrawTrianglesInRange(xPercent, yPercent, &temp);
  }

  static void DrawTrianglesInRangeMultiThread(const Vector2f xPercent,
                                              const Vector2f yPercent) {
    system->_DrawTrianglesInRangeMultiThread(xPercent, yPercent);
  }

  static void ShowTextureToBuffer(const Texture<Vector3f> &texture,
                                  const float vaulesScale = 1.0f) {
    auto size = System3D::WindowSize();
    const float xInverse = 1.0f / size.x(), yInverse = 1.0f / size.y();
    for (uint32_t x = 0; x < size.x(); ++x) {
      for (uint32_t y = 0; y < size.y(); ++y) {
        const Vector2f imagePos(x * xInverse, y * yInverse);
        Vector3f color = texture.Sample(imagePos.x(), imagePos.y());
        color *= vaulesScale;
        System3D::SetBufferColor(x, y, color);
      }
    }
  }

  static void ShowTextureToBuffer(const Texture<float> &texture,
                                  const float vaulesScale = 1.0f) {
    auto size = System3D::WindowSize();
    const float xInverse = 1.0f / size.x(), yInverse = 1.0f / size.y();
    for (uint32_t x = 0; x < size.x(); ++x) {
      for (uint32_t y = 0; y < size.y(); ++y) {
        const Vector2f imagePos(x * xInverse, y * yInverse);
        auto value = texture.Sample(imagePos.x(), imagePos.y());
        value *= vaulesScale;
        System3D::SetBufferColor(x, y, Vector3f(value, value, value));
      }
    }
  }

protected:
  void _RefreshShadowMap();

  void _BuildBVH();

  vector<const Triangle *> _GetTriasFromBVH(const Ray &ray) const;

  void _DrawBVHsFramwork() const {
    stack<const BVHNode *> nodes;
    nodes.push(bvhRoot);
    while (!nodes.empty()) {
      auto node = nodes.top();
      nodes.pop();
      if (!node->IsLeaf()) {
        nodes.push(node->NextNode1());
        nodes.push(node->NextNode2());
      } else {
        node->DrawBVHFramwork();
      }
    }
  }

  // x is row number, y is column number
  uint32_t _IndFromXY(const uint32_t x, const uint32_t y) {
    return y * window_x + x;
  }

  void _ClearBuffer() {
    if (zBuffer != nullptr) {
      delete[] zBuffer;
      zBuffer = nullptr;
    }
    if (colorBuffer != nullptr) {
      delete[] colorBuffer;
      colorBuffer = nullptr;
    }
    if (normalBuffer != nullptr) {
      delete[] normalBuffer;
      normalBuffer = nullptr;
    }
    if (albedoBuffer != nullptr) {
      delete[] albedoBuffer;
      albedoBuffer = nullptr;
    }

    colorBuffer = new Vector4f[window_x * window_y]{};
    normalBuffer = new Vector4f[window_x * window_y]{};
    albedoBuffer = new Vector4f[window_x * window_y]{};

    zBuffer = new float[window_x * window_y];
    for (uint32_t x = 0; x < window_x; ++x) {
      for (uint32_t y = 0; y < window_y; ++y) {
        auto ind = _IndFromXY(x, y);
        // colorBuffer[ind] = Vector4f::Zero();
        // normalBuffer[ind] = Vector4f::Zero();
        // albedoBuffer[ind] = Vector4f::Zero();
        zBuffer[ind] = numeric_limits<float>::infinity();
      }
    }
  }

  void _DrawSplines();

  void _DrawTriangles() {
    int temp = 0;
    _DrawTrianglesInRangePixel(Vector2i(0, window_x), Vector2i(0, window_y),
                               &temp);
  }
  void _DrawTrianglesInClip(const Primitive2D &clip);

  void _DrawTrianglesInRangeMultiThread(const Vector2f &xPercent,
                                        const Vector2f &yPercent) {
    vector<thread *> threads;
    int runninghreadCount = 0;

    const Vector2i xRange = (window_x * xPercent).cast<int>(),
                   yRange = (window_y * yPercent).cast<int>();

    // Create and manage threads
    // be careful release optimize!!
    int xCount = xRange.x();
    while (xCount < xRange.y()) {
      if (runninghreadCount < this->drawThreads) {
        int preX = xCount;
        xCount += clipX;
        if (xCount > xRange.y()) {
          xCount = xRange.y();
        }
        auto t = new thread(&System3D::_DrawTrianglesInRangePixel, this,
                            Vector2i(preX, xCount), yRange, &runninghreadCount);
        threads.push_back(t);
      } else {
        this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    // Waiting
    for (auto tP : threads) {
      tP->join();
      delete tP;
    }
  }

  void _DrawTrianglesInRange(const Vector2f &xPercent, const Vector2f &yPercent,
                             int *const runninghreadCount) {
    Vector2i xRange = (window_x * xPercent).cast<int>(),
             yRange = (window_y * yPercent).cast<int>();
    _DrawTrianglesInRangePixel(xRange, yRange, runninghreadCount);
  }

  void _DrawTrianglesInRangePixel(const Vector2i &xRange,
                                  const Vector2i &yRange,
                                  int *const runninghreadCount);

  void _DrawTrianglesInOnePixel(const Matrix4f &toView, const float current_x,
                                const float current_y, const uint32_t x,
                                const uint32_t y);

  void _SampleTrianglesInOnePixelWithMaterial(const Matrix4f &toView,
                                              const float current_x,
                                              const float current_y,
                                              const uint32_t x,
                                              const uint32_t y);

  void _SampleTrianglesInOnePixelRayTracing(const Matrix4f &toView,
                                            const float current_x,
                                            const float current_y,
                                            const uint32_t x, const uint32_t y,
                                            const float nearplane_height_step,
                                            const float nearplane_width_step);
  void _RaySample(Ray &ray);
};

#ifndef System3D_static_init
#define System3D_static_init
System3D::System3DDeleter System3D::system3DDeleter;
System3D *System3D::system = nullptr;
#endif // !System3D_static_init

#endif