#ifndef _TEXTURE_HPP_
#define _TEXTURE_HPP_
// clang-format off
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
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

#pragma region MapInBox

// Map 3D space to edgePixels x 6*edgePixels space
// return range [0,1]
// dir can be not normalized
Vector2f MapDir2TextureInBox(const Vector3f &dir) {
  auto absDir = dir.cwiseAbs();
  int maxIndex = 0;
  auto maxCoeff = absDir.maxCoeff(&maxIndex);
  auto projectedDir = dir / (maxCoeff * 2); // abs(max) == 0.5f

  int faceInd = maxIndex * 2;
  if (dir[maxIndex] > 0) {
    faceInd += 1;
  }

  Vector2f imagePos = {projectedDir[(maxIndex + 1) % 3] + 0.5f,
                       projectedDir[(maxIndex + 2) % 3] + 0.5f};
  imagePos.y() += faceInd;
  imagePos.y() *= 1.0f / 6.0f;
  return imagePos;
}

// Map to 3D space
// input pos range [0,1]
Vector3f MapTexture2DirInBox(const Vector2f &imagePos) {
  int faceInd = floor(imagePos.y() * 6);
  int maxIndex = floor(faceInd / 2);
  int sign = faceInd - maxIndex * 2; // positive if sign == 1

  Vector3f projectedDir;
  projectedDir[maxIndex] = 0.5f * (sign * 2 - 1);
  projectedDir[(maxIndex + 1) % 3] = imagePos.x() - 0.5f;
  projectedDir[(maxIndex + 2) % 3] = imagePos.y() * 6 - faceInd - 0.5f;

  return projectedDir.normalized();
}
#pragma endregion

#pragma region MapInCylinder

// Map 3D space to heightPixels x 2*heightPixels space
// return range [0,1)
// upDir is y
// dir should be normalized
Vector2f MapDir2TextureInCylinder(const Vector3f &dir) {
  const float theta = asinf(dir.y());         // [-0.5pi, 0.5pi]
  const float phi = atan2f(dir.z(), dir.x()); // [-pi, pi]

  const Vector2f imagePos = {
      clamp(theta / static_cast<float>(M_PI) + 0.5f, 0.0f, 1.0f - 1e-6f),
      clamp(phi / static_cast<float>(M_PI) * 0.5f + 0.5f, 0.0f, 1.0f - 1e-6f)};
  return imagePos;
}

// Map to 3D space
// upDir is y
// input pos range [0,1]
Vector3f MapTexture2DirInCylinder(const Vector2f &imagePos) {
  const float phi = imagePos.y() * 2 * M_PI,
              theta = (imagePos.x() - 0.5f) * M_PI;
  const float y = cosf(theta);
  const float xzLen = sqrt(1 - y * y);
  const float x = cosf(phi) * xzLen, z = sinf(phi) * xzLen;

  Vector3f projectedDir = {x, y, z};
  return projectedDir;
}
#pragma endregion

uint64_t CVImageIndStart(const uint32_t x, const uint32_t y, const int height,
                         const int width, const float xScale,
                         const float yScale, const int channels) {
  return ((height - static_cast<uint64_t>(1) -
           static_cast<uint64_t>(xScale * x)) *
              width +
          static_cast<uint64_t>(yScale * y)) *
         channels;
}

template <class T> struct Texture {
protected:
  uint32_t dataLen = 0;
  Vector2i size;
  T *data = nullptr;

public:
  Texture(const uint32_t x, const uint32_t y) : size(x, y), dataLen(x * y) {
    data = new T[dataLen];
  }

  Texture(const Texture<T> &other) : size(other.size), dataLen(other.dataLen) {
    if (data != nullptr) {
      delete[] data;
    }
    data = new T[dataLen];

    // for (uint32_t i = 0; i < dataLen; ++i) {
    //   data[i] = other.data[i];
    // }
    memcpy(data, other.data, dataLen * sizeof(T));
  }

  Texture &operator=(const Texture<T> &other) {
    if (data != nullptr) {
      delete[] data;
      data = nullptr;
    }

    size = other.size;
    dataLen = other.dataLen;

    if (data != nullptr) {
      delete[] data;
    }
    data = new T[dataLen];

    // for (uint32_t i = 0; i < dataLen; ++i) {
    //   data[i] = other.data[i];
    // }
    memcpy(data, other.data, dataLen * sizeof(T));

    return *this;
    // memcpy(data, other.data, dataLen * sizeof(T));
  }

  virtual ~Texture() {
    if (data != nullptr) {
      delete[] data;
      data = nullptr;
    }
  }

public:
  void ReSize(const uint32_t x, const uint32_t y) {
    size = Vector2i(x, y);
    dataLen = x * y;

    if (data != nullptr) {
      delete[] data;
    }
    data = new T[dataLen]{};
  }

  Vector2i Size() const { return size; }

  void Fill(const T &dataFrom) {
    for (uint32_t i = 0; i < dataLen; ++i) {
      data[i] = dataFrom;
    }
  }

  T GetData(const uint32_t x, const uint32_t y) const {
    return data[GetIndFromXY(x, y)];
  }

  void SetData(const T *const dataArr) {
    memcpy(data, dataArr, dataLen * sizeof(T));
  }

  void SetData(const uint32_t x, const uint32_t y, const T &value) {
    data[GetIndFromXY(x, y)] = value;
  }

  enum SampleMethod { Nearest, Bilinear };

  T Sample(const float x, const float y,
           const SampleMethod method = SampleMethod::Bilinear) const {
    if (method == SampleMethod::Bilinear) {
      return SampleBilinear(x, y);
    } else if (method == SampleMethod::Nearest) {
      return SampleNearest(x, y);
    }
  }

  T SampleBilinear(const float x, const float y) const {
    float mod_x = fmodf(x - 0.5f * 1.0f / size.x(), 1.0f - 1e-7f),
          mod_y = fmodf(y - 0.5f * 1.0f / size.y(), 1.0f - 1e-7f);
    if (mod_x < 0.0f) {
      mod_x += 1.0f - 1e-7f;
    }
    if (mod_y < 0.0f) {
      mod_y += 1.0f - 1e-7f;
    }

    const float xScaled = mod_x * size.x(), yScaled = mod_y * size.y();
    const uint32_t indX = static_cast<uint32_t>(xScaled),
                   indY = static_cast<uint32_t>(yScaled);
    uint32_t indX1 = indX + 1, indY1 = indY + 1;

    if (indX1 >= size.x()) {
      indX1 = indX1 % size.x();
    }
    if (indY1 >= size.y()) {
      indY1 = indY1 % size.y();
    }
    T texValue00 = GetData(indX, indY), texValue10 = GetData(indX1, indY),
      texValue01 = GetData(indX, indY1), texValue11 = GetData(indX1, indY1);

    const float xBlend = xScaled - indX, yBlend = yScaled - indY;
    const T blendValue =
        (1 - yBlend) * ((1 - xBlend) * texValue00 + xBlend * texValue10) +
        yBlend * ((1 - xBlend) * texValue01 + xBlend * texValue11);

    return blendValue;
  }

  T SampleNearest(const float x, const float y) const {
    float mod_x = fmodf(x, 1.0f - 1e-8f), mod_y = fmodf(y, 1.0f - 1e-8f);
    if (mod_x < 0.0f) {
      mod_x += 1.0f - 1e-8f;
    }
    if (mod_y < 0.0f) {
      mod_y += 1.0f - 1e-8f;
    }

    return GetData(static_cast<uint32_t>(mod_x * size.x()),
                   static_cast<uint32_t>(mod_y * size.y()));
  }

  uint32_t GetIndFromXY(const uint32_t x, const uint32_t y) const {
    return y * size.x() + x;
  }

  void ReadImage(const string &filePath);
  void ReadImageAndMatchSize(const string &filePath);
  void ReadAlpahFromImageAndMatchSize(const string &filePath);
};

void Texture<Vector3f>::ReadImage(const string &filePath) {
  const float pixelValueScale = 1.0f / 255.0f;
  cv::Mat I = cv::imread(filePath);
  const uint8_t *const imageData = I.data;
  // image size
  const int height = I.rows;
  const int width = I.cols;
  const int channels = I.channels();

  const float xScale = static_cast<float>(height) / size.x();
  const float yScale = static_cast<float>(width) / size.y();

  for (int x = 0; x < size.x(); ++x) {
    for (int y = 0; y < size.y(); ++y) {
      auto indStart =
          CVImageIndStart(x, y, height, width, xScale, yScale, channels);
      data[GetIndFromXY(x, y)] =
          Vector3f(imageData[indStart + 2], imageData[indStart + 1],
                   imageData[indStart]) *
          pixelValueScale;
    }
  }
}

void Texture<Vector3f>::ReadImageAndMatchSize(const string &filePath) {
  const float pixelValueScale = 1.0f / 255.0f;
  cv::Mat I = cv::imread(filePath);
  const uint8_t *const imageData = I.data;
  // image size
  const int height = I.rows;
  const int width = I.cols;
  const int channels = I.channels();

  ReSize(height, width);

  const float xScale = static_cast<float>(height) / size.x();
  const float yScale = static_cast<float>(width) / size.y();

  for (int x = 0; x < size.x(); ++x) {
    for (int y = 0; y < size.y(); ++y) {
      auto indStart =
          CVImageIndStart(x, y, height, width, xScale, yScale, channels);

      data[GetIndFromXY(x, y)] =
          Vector3f(imageData[indStart + 2], imageData[indStart + 1],
                   imageData[indStart]) *
          pixelValueScale;
    }
  }
}

void Texture<float>::ReadImageAndMatchSize(const string &filePath) {
  const float pixelValueScale = 1.0f / 255.0f;
  cv::Mat I = cv::imread(filePath);
  auto imageData = I.data;
  // image size
  const int height = I.rows;
  const int width = I.cols;
  const int channels = I.channels();

  ReSize(height, width);

  float xScale = static_cast<float>(height) / size.x();
  float yScale = static_cast<float>(width) / size.y();

  for (int x = 0; x < size.x(); ++x) {
    for (int y = 0; y < size.y(); ++y) {
      auto indStart =
          CVImageIndStart(x, y, height, width, xScale, yScale, channels);
      data[GetIndFromXY(x, y)] = imageData[indStart];
    }
  }
}

void Texture<float>::ReadAlpahFromImageAndMatchSize(const string &filePath) {
  const float pixelValueScale = 1.0f / 255.0f;
  cv::Mat I = cv::imread(filePath, cv::IMREAD_UNCHANGED);
  auto imageData = I.data;
  // image size
  const int height = I.rows;
  const int width = I.cols;
  const int channels = I.channels();

  ReSize(height, width);

  float xScale = static_cast<float>(height) / size.x();
  float yScale = static_cast<float>(width) / size.y();

  // if image has alpha channel
  if (channels > 3) {

    for (int x = 0; x < size.x(); ++x) {
      for (int y = 0; y < size.y(); ++y) {
        auto indStart =
            CVImageIndStart(x, y, height, width, xScale, yScale, channels);
        data[GetIndFromXY(x, y)] = imageData[indStart + 3] * pixelValueScale;
      }
    }
  } else {
    Fill(1.0f);
  }
}

#endif // !_TEXTURE_HPP_
