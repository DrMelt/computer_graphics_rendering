#pragma once

using namespace std;
using namespace Eigen;

int SpatialHashing(const Vector3f &pos, const float scale = 1.0f) {
  const Vector3f posScaled = pos * scale;
  const int hash = (static_cast<int>(posScaled.x()) * 92837111) ^
                   (static_cast<int>(posScaled.y()) * 689287499) ^
                   (static_cast<int>(posScaled.z()) * 283923481);

  return abs(hash);
}
