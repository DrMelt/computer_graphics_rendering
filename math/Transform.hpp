#ifndef _TRANSFORM_HPP_
#define _TRANSFORM_HPP_

using namespace Eigen;

std::random_device random_device_instance;
std::mt19937 gen(random_device_instance());
std::uniform_real_distribution<float> dis(0.0, 1.0);
float random_0_to_1() { return dis(gen); }

Vector3f TransformForPos(const Vector3f &originPos, const Matrix4f &transM) {
  Vector4f ex_pos(originPos.x(), originPos.y(), originPos.z(), 1.0f);
  ex_pos = transM * ex_pos;
  ex_pos /= ex_pos.w();
  return ex_pos.block<3, 1>(0, 0);
}

Vector3f TransformForVector(const Vector3f &originVec, const Matrix4f &transM) {
  return transM.block<3, 3>(0, 0) * originVec;
}

Matrix2f Rotate2d(const float radian) {

  Eigen::Matrix2f rotate2d;
  rotate2d(0, 0) = cosf(radian);
  rotate2d(1, 0) = -sinf(radian);
  rotate2d(0, 1) = sinf(radian);
  rotate2d(1, 1) = cosf(radian);

  return rotate2d;
}

Matrix3f Rotate2dH(const float radian) {

  Eigen::Matrix3f rotate2dH;
  rotate2dH(0, 0) = cosf(radian);
  rotate2dH(0, 1) = -sinf(radian);
  rotate2dH(1, 0) = sinf(radian);
  rotate2dH(1, 1) = cosf(radian);
  rotate2dH.block<1, 2>(2, 0) << 0, 0;
  rotate2dH.block<2, 1>(0, 2) << 0, 0;
  rotate2dH(2, 2) = 1;

  return rotate2dH;
}

Matrix3f Rotate3d(const Vector3f &axis, const float radian) {
  auto axis_normal = axis.normalized();
  AngleAxisd rotation_vector(-radian, axis_normal.cast<double>());
  Matrix3f rotation_matrix = rotation_vector.toRotationMatrix().cast<float>();
  return rotation_matrix;
}

Matrix4f Rotate3dH(const Vector3f &axis, const float radian) {
  Matrix3f rotation_matrix = Rotate3d(axis, radian);
  Matrix4f rotation_matrixH;

  rotation_matrixH.block<3, 3>(0, 0) = rotation_matrix;
  rotation_matrixH.block<1, 3>(3, 0) << 0, 0, 0;
  rotation_matrixH.block<3, 1>(0, 3) << 0, 0, 0;
  rotation_matrixH(3, 3) = 1;

  return rotation_matrixH;
}

Matrix4f Translate3dH(const Vector3f &translation) {
  Matrix4f translate_matrix = Matrix4f::Identity();
  translate_matrix.block<3, 1>(0, 3) = translation;
  return translate_matrix;
}

// [0,0,1] -> lookAtDir
Matrix3f ToLookAt(const Vector3f &lookAt, const Vector3f &upVec) {
  const Vector3f lookAtDir = lookAt.normalized();
  const Vector3f normal = -(lookAtDir.cross(upVec)).normalized();
  const Vector3f up = lookAtDir.cross(normal);

  Matrix3f rotate;
  rotate.block<3, 1>(0, 0) = normal;
  rotate.block<3, 1>(0, 1) = up;
  rotate.block<3, 1>(0, 2) = lookAtDir;

  return rotate;
}
// [0,0,1] -> lookAtDir and translate
Matrix4f ToLookAtMatrix(const Vector3f &eyePos, const Vector3f &lookAtCenterPos,
                        const Vector3f &upVec) {

  Matrix4f transfer = Matrix4f::Identity();
  transfer.block<3, 1>(0, 3) = eyePos;

  Matrix4f rotate = Matrix4f::Zero();
  rotate.block<3, 3>(0, 0) = ToLookAt(lookAtCenterPos - eyePos, upVec);
  rotate(3, 3) = 1.0f;

  const Matrix4f toLookAtMatrix = transfer * rotate;

  return toLookAtMatrix;
}

// View transfer  ( same as Camera look dir -> [0,0,1] )
// Translate and rotate
Matrix4f LookAtMatrix(const Vector3f &eyePos, const Vector3f &lookAtCenterPos,
                      const Vector3f &upVec) {
  return ToLookAtMatrix(eyePos, lookAtCenterPos, upVec).inverse();
}

// Project transfer

// x -> [0,1]
// y -> [0,1]
// z -> [-1,1]
Matrix4f OrthogonalProjection(float left, float right, float bottom, float top,
                              float zNear, float zFar) {
  Matrix4f orthogonalProjection = Matrix4f::Identity();
  // x -> [0,1]
  orthogonalProjection(0, 0) = 1 / (right - left);
  orthogonalProjection(0, 3) = -1 * left / (right - left);
  // y -> [0,1]
  orthogonalProjection(1, 1) = 1 / (top - bottom);
  orthogonalProjection(1, 3) = -1 * bottom / (top - bottom);
  // z -> [-1,1]
  orthogonalProjection(2, 2) = 2 / (zFar - zNear);
  orthogonalProjection(2, 3) = -1 * (zFar + zNear) / (zFar - zNear);

  return orthogonalProjection;
}

// x -> [0,1]
// y -> [0,1]
// z -> [-1,1]
Matrix4f PerspectiveProjection(float left, float right, float bottom, float top,
                               float zNear, float zFar) {
  float xMax = std::max<float>(std::abs(left), std::abs(right));
  float yMax = std::max<float>(std::abs(bottom), std::abs(top));

  Matrix4f perspectiveProjection = Matrix4f::Identity();
  perspectiveProjection(0, 0) = zNear / xMax;
  perspectiveProjection(1, 1) = zNear / yMax;
  perspectiveProjection(2, 2) = zFar + zNear;
  perspectiveProjection(2, 3) = -zFar * zNear;
  perspectiveProjection(3, 2) = 1;
  perspectiveProjection(3, 3) = 0;
  // x -> [-1,1]
  // y -> [-1,1]
  // z -> z'

  auto orthogonalProjection = OrthogonalProjection(
      left / xMax, right / xMax, bottom / yMax, top / yMax, zNear, zFar);

  perspectiveProjection = orthogonalProjection * perspectiveProjection;

  return perspectiveProjection;
}

#endif // !_TRANSFORM_HPP_
