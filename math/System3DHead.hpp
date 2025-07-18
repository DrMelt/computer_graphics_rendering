#ifndef _SYSTEM3D_HEAD_HPP_
#define _SYSTEM3D_HEAD_HPP_

// clang-format off
#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <random>
#include <stack>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stack>
#include <string>
#include <vector>
#include <filesystem>
#include <Eigen/Geometry>
#include <thread>

constexpr bool IS_RAY_TRACING = false;
constexpr bool OPEN_DENOISE = true;
constexpr bool IS_PERSPECTIVE_PROJECT = true;

#include "Transform.hpp"
#include "Texture.hpp"
#include "Material.hpp"
#include "System3D.hpp"

#include "DataStruct.hpp"
#include "Ray.hpp"
#include "Primitive.hpp"
#include "Geometry.hpp"

#include "Light.hpp"
#include "System3DImplementation.hpp"
#include "IO.hpp"

#include "Hash.hpp"

// clang-format on

#endif // !_SYSTEM3D_HEAD_HPP_
