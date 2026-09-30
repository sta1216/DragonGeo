#pragma once

#include <numbers>

namespace DragonGeo::Core {

/// 圆周率。
inline constexpr double PI = std::numbers::pi;

/// 半圆周率，π/2。
inline constexpr double HALF_PI = PI / 2.0;

/// 整圆周，2π。
inline constexpr double TWO_PI = 2.0 * PI;

/// 四分之一圆周，π/4。
inline constexpr double QUARTER_PI = PI / 4.0;

/// √2。
inline constexpr double SQRT_TWO = std::numbers::sqrt2;

} // namespace DragonGeo::Core
