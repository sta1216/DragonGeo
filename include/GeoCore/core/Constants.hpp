#pragma once

#include <numbers>

namespace GeoCore::core {

/// 圆周率。
inline constexpr double pi = std::numbers::pi;

/// 半圆周率，π/2。
inline constexpr double half_pi = pi / 2.0;

/// 整圆周，2π。
inline constexpr double two_pi = 2.0 * pi;

/// 四分之一圆周，π/4。
inline constexpr double quarter_pi = pi / 4.0;

/// √2。
inline constexpr double sqrt_two = std::numbers::sqrt2;

} // namespace GeoCore::core
