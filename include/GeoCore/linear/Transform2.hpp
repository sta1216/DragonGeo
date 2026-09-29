#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 二维仿射变换，内部为 3×3 齐次矩阵（行主序）。
/// 分层约束与 Transform3T 相同，详见该类型的文档。
template <typename Scalar>
struct Transform2T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 3> matrix = identity<Scalar, 3>();

    [[nodiscard]] constexpr bool operator==(const Transform2T&) const noexcept = default;
};

using Transform2 = Transform2T<double>;
using Transform2f = Transform2T<float>;

template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> translation_2d(Vector2T<Scalar> offset) noexcept {
    Transform2T<Scalar> result{};
    result.matrix.data[0][2] = offset.x;
    result.matrix.data[1][2] = offset.y;
    return result;
}

template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> scaling_2d(Vector2T<Scalar> factors) noexcept {
    Transform2T<Scalar> result{};
    result.matrix.data[0][0] = factors.x;
    result.matrix.data[1][1] = factors.y;
    return result;
}

template <std::floating_point Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> scaling_2d(Scalar factor) noexcept {
    return scaling_2d(Vector2T<Scalar>{factor, factor});
}

/// 逆时针旋转 angle_radians 弧度。
template <typename Scalar>
[[nodiscard]] Transform2T<Scalar> rotation_2d(Scalar angle_radians) noexcept {
    const Scalar cosine = std::cos(angle_radians);
    const Scalar sine = std::sin(angle_radians);

    Transform2T<Scalar> result{};
    result.matrix.data[0][0] = cosine;
    result.matrix.data[0][1] = -sine;
    result.matrix.data[1][0] = sine;
    result.matrix.data[1][1] = cosine;
    return result;
}

/// 施加完整仿射变换，输入按**位置**解读。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> apply(
    const Transform2T<Scalar>& t, Vector2T<Scalar> position) noexcept {
    const MatrixT<Scalar, 3>& m = t.matrix;
    return Vector2T<Scalar>{
        m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2],
        m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2],
    };
}

/// 只施加线性部分，输入按**方向**解读。
template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(
    const Transform2T<Scalar>& t, Vector2T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 3>& m = t.matrix;
    return Vector2T<Scalar>{
        m.data[0][0] * direction.x + m.data[0][1] * direction.y,
        m.data[1][0] * direction.x + m.data[1][1] * direction.y,
    };
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> operator*(
    const Transform2T<Scalar>& a, const Transform2T<Scalar>& b) noexcept {
    return Transform2T<Scalar>{a.matrix * b.matrix};
}

template <typename Scalar>
[[nodiscard]] std::optional<Transform2T<Scalar>> inverse(
    const Transform2T<Scalar>& t, core::Tolerance tolerance = {}) noexcept {
    const auto inverse_matrix = inverse(t.matrix, tolerance);
    if (!inverse_matrix.has_value()) {
        return std::nullopt;
    }
    return Transform2T<Scalar>{*inverse_matrix};
}

} // namespace GeoCore::linear
