#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Quaternion.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维仿射变换，内部为 4×4 齐次矩阵（行主序，列向量约定 `M * v`）。
/// 平移量位于第 4 列。
///
/// 注意分层约束：Point3 属于 prim 层，linear 不得依赖它。因此本类型
/// 只提供作用于 Vector3T 的运算 ——
///   operator*(v)          只施加线性部分，用于变换方向
///   apply(t, v)           施加完整仿射变换，v 被解读为位置
/// 作用于 Point3 的运算符由 prim 层提供。
template <typename Scalar>
struct Transform3T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 4> matrix = identity<Scalar, 4>();

    [[nodiscard]] constexpr bool operator==(const Transform3T&) const noexcept = default;
};

using Transform3 = Transform3T<double>;
using Transform3f = Transform3T<float>;

// ---- 工厂 ----

template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> identity_transform() noexcept {
    return Transform3T<Scalar>{};
}

template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> translation_3d(Vector3T<Scalar> offset) noexcept {
    Transform3T<Scalar> result{};
    result.matrix.data[0][3] = offset.x;
    result.matrix.data[1][3] = offset.y;
    result.matrix.data[2][3] = offset.z;
    return result;
}

/// 各轴独立的缩放。
template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> scaling_3d(Vector3T<Scalar> factors) noexcept {
    Transform3T<Scalar> result{};
    result.matrix.data[0][0] = factors.x;
    result.matrix.data[1][1] = factors.y;
    result.matrix.data[2][2] = factors.z;
    return result;
}

/// 各轴相同的缩放。
///
/// 参数类型即 Scalar，因此调用处写 `scaling_3d(2.0)` 即可。传整数字面量
/// 会因不满足 floating_point 约束而编译失败 —— 这是刻意的：缩放因子作用
/// 在浮点变换上，应当是浮点数。若确实要写 `scaling_3d(2)`，请改为 `2.0`。
template <std::floating_point Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> scaling_3d(Scalar factor) noexcept {
    return scaling_3d(Vector3T<Scalar>{factor, factor, factor});
}

/// 绕给定单位轴旋转 angle_radians 弧度。
template <typename Scalar>
[[nodiscard]] Transform3T<Scalar> rotation_3d(
    UnitVector3T<Scalar> axis, Scalar angle_radians) noexcept {
    const QuaternionT<Scalar> q = from_axis_angle(axis, angle_radians);
    const MatrixT<Scalar, 3> rotation = to_matrix(q);

    Transform3T<Scalar> result{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.matrix.data[i][j] = rotation.data[i][j];
        }
    }
    return result;
}

// ---- 应用 ----

/// 施加完整仿射变换，输入按**位置**解读（平移生效）。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> apply(
    const Transform3T<Scalar>& t, Vector3T<Scalar> position) noexcept {
    const MatrixT<Scalar, 4>& m = t.matrix;
    return Vector3T<Scalar>{
        m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2] * position.z + m.data[0][3],
        m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2] * position.z + m.data[1][3],
        m.data[2][0] * position.x + m.data[2][1] * position.y + m.data[2][2] * position.z + m.data[2][3],
    };
}

/// 只施加线性部分，输入按**方向**解读（平移不生效）。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const Transform3T<Scalar>& t, Vector3T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 4>& m = t.matrix;
    return Vector3T<Scalar>{
        m.data[0][0] * direction.x + m.data[0][1] * direction.y + m.data[0][2] * direction.z,
        m.data[1][0] * direction.x + m.data[1][1] * direction.y + m.data[1][2] * direction.z,
        m.data[2][0] * direction.x + m.data[2][1] * direction.y + m.data[2][2] * direction.z,
    };
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> operator*(
    const Transform3T<Scalar>& a, const Transform3T<Scalar>& b) noexcept {
    return Transform3T<Scalar>{a.matrix * b.matrix};
}

/// 逆变换。线性部分奇异时返回 std::nullopt。
template <typename Scalar>
[[nodiscard]] std::optional<Transform3T<Scalar>> inverse(
    const Transform3T<Scalar>& t, core::Tolerance tolerance = {}) noexcept {
    const auto inverse_matrix = inverse(t.matrix, tolerance);
    if (!inverse_matrix.has_value()) {
        return std::nullopt;
    }
    return Transform3T<Scalar>{*inverse_matrix};
}

} // namespace GeoCore::linear
