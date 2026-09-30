#pragma once

#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Point3.hpp>
#include <GeoCore/linear/Quaternion.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维仿射变换，内部为 4×4 齐次矩阵（行主序，列向量约定 `M * v`）。
/// 平移量位于第 4 列。
///
/// 第 4 行假定为 (0,0,0,1)，且 apply / operator* 都不读取它 —— 直接改写
/// matrix 而破坏这个前提时，两者的结果不再被保证。类型不做这项检查。
///
/// 三种「施加」语义不要混用 ——
///   operator*(Point3T)    施加完整仿射变换，输入按**位置**解读
///   operator*(Vector3T)   只施加线性部分，输入按**方向**解读（平移不生效）
///   apply(Vector3T)       施加完整仿射变换，把 Vector 读作位置（历史用法，新代码请用 transform_point）
/// transform_point(Point3T) 与 operator*(Point3T) 等价，名字更直白。
template <typename Scalar>
struct Transform3T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 4> matrix = MatrixT<Scalar, 4>::identity();

    [[nodiscard]] constexpr bool operator==(const Transform3T&) const noexcept = default;

    /// 单位变换。
    [[nodiscard]] static constexpr Transform3T identity() noexcept {
        return Transform3T<Scalar>{};
    }

    /// 平移变换，平移量写入第 4 列。
    [[nodiscard]] static constexpr Transform3T translation(Vector3T<Scalar> offset) noexcept {
        Transform3T<Scalar> result{};
        result.matrix.data[0][3] = offset.x;
        result.matrix.data[1][3] = offset.y;
        result.matrix.data[2][3] = offset.z;
        return result;
    }

    /// 各轴独立的缩放。
    [[nodiscard]] static constexpr Transform3T scaling(Vector3T<Scalar> factors) noexcept {
        Transform3T<Scalar> result{};
        result.matrix.data[0][0] = factors.x;
        result.matrix.data[1][1] = factors.y;
        result.matrix.data[2][2] = factors.z;
        return result;
    }

    /// 各轴相同的缩放。
    ///
    /// 参数是浮点数（由实参推导），因此调用处写 `Transform3::scaling(2.0)` 即可。
    /// 传整数字面量会因不满足 floating_point 约束而编译失败 —— 这是刻意的：
    /// 缩放因子作用在浮点变换上，应当是浮点数。若确实要写 `scaling(2)`，请改为
    /// `2.0`。
    template <std::floating_point Factor>
    [[nodiscard]] static constexpr Transform3T scaling(Factor factor) noexcept {
        return scaling(Vector3T<Scalar>{factor, factor, factor});
    }

    /// 绕给定单位轴旋转 angle_radians 弧度。
    [[nodiscard]] static Transform3T rotation(
        UnitVector3T<Scalar> axis, Scalar angle_radians) noexcept {
        const QuaternionT<Scalar> q = QuaternionT<Scalar>::from_axis_angle(axis, angle_radians);
        const MatrixT<Scalar, 3> rotation = q.to_matrix();

        Transform3T<Scalar> result{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.matrix.data[i][j] = rotation.data[i][j];
            }
        }
        return result;
    }

    /// 施加完整仿射变换，输入按**位置**解读（平移生效）。
    ///
    /// 历史用法：位置以 Vector3T 承载时的入口。它仍然有效，但**新代码请用
    /// transform_point(Point3T)**，或等价的 operator*(Point3T) —— 位置由
    /// Point3T 承载，语义在类型上就是对的。
    [[nodiscard]] constexpr Vector3T<Scalar> apply(Vector3T<Scalar> position) const noexcept {
        const MatrixT<Scalar, 4>& m = matrix;
        return Vector3T<Scalar>{
            m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2] * position.z + m.data[0][3],
            m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2] * position.z + m.data[1][3],
            m.data[2][0] * position.x + m.data[2][1] * position.y + m.data[2][2] * position.z + m.data[2][3],
        };
    }

    /// 施加完整仿射变换，输入按**位置**解读（平移生效）。
    ///
    /// 与 operator*(Point3T) 等价，名字更直白。与 apply() 的数学内容完全相同，
    /// 只是接受并返回 Point3T —— 因此它是 apply() 的替代入口，不是它的补充。
    [[nodiscard]] constexpr Point3T<Scalar> transform_point(Point3T<Scalar> position) const noexcept {
        const MatrixT<Scalar, 4>& m = matrix;
        return Point3T<Scalar>{
            m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2] * position.z + m.data[0][3],
            m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2] * position.z + m.data[1][3],
            m.data[2][0] * position.x + m.data[2][1] * position.y + m.data[2][2] * position.z + m.data[2][3],
        };
    }

    /// 逆变换。线性部分奇异时返回 std::nullopt。
    [[nodiscard]] std::optional<Transform3T<Scalar>> inverse(
        core::Tolerance tolerance = {}) const noexcept {
        const auto inverse_matrix = matrix.inverse(tolerance);
        if (!inverse_matrix.has_value()) {
            return std::nullopt;
        }
        return Transform3T<Scalar>{*inverse_matrix};
    }
};

using Transform3 = Transform3T<double>;
using Transform3f = Transform3T<float>;

// ---- 运算符 ----

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

/// 施加完整仿射变换，输入按**位置**解读（平移生效）。
///
/// 与 transform_point 等价。注意它与上面的 operator*(Vector3T) 是**两个语义
/// 不同**的重载：点吃平移，方向不吃 —— 两者不可互相替代。
template <typename Scalar>
[[nodiscard]] constexpr Point3T<Scalar> operator*(
    const Transform3T<Scalar>& t, Point3T<Scalar> point) noexcept {
    return t.transform_point(point);
}

} // namespace GeoCore::linear
