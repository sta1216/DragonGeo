#pragma once

#include <concepts>
#include <optional>

#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Matrix.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Quaternion.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>

namespace DragonGeo::Linear {

/// 三维仿射变换，内部为 4×4 齐次矩阵（行主序，列向量约定 `M * v`）。
/// 平移量位于第 4 列。
///
/// 第 4 行假定为 (0,0,0,1)，且 TransformPoint / operator* 都不读取它 —— 直接改写
/// Matrix 而破坏这个前提时，两者的结果不再被保证。类型不做这项检查。
///
/// 两种「施加」语义不要混用 ——
///   operator*(Point3T)    施加完整仿射变换，输入按**位置**解读
///   operator*(Vector3T)   只施加线性部分，输入按**方向**解读（平移不生效）
/// TransformPoint(Point3T) 与 operator*(Point3T) 等价，名字更直白。
/// 不提供把 Vector 当位置施加的 Apply：它与 TransformPoint 重复，并且会让
/// `a * b.Apply(v)` 静默丢掉 a 的平移。
template <typename Scalar>
struct Transform3T {
    using ScalarType = Scalar;

    MatrixT<Scalar, 4> Matrix = MatrixT<Scalar, 4>::Identity();

    [[nodiscard]] constexpr bool operator==(const Transform3T&) const noexcept = default;

    /// 单位变换。
    [[nodiscard]] static constexpr Transform3T Identity() noexcept {
        return Transform3T<Scalar>{};
    }

    /// 平移变换，平移量写入第 4 列。
    [[nodiscard]] static constexpr Transform3T Translation(Vector3T<Scalar> offset) noexcept {
        Transform3T<Scalar> result{};
        result.Matrix.Data[0][3] = offset.X;
        result.Matrix.Data[1][3] = offset.Y;
        result.Matrix.Data[2][3] = offset.Z;
        return result;
    }

    /// 各轴独立的缩放。
    [[nodiscard]] static constexpr Transform3T Scaling(Vector3T<Scalar> factors) noexcept {
        Transform3T<Scalar> result{};
        result.Matrix.Data[0][0] = factors.X;
        result.Matrix.Data[1][1] = factors.Y;
        result.Matrix.Data[2][2] = factors.Z;
        return result;
    }

    /// 各轴相同的缩放。
    ///
    /// 参数是浮点数（由实参推导），因此调用处写 `Transform3::Scaling(2.0)` 即可。
    /// 传整数字面量会因不满足 floating_point 约束而编译失败 —— 这是刻意的：
    /// 缩放因子作用在浮点变换上，应当是浮点数。若确实要写 `Scaling(2)`，请改为
    /// `2.0`。
    template <std::floating_point Factor>
    [[nodiscard]] static constexpr Transform3T Scaling(Factor factor) noexcept {
        return Scaling(Vector3T<Scalar>{factor, factor, factor});
    }

    /// 绕给定单位轴旋转 angleRadians 弧度。
    [[nodiscard]] static Transform3T Rotation(
        UnitVector3T<Scalar> axis, Scalar angleRadians) noexcept {
        const QuaternionT<Scalar> q = QuaternionT<Scalar>::FromAxisAngle(axis, angleRadians);
        const MatrixT<Scalar, 3> rotation = q.ToMatrix();

        Transform3T<Scalar> result{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.Matrix.Data[i][j] = rotation.Data[i][j];
            }
        }
        return result;
    }

    /// 施加完整仿射变换，输入按**位置**解读（平移生效）。
    ///
    /// 与 operator*(Point3T) 等价，名字更直白。
    [[nodiscard]] constexpr Point3T<Scalar> TransformPoint(Point3T<Scalar> position) const noexcept {
        const MatrixT<Scalar, 4>& m = Matrix;
        return Point3T<Scalar>{
            m.Data[0][0] * position.X + m.Data[0][1] * position.Y + m.Data[0][2] * position.Z + m.Data[0][3],
            m.Data[1][0] * position.X + m.Data[1][1] * position.Y + m.Data[1][2] * position.Z + m.Data[1][3],
            m.Data[2][0] * position.X + m.Data[2][1] * position.Y + m.Data[2][2] * position.Z + m.Data[2][3],
        };
    }

    /// 逆变换。线性部分奇异时返回 std::nullopt。
    [[nodiscard]] std::optional<Transform3T<Scalar>> Inverse(
        Core::Tolerance tolerance = {}) const noexcept {
        const auto inverseMatrix = Matrix.Inverse(tolerance);
        if (!inverseMatrix.has_value()) {
            return std::nullopt;
        }
        return Transform3T<Scalar>{*inverseMatrix};
    }
};

using Transform3 = Transform3T<double>;
using Transform3f = Transform3T<float>;

// ---- 运算符 ----

/// 只施加线性部分，输入按**方向**解读（平移不生效）。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const Transform3T<Scalar>& t, Vector3T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 4>& m = t.Matrix;
    return Vector3T<Scalar>{
        m.Data[0][0] * direction.X + m.Data[0][1] * direction.Y + m.Data[0][2] * direction.Z,
        m.Data[1][0] * direction.X + m.Data[1][1] * direction.Y + m.Data[1][2] * direction.Z,
        m.Data[2][0] * direction.X + m.Data[2][1] * direction.Y + m.Data[2][2] * direction.Z,
    };
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform3T<Scalar> operator*(
    const Transform3T<Scalar>& a, const Transform3T<Scalar>& b) noexcept {
    return Transform3T<Scalar>{a.Matrix * b.Matrix};
}

/// 施加完整仿射变换，输入按**位置**解读（平移生效）。
///
/// 与 TransformPoint 等价。注意它与上面的 operator*(Vector3T) 是**两个语义
/// 不同**的重载：点吃平移，方向不吃 —— 两者不可互相替代。
template <typename Scalar>
[[nodiscard]] constexpr Point3T<Scalar> operator*(
    const Transform3T<Scalar>& t, Point3T<Scalar> point) noexcept {
    return t.TransformPoint(point);
}

} // namespace DragonGeo::Linear
