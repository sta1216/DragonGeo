#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Matrix.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 二维仿射变换，内部为 3×3 齐次矩阵（行主序）。
///
/// 两种「施加」语义不要混用 —— operator*(Point2T)    施加完整仿射变换，输入按**位置**解读 operator*(Vector2T)   只施加线性部分，输入按**方向**解读（平移不生效）
/// TransformPoint(Point2T) 与 operator*(Point2T) 等价，名字更直白。不提供把 Vector 当位置施加的 Apply：它与 TransformPoint 重复，并且会让 `a * b.Apply(v)` 静默丢掉 a 的平移。三维同样不提供。
template <typename Scalar> struct Transform2T {
    using ScalarType = Scalar;

    MatrixT<Scalar, 3> Matrix = MatrixT<Scalar, 3>::Identity();

    [[nodiscard]] constexpr bool operator==(const Transform2T&) const noexcept = default;

    /// 单位变换。
    [[nodiscard]] static constexpr Transform2T Identity() noexcept {
        return Transform2T<Scalar>{};
    }

    [[nodiscard]] static constexpr Transform2T Translation(Vector2T<Scalar> offset) noexcept {
        Transform2T<Scalar> result{};
        result.Matrix.Data[0][2] = offset.X;
        result.Matrix.Data[1][2] = offset.Y;
        return result;
    }

    [[nodiscard]] static constexpr Transform2T Scaling(Vector2T<Scalar> factors) noexcept {
        Transform2T<Scalar> result{};
        result.Matrix.Data[0][0] = factors.X;
        result.Matrix.Data[1][1] = factors.Y;
        return result;
    }

    /// 各轴相同的缩放。
    ///
    /// 参数是浮点数（由实参推导），因此调用处写 `Transform2::Scaling(2.0)` 即可。传整数字面量会因不满足 floating_point 约束而编译失败 —— 这是刻意的：缩放因子作用在浮点变换上，应当是浮点数。若确实要写 `Scaling(2)`，请改为
    /// `2.0`。
    template <std::floating_point Factor> [[nodiscard]] static constexpr Transform2T Scaling(Factor factor) noexcept {
        return Scaling(Vector2T<Scalar>{factor, factor});
    }

    /// 逆时针旋转 angleRadians 弧度。
    [[nodiscard]] static Transform2T Rotation(Scalar angleRadians) noexcept {
        const Scalar cosine = std::cos(angleRadians);
        const Scalar sine = std::sin(angleRadians);

        Transform2T<Scalar> result{};
        result.Matrix.Data[0][0] = cosine;
        result.Matrix.Data[0][1] = -sine;
        result.Matrix.Data[1][0] = sine;
        result.Matrix.Data[1][1] = cosine;
        return result;
    }

    /// 绕 `origin` 逆时针旋转 `angleRadians` 弧度（Prim 绕中心旋转应使用本工厂）。
    [[nodiscard]] static Transform2T RotationAbout(Point2T<Scalar> origin, Scalar angleRadians) noexcept {
        return Translation(Vector2T<Scalar>{origin.X, origin.Y}) * Rotation(angleRadians) * Translation(Vector2T<Scalar>{-origin.X, -origin.Y});
    }

    /// 关于过 `point`、以 `normal` 为法向的直线做反射：沿法向的分量取反。
    ///
    /// 线性部分是 `I − 2 n nᵀ`，平移列是 `2 (n · point) n`。`normal` 与 `-normal` 是同一面镜子。法向须是单位向量；长度不对时结果不是等距，本函数不做检查，与三维 `Rotation` 对轴的态度相同。
    [[nodiscard]] static constexpr Transform2T Reflection(Point2T<Scalar> point, UnitVector2T<Scalar> normal) noexcept {
        const Scalar nx = normal.X();
        const Scalar ny = normal.Y();
        const Scalar along = Scalar{2} * (nx * point.X + ny * point.Y);

        Transform2T<Scalar> result{};
        result.Matrix.Data[0][0] = Scalar{1} - Scalar{2} * nx * nx;
        result.Matrix.Data[0][1] = -Scalar{2} * nx * ny;
        result.Matrix.Data[1][0] = -Scalar{2} * nx * ny;
        result.Matrix.Data[1][1] = Scalar{1} - Scalar{2} * ny * ny;
        result.Matrix.Data[0][2] = along * nx;
        result.Matrix.Data[1][2] = along * ny;
        return result;
    }

    /// 关于过原点的 X 轴反射，把 `(x, y)` 变成 `(x, −y)`。法向是 +Y。
    [[nodiscard]] static constexpr Transform2T ReflectionX() noexcept {
        return Reflection(Point2T<Scalar>{}, UnitVector2T<Scalar>::FromNormalizedUnchecked(Vector2T<Scalar>{Scalar{0}, Scalar{1}}));
    }

    /// 关于过原点的 Y 轴反射，把 `(x, y)` 变成 `(−x, y)`。法向是 +X。
    [[nodiscard]] static constexpr Transform2T ReflectionY() noexcept {
        return Reflection(Point2T<Scalar>{}, UnitVector2T<Scalar>::FromNormalizedUnchecked(Vector2T<Scalar>{Scalar{1}, Scalar{0}}));
    }

    /// 施加完整仿射变换，输入按**位置**解读。
    ///
    /// 与 operator*(Point2T) 等价，名字更直白。
    [[nodiscard]] constexpr Point2T<Scalar> TransformPoint(Point2T<Scalar> position) const noexcept {
        const MatrixT<Scalar, 3>& m = Matrix;
        return Point2T<Scalar>{
            m.Data[0][0] * position.X + m.Data[0][1] * position.Y + m.Data[0][2],
            m.Data[1][0] * position.X + m.Data[1][1] * position.Y + m.Data[1][2],
        };
    }

    /// 逆变换。线性部分奇异时返回 std::nullopt。
    [[nodiscard]] std::optional<Transform2T<Scalar>> Inverse(Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        const auto inverseMatrix = Matrix.Inverse(tolerance);
        if (!inverseMatrix.has_value()) {
            return std::nullopt;
        }
        return Transform2T<Scalar>{*inverseMatrix};
    }
};

using Transform2 = Transform2T<double>;
using Transform2f = Transform2T<float>;

// ---- 运算符 ----

/// 只施加线性部分，输入按**方向**解读。
template <typename Scalar> [[nodiscard]] constexpr Vector2T<Scalar> operator*(const Transform2T<Scalar>& t, Vector2T<Scalar> direction) noexcept {
    const MatrixT<Scalar, 3>& m = t.Matrix;
    return Vector2T<Scalar>{m.Data[0][0] * direction.X + m.Data[0][1] * direction.Y, m.Data[1][0] * direction.X + m.Data[1][1] * direction.Y};
}

/// 复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr Transform2T<Scalar> operator*(const Transform2T<Scalar>& a, const Transform2T<Scalar>& b) noexcept {
    return Transform2T<Scalar>{a.Matrix * b.Matrix};
}

/// 施加完整仿射变换，输入按**位置**解读（平移生效）。
///
/// 与 TransformPoint 等价。注意它与上面的 operator*(Vector2T) 是**两个语义不同**的重载：点吃平移，方向不吃 —— 两者不可互相替代。
template <typename Scalar> [[nodiscard]] constexpr Point2T<Scalar> operator*(const Transform2T<Scalar>& t, Point2T<Scalar> point) noexcept {
    return t.TransformPoint(point);
}

} // namespace DragonGeo::Linear
