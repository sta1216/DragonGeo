#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 二维仿射变换，内部为 3×3 齐次矩阵（行主序）。
/// 两种「施加」语义的分工同 Transform3T，详见该类型的文档。
template <typename Scalar>
struct Transform2T {
    using scalar_type = Scalar;

    MatrixT<Scalar, 3> matrix = MatrixT<Scalar, 3>::identity();

    [[nodiscard]] constexpr bool operator==(const Transform2T&) const noexcept = default;

    /// 单位变换。
    [[nodiscard]] static constexpr Transform2T identity() noexcept {
        return Transform2T<Scalar>{};
    }

    [[nodiscard]] static constexpr Transform2T translation(Vector2T<Scalar> offset) noexcept {
        Transform2T<Scalar> result{};
        result.matrix.data[0][2] = offset.x;
        result.matrix.data[1][2] = offset.y;
        return result;
    }

    [[nodiscard]] static constexpr Transform2T scaling(Vector2T<Scalar> factors) noexcept {
        Transform2T<Scalar> result{};
        result.matrix.data[0][0] = factors.x;
        result.matrix.data[1][1] = factors.y;
        return result;
    }

    /// 各轴相同的缩放。
    ///
    /// 参数是浮点数（由实参推导），因此调用处写 `Transform2::scaling(2.0)` 即可。
    /// 传整数字面量会因不满足 floating_point 约束而编译失败 —— 这是刻意的：
    /// 缩放因子作用在浮点变换上，应当是浮点数。若确实要写 `scaling(2)`，请改为
    /// `2.0`。
    template <std::floating_point Factor>
    [[nodiscard]] static constexpr Transform2T scaling(Factor factor) noexcept {
        return scaling(Vector2T<Scalar>{factor, factor});
    }

    /// 逆时针旋转 angle_radians 弧度。
    [[nodiscard]] static Transform2T rotation(Scalar angle_radians) noexcept {
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
    [[nodiscard]] constexpr Vector2T<Scalar> apply(Vector2T<Scalar> position) const noexcept {
        const MatrixT<Scalar, 3>& m = matrix;
        return Vector2T<Scalar>{
            m.data[0][0] * position.x + m.data[0][1] * position.y + m.data[0][2],
            m.data[1][0] * position.x + m.data[1][1] * position.y + m.data[1][2],
        };
    }

    /// 逆变换。线性部分奇异时返回 std::nullopt。
    [[nodiscard]] std::optional<Transform2T<Scalar>> inverse(
        core::Tolerance tolerance = {}) const noexcept {
        const auto inverse_matrix = matrix.inverse(tolerance);
        if (!inverse_matrix.has_value()) {
            return std::nullopt;
        }
        return Transform2T<Scalar>{*inverse_matrix};
    }
};

using Transform2 = Transform2T<double>;
using Transform2f = Transform2T<float>;

// ---- 运算符 ----

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

} // namespace GeoCore::linear
