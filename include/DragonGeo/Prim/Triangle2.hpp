#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point2.hpp>

namespace DragonGeo::Prim {

/// 二维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 本步只是类型壳：字段与 `IsValid`；面积与包含判定在后续任务添加。
/// 非有限坐标可以存入；`IsValid` 仅在六个分量都有限时为真。
template <typename Scalar>
struct Triangle2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point2T<Scalar> A{};
    Linear::Point2T<Scalar> B{};
    Linear::Point2T<Scalar> C{};

    /// 三个顶点的六个分量均为有限值时为真。零面积（三点共线）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(B.X) && IsFinite(B.Y) && IsFinite(C.X)
            && IsFinite(C.Y);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Triangle2T<Scalar> a, Triangle2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle2 = Triangle2T<double>;
using Triangle2f = Triangle2T<float>;

} // namespace DragonGeo::Prim
