#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point2.hpp>

namespace DragonGeo::Prim {

/// 二维线段：由两个端点 `A`、`B` 给出的有限曲线段。
///
/// 本步只是类型壳：字段与 `IsValid`；曲线协议方法在后续任务添加。
/// 非有限坐标可以存入；`IsValid` 仅在四个分量都有限时为真。
template <typename Scalar>
struct Segment2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point2T<Scalar> A{};
    Linear::Point2T<Scalar> B{};

    /// 两个端点的四个分量均为有限值时为真。零长度（`A == B`）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(B.X) && IsFinite(B.Y);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Segment2T<Scalar> a, Segment2T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B;
}

using Segment2 = Segment2T<double>;
using Segment2f = Segment2T<float>;

} // namespace DragonGeo::Prim
