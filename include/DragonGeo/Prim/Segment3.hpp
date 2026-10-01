#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point3.hpp>

namespace DragonGeo::Prim {

/// 三维线段：由两个端点 `A`、`B` 给出的有限曲线段。
///
/// 本步只是类型壳：字段与 `IsValid`；曲线协议方法在后续任务添加。
/// 非有限坐标可以存入；`IsValid` 仅在六个分量都有限时为真。
template <typename Scalar>
struct Segment3T {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point3T<Scalar> A{};
    Linear::Point3T<Scalar> B{};

    /// 两个端点的六个分量均为有限值时为真。零长度（`A == B`）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(A.Z) && IsFinite(B.X) && IsFinite(B.Y)
            && IsFinite(B.Z);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Segment3T<Scalar> a, Segment3T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B;
}

using Segment3 = Segment3T<double>;
using Segment3f = Segment3T<float>;

} // namespace DragonGeo::Prim
