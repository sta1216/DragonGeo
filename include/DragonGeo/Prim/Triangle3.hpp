#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point3.hpp>

namespace DragonGeo::Prim {

/// 三维三角形：由顶点 `A`、`B`、`C` 给出。
///
/// 本步只是类型壳：字段与 `IsValid`；面积与包含判定在后续任务添加。
/// 非有限坐标可以存入；`IsValid` 仅在九个分量都有限时为真。
template <typename Scalar>
struct Triangle3T {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    Linear::Point3T<Scalar> A{};
    Linear::Point3T<Scalar> B{};
    Linear::Point3T<Scalar> C{};

    /// 三个顶点的九个分量均为有限值时为真。零面积（三点共线）仍视为有效。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(A.X) && IsFinite(A.Y) && IsFinite(A.Z) && IsFinite(B.X) && IsFinite(B.Y)
            && IsFinite(B.Z) && IsFinite(C.X) && IsFinite(C.Y) && IsFinite(C.Z);
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Triangle3T<Scalar> a, Triangle3T<Scalar> b) noexcept {
    return a.A == b.A && a.B == b.B && a.C == b.C;
}

using Triangle3 = Triangle3T<double>;
using Triangle3f = Triangle3T<float>;

} // namespace DragonGeo::Prim
