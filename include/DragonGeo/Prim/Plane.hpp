#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>

namespace DragonGeo::Prim {

/// 平面：过 `Origin`、法线为 `Normal` 的平面。
///
/// `SignedDistance` 是 `Normal · (Point - Origin)`，法线一侧为正，平面上为 0。
/// `Distance` 是它的绝对值。`ClosestPoint` 是垂足；点已在平面上时就是该点。
/// `Flipped` 取反法线、保持原点，得到的平面与原平面表示不相等。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查法线是否已归一化。
template <typename Scalar>
struct PlaneT {
    using ScalarType = Scalar;

    // 同 Point3T / Box2T：不声明任何构造函数，以保持聚合性。
    // 单位向量没有默认构造函数，故 Normal 不写默认成员初始化器。
    Linear::Point3T<Scalar> Origin{};
    Linear::UnitVector3T<Scalar> Normal;

    /// 原点与法线的每个分量均为有限值时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(Origin.X) && IsFinite(Origin.Y) && IsFinite(Origin.Z) && IsFinite(Normal.X())
            && IsFinite(Normal.Y()) && IsFinite(Normal.Z());
    }

    /// `Normal · (point - Origin)`。法线一侧为正，平面上为 0。
    [[nodiscard]] constexpr Scalar SignedDistance(Linear::Point3T<Scalar> point) const noexcept {
        return (point - Origin).Dot(Normal.AsVector());
    }

    /// `SignedDistance` 的绝对值。
    [[nodiscard]] constexpr Scalar Distance(Linear::Point3T<Scalar> point) const noexcept {
        return Core::AbsoluteValue(SignedDistance(point));
    }

    /// 点到平面的垂足：`point - SignedDistance(point) * Normal`。
    /// 点已在平面上时返回该点本身。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> ClosestPoint(
        Linear::Point3T<Scalar> point) const noexcept {
        return point - Normal * SignedDistance(point);
    }

    /// 法线取反，原点不变。结果与原平面表示不相等。
    [[nodiscard]] constexpr PlaneT Flipped() const noexcept {
        return PlaneT{Origin, -Normal};
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(PlaneT<Scalar> a, PlaneT<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Normal == b.Normal;
}

using Plane = PlaneT<double>;
using Planef = PlaneT<float>;

} // namespace DragonGeo::Prim
