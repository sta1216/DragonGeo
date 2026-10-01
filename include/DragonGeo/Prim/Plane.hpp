#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>

namespace DragonGeo::Prim {

/// 平面：过 `Origin`、法线为 `Normal` 的平面。
///
/// 本步只是类型壳：字段与 `IsValid`；求交等方法在后续任务添加。
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
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(PlaneT<Scalar> a, PlaneT<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Normal == b.Normal;
}

using Plane = PlaneT<double>;
using Planef = PlaneT<float>;

} // namespace DragonGeo::Prim
