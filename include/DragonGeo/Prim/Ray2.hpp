#pragma once

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>

namespace DragonGeo::Prim {

/// 二维射线：从 `Origin` 沿 `Direction` 伸出的半直线。
///
/// 本步只是类型壳：字段与 `IsValid`；曲线协议方法在后续任务添加。
/// 非有限坐标可以存入；`IsValid` 只检查分量是否有限，不检查方向是否已归一化。
template <typename Scalar>
struct Ray2T {
    using ScalarType = Scalar;

    // 同 Point2T / Box2T：不声明任何构造函数，以保持聚合性。
    // 单位向量没有默认构造函数，故 Direction 不写默认成员初始化器。
    Linear::Point2T<Scalar> Origin{};
    Linear::UnitVector2T<Scalar> Direction;

    /// 原点与方向的每个分量均为有限值时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        using Core::IsFinite;
        return IsFinite(Origin.X) && IsFinite(Origin.Y) && IsFinite(Direction.X())
            && IsFinite(Direction.Y());
    }
};

/// 逐字段比较。`operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(Ray2T<Scalar> a, Ray2T<Scalar> b) noexcept {
    return a.Origin == b.Origin && a.Direction == b.Direction;
}

using Ray2 = Ray2T<double>;
using Ray2f = Ray2T<float>;

} // namespace DragonGeo::Prim
