#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>
#include <span>
#include <utility>
#include <variant>
#include <vector>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Linear/Transform3.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Segment3.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

/// 三维折线：至少两个有限点。
///
/// 点列是私有的，只能经 `FromPoints` 构造，因此本类型不是聚合。
/// 边由相邻两点现拼，不另存线段。
/// `Point(0) == Point(PointCount - 1)` 时闭合。闭合不要求共面。
/// 参数域是 `[0, SegmentCount()]`，每一段占长度 1。
template <typename Scalar>
class Polyline3T {
public:
    using ScalarType = Scalar;

    /// 空序列、只有一个点，或任一坐标分量非有限时返回空。
    [[nodiscard]] static std::optional<Polyline3T> FromPoints(
        std::span<const Linear::Point3T<Scalar>> points);

    [[nodiscard]] std::size_t PointCount() const noexcept;

    /// `index` 必须小于 `PointCount()`。越界是未定义行为。
    [[nodiscard]] Linear::Point3T<Scalar> Point(std::size_t index) const;

    /// 相邻两点构成的边数，即 `PointCount() - 1`。
    [[nodiscard]] std::size_t SegmentCount() const noexcept;

    /// `index` 必须小于 `SegmentCount()`。越界是未定义行为。
    /// 由 `Point(index)` 与 `Point(index + 1)` 现拼，不另存线段。
    [[nodiscard]] Segment3T<Scalar> Segment(std::size_t index) const;

    /// 每个点的分量都有限时为真。零长度边仍有效。
    /// 工厂已经拒绝非有限点，成功构造的折线因此为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        for (const Linear::Point3T<Scalar>& point : m_points) {
            if (!Detail::CoordinatesAreFinite(point)) {
                return false;
            }
        }
        return true;
    }

    /// 至少两个点，且首尾用 `operator==` 精确重合。不容差。
    [[nodiscard]] bool IsClosed() const noexcept;

    /// 参数域 `[0, SegmentCount()]`。每一段占长度 1。
    [[nodiscard]] Linear::IntervalT<Scalar> Domain() const noexcept;

    /// `t` 不在域内或非有限时为空。
    /// 整数参数落在顶点上；`t == SegmentCount()` 是最后一个点。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 第一个点。工厂保证至少两个点，因此不为空。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept;

    /// 最后一个点。闭合时与 `StartPoint` 是同一个点。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept;

    /// 参数域中点处的 `PointAt`。它是参数中点，不是弧长中点，也不是重心。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept;

    /// 各边长度之和，非负。零长度边贡献 0。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 不闭合时为空。闭合时是矢量面积 `(1/2) Σ Pi × Pi+1` 的长度，非负。
    /// 求和只沿着存储的边；重复的末点是最后一条边的终点，不再当作一个新顶点。
    /// 不要求共面：非平面折线也不投影，直接取这个矢量的长度。恰好为零是值，不是空。
    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    /// 不闭合时为空。
    /// 闭合且矢量面积（`Σ Pi × Pi+1`，不取一半）恰好为零时是 `Degenerate`。
    /// 非零时没有单一的二维绕向：取叉积和绝对值最大的分量，该分量为正是
    /// `CounterClockwise`，否则是 `Clockwise`。绝对值并列时按 X、Y、Z 的顺序
    /// 保留先出现的分量。落在 xy 平面上、法向指向 +Z 时，这与二维逆时针为正一致。
    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    /// 全部顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box3T<Scalar> Box() const noexcept;

    /// 不闭合，或闭合但矢量面积恰好为零时为空。
    /// 否则是不重复顶点的平均：丢掉与首点相同的末点，不把面积当权重。
    [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept;

    /// 点在填充区域内（含边界）时为真。只对 `double` 提供。
    ///
    /// 不闭合，或顶点不共面时为假。顶点共面时，查询点不在该平面上也为假。
    /// 共面且闭合时，先把落在任一条边上的点算作内部；其余点投影到矢量面积所在的
    /// 平面上，用多边形的非零环绕数判断。矢量面积恰好为零、但顶点并不共线时，
    /// 平面改取第一组不共线的三个顶点。全体共线时只有边上的点算内部。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Contains(Linear::Point3T<S> point) const noexcept;

    /// 点到最近一条边的距离不超过 `tolerance.Resolve(尺度)`。
    /// 尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 最近边上的参数，落在 `[0, SegmentCount()]`。`ContainsPoint` 为假时为空。
    /// 距离相同取较小的参数。闭合折线的接缝返回 `0`，不返回域的上端。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 第一条边的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept;

    /// 最后一条边的单位方向。`t == SegmentCount()` 也用这一条边。
    /// 这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept;

    /// 参数中点所在边的单位方向。该边没有有限正长度，或参数不被接受时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept;

    /// `t` 不在域内、非有限，或所在边没有有限正长度时为空。
    /// `t == SegmentCount()` 用最后一条边。其余参数用 `floor(t)` 所在的边。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> TangentAt(Scalar t) const noexcept;

    void Translate(Linear::Vector3T<Scalar> vector) noexcept;

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。
    void Rotate(
        Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    void Mirror(
        Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept;

    /// 反转点序。开放折线的起点与终点对调。
    /// 闭合时接缝仍是原来的 `Point(0)`：反转其余不重复顶点，再把首点接到末尾。
    void Reverse() noexcept;

    /// 按值复制整条折线。复制点列可能分配内存。
    [[nodiscard]] Polyline3T Clone() const;

    /// 变换每个点。任一结果分量非有限时返回 `false`，点列保持原样。
    [[nodiscard]] bool Transform(const Linear::Transform3T<Scalar>& transform) noexcept;

    /// 区间必须落在参数域内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。
    /// 整段落在同一条边上时返回 `Segment3`，跨过顶点时返回 `Polyline3`。
    [[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>> Subcurve(
        Linear::IntervalT<Scalar> interval) const;

private:
    explicit Polyline3T(std::vector<Linear::Point3T<Scalar>> points);

    [[nodiscard]] static Linear::Vector3T<Scalar> Position(
        Linear::Point3T<Scalar> point) noexcept;

    /// `Σ Pi × Pi+1`，沿每一条存储的边。闭合面积再取一半后的长度。
    [[nodiscard]] Linear::Vector3T<Scalar> VectorAreaSum() const noexcept;

    /// `t` 必须已落在参数域内。右端点归到最后一条边。
    [[nodiscard]] Linear::Point3T<Scalar> Locate(Scalar t) const noexcept;

    /// `t == SegmentCount()` 用最后一条边。其余用截断后的边号。
    [[nodiscard]] std::size_t EdgeIndex(Scalar t) const noexcept;

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector3T<Scalar>> UnitEdge(
        Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to) noexcept;

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept;

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point3T<Scalar> Point{};
    };

    /// 各边上的最近点。距离相同取较小的参数。闭合折线的参数上端折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept;

    [[nodiscard]] Scalar BoundaryParameter(
        std::size_t edge,
        Linear::Point3T<Scalar> from,
        Linear::Point3T<Scalar> to,
        Linear::Point3T<Scalar> closest) const noexcept;

    /// 投影时丢掉的轴：0 是 X，1 是 Y，2 是 Z。绝对值并列时保留先出现的轴。
    [[nodiscard]] static int DroppedAxis(Linear::Vector3 normal) noexcept;

    [[nodiscard]] static Linear::Point2 Project(Linear::Point3 point, int dropped) noexcept;

    std::vector<Linear::Point3T<Scalar>> m_points;
};

/// 逐点比较存储的点列。按 const 引用传递，避免复制整列。
/// `operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] bool operator==(
    const Polyline3T<Scalar>& a, const Polyline3T<Scalar>& b) noexcept {
    if (a.PointCount() != b.PointCount()) {
        return false;
    }
    for (std::size_t index = 0; index < a.PointCount(); ++index) {
        if (!(a.Point(index) == b.Point(index))) {
            return false;
        }
    }
    return true;
}

using Polyline3 = Polyline3T<double>;
using Polyline3f = Polyline3T<float>;


extern template struct Polyline3T<double>;
extern template struct Polyline3T<float>;
} // namespace DragonGeo::Prim
