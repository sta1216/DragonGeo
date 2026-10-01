#pragma once

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
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Winding.hpp>

namespace DragonGeo::Prim {

/// 二维折线：至少两个有限点。
///
/// 点列是私有的，只能经 `FromPoints` 构造，因此本类型不是聚合。
/// 边由相邻两点现拼，不另存线段。
/// `Point(0) == Point(PointCount - 1)` 时闭合。
/// 参数域是 `[0, SegmentCount()]`，每一段占长度 1。
template <typename Scalar>
class PolylineT {
public:
    using ScalarType = Scalar;

    /// 空序列、只有一个点，或任一坐标分量非有限时返回空。
    [[nodiscard]] static std::optional<PolylineT> FromPoints(
        std::span<const Linear::Point2T<Scalar>> points);

    [[nodiscard]] std::size_t PointCount() const noexcept;

    /// `index` 必须小于 `PointCount()`。越界是未定义行为。
    [[nodiscard]] Linear::Point2T<Scalar> Point(std::size_t index) const;

    /// 相邻两点构成的边数，即 `PointCount() - 1`。
    [[nodiscard]] std::size_t SegmentCount() const noexcept;

    /// `index` 必须小于 `SegmentCount()`。越界是未定义行为。
    /// 由 `Point(index)` 与 `Point(index + 1)` 现拼，不另存线段。
    [[nodiscard]] Segment2T<Scalar> Segment(std::size_t index) const;

    /// 每个点的分量都有限时为真。零长度边仍有效。
    /// 工厂已经拒绝非有限点，成功构造的折线因此为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        for (const Linear::Point2T<Scalar>& point : m_points) {
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
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PointAt(Scalar t) const noexcept;

    /// 第一个点。工厂保证至少两个点，因此不为空。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> StartPoint() const noexcept;

    /// 最后一个点。闭合时与 `StartPoint` 是同一个点。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> EndPoint() const noexcept;

    /// 参数域中点处的 `PointAt`。它是参数中点，不是弧长中点，也不是重心。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> MidPoint() const noexcept;

    /// 各边长度之和，非负。零长度边贡献 0。
    [[nodiscard]] Scalar Length() const noexcept;

    /// 不闭合时为空。闭合时是鞋带公式面积的绝对值。
    /// 求和只沿着存储的边；重复的末点是最后一条边的终点，不再当作一个新顶点。
    /// 恰好为零是值，不是空。
    [[nodiscard]] std::optional<Scalar> Area() const noexcept;

    /// 不闭合时为空。闭合且有向面积恰好为零时是 `Degenerate`。
    /// 有向面积为正是 `CounterClockwise`，为负是 `Clockwise`。
    [[nodiscard]] std::optional<Winding> Orientation() const noexcept;

    /// 全部顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box2T<Scalar> Box() const noexcept;

    /// 不闭合，或闭合但有向面积恰好为零时为空。
    /// 否则是不重复顶点的平均：丢掉与首点相同的末点，不把面积当权重。
    [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Centroid() const noexcept;

    /// 点在填充区域内（含边界）时为真。只对 `double` 提供。
    ///
    /// 不闭合时为假。闭合时，落在任一条边上的点算内部；其余点用非零环绕数。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Contains(Linear::Point2T<S> point) const noexcept;

    /// 点到最近一条边的距离不超过 `tolerance.Resolve(尺度)`。
    /// 尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 最近边上的参数，落在 `[0, SegmentCount()]`。`ContainsPoint` 为假时为空。
    /// 距离相同取较小的参数。闭合折线的接缝返回 `0`，不返回域的上端。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept;

    /// 第一条边的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> StartTangent() const noexcept;

    /// 最后一条边的单位方向。`t == SegmentCount()` 也用这一条边。
    /// 这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> EndTangent() const noexcept;

    /// 参数中点所在边的单位方向。该边没有有限正长度，或参数不被接受时为空。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> MidTangent() const noexcept;

    /// `t` 不在域内、非有限，或所在边没有有限正长度时为空。
    /// `t == SegmentCount()` 用最后一条边。其余参数用 `floor(t)` 所在的边。
    [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> TangentAt(Scalar t) const noexcept;

    void Translate(Linear::Vector2T<Scalar> vector) noexcept;

    /// 绕 `center` 逆时针旋转 `radians` 弧度。
    void Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept;

    /// 关于过 `point`、法向为 `unitNormal` 的直线反射。
    void Mirror(
        Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept;

    /// 反转点序。开放折线的起点与终点对调。
    /// 闭合时接缝仍是原来的 `Point(0)`：反转其余不重复顶点，再把首点接到末尾。
    void Reverse() noexcept;

    /// 按值复制整条折线。复制点列可能分配内存。
    [[nodiscard]] PolylineT Clone() const;

    /// 变换每个点。任一结果分量非有限时返回 `false`，点列保持原样。
    [[nodiscard]] bool Transform(const Linear::Transform2T<Scalar>& transform) noexcept;

    /// 区间必须落在参数域内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。
    /// 整段落在同一条边上时返回 `Segment2`，跨过顶点时返回 `Polyline`。
    [[nodiscard]] std::optional<std::variant<Segment2T<Scalar>, PolylineT>> Subcurve(
        Linear::IntervalT<Scalar> interval) const;

private:
    explicit PolylineT(std::vector<Linear::Point2T<Scalar>> points);

    /// 鞋带求和 `Σ Pi × Pi+1`，沿每一条存储的边。面积再取一半的绝对值。
    [[nodiscard]] Scalar SignedAreaSum() const noexcept;

    /// `t` 必须已落在参数域内。右端点归到最后一条边。
    [[nodiscard]] Linear::Point2T<Scalar> Locate(Scalar t) const noexcept;

    /// `t == SegmentCount()` 用最后一条边。其余用截断后的边号。
    [[nodiscard]] std::size_t EdgeIndex(Scalar t) const noexcept;

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector2T<Scalar>> UnitEdge(
        Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept;

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept;

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point2T<Scalar> Point{};
    };

    /// 各边上的最近点。距离相同取较小的参数。闭合折线的参数上端折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point2T<Scalar> point) const noexcept;

    [[nodiscard]] Scalar BoundaryParameter(
        std::size_t edge,
        Linear::Point2T<Scalar> from,
        Linear::Point2T<Scalar> to,
        Linear::Point2T<Scalar> closest) const noexcept;

    std::vector<Linear::Point2T<Scalar>> m_points;
};

/// 逐点比较存储的点列。按 const 引用传递，避免复制整列。
/// `operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] bool operator==(const PolylineT<Scalar>& a, const PolylineT<Scalar>& b) noexcept {
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

using Polyline = PolylineT<double>;
using Polylinef = PolylineT<float>;

extern template struct PolylineT<double>;
extern template struct PolylineT<float>;

} // namespace DragonGeo::Prim
