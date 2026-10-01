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
        std::span<const Linear::Point3T<Scalar>> points) {
        if (points.size() < 2) {
            return std::nullopt;
        }
        for (const Linear::Point3T<Scalar>& point : points) {
            if (!Detail::CoordinatesAreFinite(point)) {
                return std::nullopt;
            }
        }
        return std::optional<Polyline3T>(Polyline3T{
            std::vector<Linear::Point3T<Scalar>>(points.begin(), points.end())});
    }

    [[nodiscard]] constexpr std::size_t PointCount() const noexcept {
        return m_points.size();
    }

    /// `index` 必须小于 `PointCount()`。越界是未定义行为。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Point(std::size_t index) const {
        return m_points[index];
    }

    /// 相邻两点构成的边数，即 `PointCount() - 1`。
    [[nodiscard]] constexpr std::size_t SegmentCount() const noexcept {
        return m_points.size() - 1;
    }

    /// `index` 必须小于 `SegmentCount()`。越界是未定义行为。
    /// 由 `Point(index)` 与 `Point(index + 1)` 现拼，不另存线段。
    [[nodiscard]] constexpr Segment3T<Scalar> Segment(std::size_t index) const {
        return Segment3T<Scalar>{Point(index), Point(index + 1)};
    }

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
    [[nodiscard]] constexpr bool IsClosed() const noexcept {
        return PointCount() >= 2 && Point(0) == Point(PointCount() - 1);
    }

    /// 参数域 `[0, SegmentCount()]`。每一段占长度 1。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, static_cast<Scalar>(SegmentCount())};
    }

    /// `t` 不在域内或非有限时为空。
    /// 整数参数落在顶点上；`t == SegmentCount()` 是最后一个点。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        return Locate(t);
    }

    /// 第一个点。工厂保证至少两个点，因此不为空。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> StartPoint() const noexcept {
        return Point(0);
    }

    /// 最后一个点。闭合时与 `StartPoint` 是同一个点。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> EndPoint() const noexcept {
        return Point(PointCount() - 1);
    }

    /// 参数域中点处的 `PointAt`。它是参数中点，不是弧长中点，也不是重心。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> MidPoint() const noexcept {
        return PointAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
    }

    /// 各边长度之和，非负。零长度边贡献 0。
    [[nodiscard]] Scalar Length() const noexcept {
        Scalar length{0};
        for (std::size_t index = 0; index < SegmentCount(); ++index) {
            length += m_points[index].DistanceTo(m_points[index + 1]);
        }
        return length;
    }

    /// 不闭合时为空。闭合时是矢量面积 `(1/2) Σ Pi × Pi+1` 的长度，非负。
    /// 求和只沿着存储的边；重复的末点是最后一条边的终点，不再当作一个新顶点。
    /// 不要求共面：非平面折线也不投影，直接取这个矢量的长度。恰好为零是值，不是空。
    [[nodiscard]] std::optional<Scalar> Area() const noexcept {
        if (!IsClosed()) {
            return std::nullopt;
        }
        return (VectorAreaSum() * Scalar{0.5}).Length();
    }

    /// 不闭合时为空。
    /// 闭合且矢量面积（`Σ Pi × Pi+1`，不取一半）恰好为零时是 `Degenerate`。
    /// 非零时没有单一的二维绕向：取叉积和绝对值最大的分量，该分量为正是
    /// `CounterClockwise`，否则是 `Clockwise`。绝对值并列时按 X、Y、Z 的顺序
    /// 保留先出现的分量。落在 xy 平面上、法向指向 +Z 时，这与二维逆时针为正一致。
    [[nodiscard]] constexpr std::optional<Winding> Orientation() const noexcept {
        if (!IsClosed()) {
            return std::nullopt;
        }
        const Linear::Vector3T<Scalar> sum = VectorAreaSum();
        if (sum.LengthSquared() == Scalar{0}) {
            return Winding::Degenerate;
        }
        Scalar component = sum.X;
        Scalar magnitude = Core::AbsoluteValue(sum.X);
        const Scalar absY = Core::AbsoluteValue(sum.Y);
        if (absY > magnitude) {
            component = sum.Y;
            magnitude = absY;
        }
        const Scalar absZ = Core::AbsoluteValue(sum.Z);
        if (absZ > magnitude) {
            component = sum.Z;
        }
        if (component > Scalar{0}) {
            return Winding::CounterClockwise;
        }
        return Winding::Clockwise;
    }

    /// 全部顶点的轴对齐包围盒。
    [[nodiscard]] Linear::Box3T<Scalar> Box() const noexcept {
        return Linear::Box3T<Scalar>::FromPoints(m_points);
    }

    /// 不闭合，或闭合但矢量面积恰好为零时为空。
    /// 否则是不重复顶点的平均：丢掉与首点相同的末点，不把面积当权重。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> Centroid() const noexcept {
        if (!IsClosed() || VectorAreaSum().LengthSquared() == Scalar{0}) {
            return std::nullopt;
        }
        const std::size_t count = m_points.size() - 1;
        Linear::Vector3T<Scalar> sum{};
        for (std::size_t index = 0; index < count; ++index) {
            sum = sum + Position(m_points[index]);
        }
        const Scalar divisor = static_cast<Scalar>(count);
        return Linear::Point3T<Scalar>{sum.X / divisor, sum.Y / divisor, sum.Z / divisor};
    }

    /// 点在填充区域内（含边界）时为真。只对 `double` 提供。
    ///
    /// 不闭合，或顶点不共面时为假。顶点共面时，查询点不在该平面上也为假。
    /// 共面且闭合时，先把落在任一条边上的点算作内部；其余点投影到矢量面积所在的
    /// 平面上，用多边形的非零环绕数判断。矢量面积恰好为零、但顶点并不共线时，
    /// 平面改取第一组不共线的三个顶点。全体共线时只有边上的点算内部。
    template <typename S = Scalar>
        requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Contains(Linear::Point3T<S> point) const noexcept {
        if (!IsClosed()) {
            return false;
        }

        const auto collinear = [](Linear::Point3 first, Linear::Point3 second,
                                  Linear::Point3 third) noexcept {
            const auto orient = [](double ax, double ay, double bx, double by, double cx,
                                   double cy) noexcept {
                return Predicates::Orient2d(
                    Linear::Point2{ax, ay}, Linear::Point2{bx, by}, Linear::Point2{cx, cy});
            };
            return orient(first.X, first.Y, second.X, second.Y, third.X, third.Y) == 0
                && orient(first.Y, first.Z, second.Y, second.Z, third.Y, third.Z) == 0
                && orient(first.Z, first.X, second.Z, second.X, third.Z, third.X) == 0;
        };
        const auto onEdge = [&](Linear::Point3 start, Linear::Point3 end,
                                Linear::Point3 query) noexcept {
            if (start == end) {
                return query == start;
            }
            if (!collinear(start, end, query)) {
                return false;
            }
            const Linear::Vector3 chord = end - start;
            const double lengthSquared = chord.LengthSquared();
            if (!(lengthSquared > 0.0)) {
                return query == start || query == end;
            }
            const double parameter = (query - start).Dot(chord) / lengthSquared;
            return parameter >= 0.0 && parameter <= 1.0;
        };
        const auto onBoundary = [&]() noexcept {
            for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
                if (onEdge(m_points[edge], m_points[edge + 1], point)) {
                    return true;
                }
            }
            return false;
        };

        const std::size_t uniqueCount = m_points.size() - 1;
        std::optional<Linear::Point3> basis0;
        std::optional<Linear::Point3> basis1;
        std::optional<Linear::Point3> basis2;
        std::size_t origin = 0;
        std::size_t along = 1;
        while (along < uniqueCount && m_points[along] == m_points[origin]) {
            ++along;
        }
        if (along < uniqueCount) {
            for (std::size_t off = along + 1; off < uniqueCount; ++off) {
                if (!collinear(m_points[origin], m_points[along], m_points[off])) {
                    basis0 = m_points[origin];
                    basis1 = m_points[along];
                    basis2 = m_points[off];
                    break;
                }
            }
        }
        if (!basis0.has_value()) {
            return onBoundary();
        }

        for (std::size_t index = 0; index < uniqueCount; ++index) {
            if (Predicates::Orient3d(*basis0, *basis1, *basis2, m_points[index]) != 0) {
                return false;
            }
        }
        if (Predicates::Orient3d(*basis0, *basis1, *basis2, point) != 0) {
            return false;
        }
        if (onBoundary()) {
            return true;
        }

        Linear::Vector3 normal = VectorAreaSum();
        if (!(normal.LengthSquared() > 0.0) || !Core::IsFinite(normal.LengthSquared())) {
            normal = (*basis1 - *basis0).Cross(*basis2 - *basis0);
        }
        if (!(normal.LengthSquared() > 0.0) || !Core::IsFinite(normal.LengthSquared())) {
            return false;
        }

        const int dropped = DroppedAxis(normal);
        const Linear::Point2 query = Project(point, dropped);
        int winding = 0;
        for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
            const Linear::Point2 start = Project(m_points[edge], dropped);
            const Linear::Point2 next = Project(m_points[edge + 1], dropped);
            if (start.Y <= query.Y) {
                if (next.Y > query.Y && Predicates::Orient2d(start, next, query) > 0) {
                    ++winding;
                }
            } else if (next.Y <= query.Y && Predicates::Orient2d(start, next, query) < 0) {
                --winding;
            }
        }
        return winding != 0;
    }

    /// 点到最近一条边的距离不超过 `tolerance.Resolve(尺度)`。
    /// 尺度是包围盒对角线；对角线不是大于 0 的有限数时用 `1`。
    [[nodiscard]] bool ContainsPoint(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        return std::sqrt(ClosestBoundary(point).DistanceSquared) <= tolerance.Resolve(BoundaryScale());
    }

    /// 最近边上的参数，落在 `[0, SegmentCount()]`。`ContainsPoint` 为假时为空。
    /// 距离相同取较小的参数。闭合折线的接缝返回 `0`，不返回域的上端。
    [[nodiscard]] std::optional<Scalar> ParameterOf(
        Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance = {}) const noexcept {
        if (!ContainsPoint(point, tolerance)) {
            return std::nullopt;
        }
        return ClosestBoundary(point).Parameter;
    }

    /// 第一条边的单位方向。这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> StartTangent() const noexcept {
        return TangentAt(Scalar{0});
    }

    /// 最后一条边的单位方向。`t == SegmentCount()` 也用这一条边。
    /// 这条边没有有限正长度时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> EndTangent() const noexcept {
        return TangentAt(static_cast<Scalar>(SegmentCount()));
    }

    /// 参数中点所在边的单位方向。该边没有有限正长度，或参数不被接受时为空。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> MidTangent() const noexcept {
        return TangentAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
    }

    /// `t` 不在域内、非有限，或所在边没有有限正长度时为空。
    /// `t == SegmentCount()` 用最后一条边。其余参数用 `floor(t)` 所在的边。
    [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> TangentAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        const std::size_t edge = EdgeIndex(t);
        return UnitEdge(m_points[edge], m_points[edge + 1]);
    }

    constexpr void Translate(Linear::Vector3T<Scalar> vector) noexcept {
        for (Linear::Point3T<Scalar>& point : m_points) {
            point = point + vector;
        }
    }

    /// 绕过 `origin`、方向为 `axis` 的轴旋转 `radians` 弧度。
    void Rotate(
        Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept {
        const Linear::Transform3T<Scalar> rotation =
            Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
        for (Linear::Point3T<Scalar>& point : m_points) {
            point = rotation.TransformPoint(point);
        }
    }

    /// 关于过 `point`、法向为 `unitNormal` 的平面反射。
    constexpr void Mirror(
        Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
        const Linear::Transform3T<Scalar> mirror =
            Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
        for (Linear::Point3T<Scalar>& vertex : m_points) {
            vertex = mirror.TransformPoint(vertex);
        }
    }

    /// 反转点序。开放折线的起点与终点对调。
    /// 闭合时接缝仍是原来的 `Point(0)`：反转其余不重复顶点，再把首点接到末尾。
    constexpr void Reverse() noexcept {
        if (IsClosed()) {
            std::reverse(m_points.begin() + 1, m_points.end() - 1);
            return;
        }
        std::reverse(m_points.begin(), m_points.end());
    }

    /// 按值复制整条折线。复制点列可能分配内存。
    [[nodiscard]] Polyline3T Clone() const {
        return *this;
    }

    /// 变换每个点。任一结果分量非有限时返回 `false`，点列保持原样。
    [[nodiscard]] constexpr bool Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
        for (const Linear::Point3T<Scalar>& point : m_points) {
            if (!Detail::CoordinatesAreFinite(transform.TransformPoint(point))) {
                return false;
            }
        }
        for (Linear::Point3T<Scalar>& point : m_points) {
            point = transform.TransformPoint(point);
        }
        return true;
    }

    /// 区间必须落在参数域内且长度大于 0，否则为空。端点非有限或区间倒置同样为空。
    /// 整段落在同一条边上时返回 `Segment3`，跨过顶点时返回 `Polyline3`。
    [[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>> Subcurve(
        Linear::IntervalT<Scalar> interval) const {
        using Curve = std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>;
        if (!Detail::IsFiniteSubinterval(interval, Domain())) {
            return std::nullopt;
        }
        for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
            const Scalar edgeMin = static_cast<Scalar>(edge);
            const Scalar edgeMax = edgeMin + Scalar{1};
            if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
                return Curve{Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)}};
            }
        }

        std::vector<Linear::Point3T<Scalar>> corners;
        corners.push_back(Locate(interval.Min));
        for (std::size_t vertex = 1; vertex < SegmentCount(); ++vertex) {
            const Scalar parameter = static_cast<Scalar>(vertex);
            if (parameter > interval.Min && parameter < interval.Max) {
                corners.push_back(m_points[vertex]);
            }
        }
        corners.push_back(Locate(interval.Max));
        auto polyline = FromPoints(corners);
        if (!polyline.has_value()) {
            return std::nullopt;
        }
        return Curve{std::move(*polyline)};
    }

private:
    explicit Polyline3T(std::vector<Linear::Point3T<Scalar>> points)
        : m_points(std::move(points)) {}

    [[nodiscard]] static constexpr Linear::Vector3T<Scalar> Position(
        Linear::Point3T<Scalar> point) noexcept {
        return {point.X, point.Y, point.Z};
    }

    /// `Σ Pi × Pi+1`，沿每一条存储的边。闭合面积再取一半后的长度。
    [[nodiscard]] constexpr Linear::Vector3T<Scalar> VectorAreaSum() const noexcept {
        Linear::Vector3T<Scalar> sum{};
        for (std::size_t index = 0; index + 1 < m_points.size(); ++index) {
            sum = sum + Position(m_points[index]).Cross(Position(m_points[index + 1]));
        }
        return sum;
    }

    /// `t` 必须已落在参数域内。右端点归到最后一条边。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Locate(Scalar t) const noexcept {
        if (t == static_cast<Scalar>(SegmentCount())) {
            return m_points.back();
        }
        const auto index = static_cast<std::size_t>(t);
        const Scalar local = t - static_cast<Scalar>(index);
        return m_points[index] + (m_points[index + 1] - m_points[index]) * local;
    }

    /// `t == SegmentCount()` 用最后一条边。其余用截断后的边号。
    [[nodiscard]] constexpr std::size_t EdgeIndex(Scalar t) const noexcept {
        if (t == static_cast<Scalar>(SegmentCount())) {
            return SegmentCount() - 1;
        }
        return static_cast<std::size_t>(t);
    }

    /// 边没有有限正长度，或方向无法归一化时为空。
    [[nodiscard]] static std::optional<Linear::UnitVector3T<Scalar>> UnitEdge(
        Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to) noexcept {
        const Scalar length = from.DistanceTo(to);
        if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
            return std::nullopt;
        }
        return (to - from).Normalized();
    }

    /// 包围盒对角线。不是大于 0 的有限数时用 `1`。
    [[nodiscard]] Scalar BoundaryScale() const noexcept {
        const Scalar diagonal = Box().Extent().Length();
        if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
            return diagonal;
        }
        return Scalar{1};
    }

    struct BoundaryLocation {
        Scalar DistanceSquared{};
        Scalar Parameter{};
        Linear::Point3T<Scalar> Point{};
    };

    /// 各边上的最近点。距离相同取较小的参数。闭合折线的参数上端折回接缝 `0`。
    [[nodiscard]] BoundaryLocation ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept {
        BoundaryLocation best{};
        for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
            const Segment3T<Scalar> segment{m_points[edge], m_points[edge + 1]};
            const Linear::Point3T<Scalar> candidate = segment.ClosestPoint(point);
            const Scalar distanceSquared = (point - candidate).LengthSquared();
            const Scalar parameter = BoundaryParameter(
                edge, m_points[edge], m_points[edge + 1], candidate);
            if (edge == 0 || distanceSquared < best.DistanceSquared
                || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
                best = BoundaryLocation{distanceSquared, parameter, candidate};
            }
        }
        return best;
    }

    [[nodiscard]] Scalar BoundaryParameter(
        std::size_t edge,
        Linear::Point3T<Scalar> from,
        Linear::Point3T<Scalar> to,
        Linear::Point3T<Scalar> closest) const noexcept {
        Scalar parameter = static_cast<Scalar>(edge);
        const Scalar length = from.DistanceTo(to);
        if (length > Scalar{0} && Core::IsFinite(length)) {
            const auto direction = (to - from).Normalized();
            if (direction.has_value()) {
                Scalar local = Detail::ProjectParameter(from, *direction, closest) / length;
                if (local < Scalar{0}) {
                    local = Scalar{0};
                } else if (local > Scalar{1}) {
                    local = Scalar{1};
                }
                parameter += local;
            }
        }
        if (IsClosed() && parameter == static_cast<Scalar>(SegmentCount())) {
            parameter = Scalar{0};
        }
        return parameter;
    }

    /// 投影时丢掉的轴：0 是 X，1 是 Y，2 是 Z。绝对值并列时保留先出现的轴。
    [[nodiscard]] static int DroppedAxis(Linear::Vector3 normal) noexcept {
        const double absX = Core::AbsoluteValue(normal.X);
        const double absY = Core::AbsoluteValue(normal.Y);
        const double absZ = Core::AbsoluteValue(normal.Z);
        if (absX >= absY && absX >= absZ) {
            return 0;
        }
        if (absY >= absZ) {
            return 1;
        }
        return 2;
    }

    [[nodiscard]] static Linear::Point2 Project(Linear::Point3 point, int dropped) noexcept {
        if (dropped == 0) {
            return Linear::Point2{point.Y, point.Z};
        }
        if (dropped == 1) {
            return Linear::Point2{point.X, point.Z};
        }
        return Linear::Point2{point.X, point.Y};
    }

    std::vector<Linear::Point3T<Scalar>> m_points;
};

/// 逐点比较存储的点列。按 const 引用传递，避免复制整列。
/// `operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(
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

} // namespace DragonGeo::Prim
