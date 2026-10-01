#include <DragonGeo/Prim/Polyline.hpp>

#include <algorithm>
#include <cmath>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] std::optional<PolylineT<Scalar>> PolylineT<Scalar>::FromPoints(
    std::span<const Linear::Point2T<Scalar>> points) {
    if (points.size() < 2) {
        return std::nullopt;
    }
    for (const Linear::Point2T<Scalar>& point : points) {
        if (!Detail::CoordinatesAreFinite(point)) {
            return std::nullopt;
        }
    }
    return std::optional<PolylineT>(
        PolylineT{std::vector<Linear::Point2T<Scalar>>(points.begin(), points.end())});
}

template <typename Scalar>
[[nodiscard]] std::size_t PolylineT<Scalar>::PointCount() const noexcept {
    return m_points.size();
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> PolylineT<Scalar>::Point(std::size_t index) const {
    return m_points[index];
}

template <typename Scalar>
[[nodiscard]] std::size_t PolylineT<Scalar>::SegmentCount() const noexcept {
    return m_points.size() - 1;
}

template <typename Scalar>
[[nodiscard]] Segment2T<Scalar> PolylineT<Scalar>::Segment(std::size_t index) const {
    return Segment2T<Scalar>{Point(index), Point(index + 1)};
}

template <typename Scalar>
[[nodiscard]] bool PolylineT<Scalar>::IsClosed() const noexcept {
    return PointCount() >= 2 && Point(0) == Point(PointCount() - 1);
}

template <typename Scalar>
[[nodiscard]] Linear::IntervalT<Scalar> PolylineT<Scalar>::Domain() const noexcept {
    return {Scalar{0}, static_cast<Scalar>(SegmentCount())};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::StartPoint() const noexcept {
    return Point(0);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::EndPoint() const noexcept {
    return Point(PointCount() - 1);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::MidPoint() const noexcept {
    return PointAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar>
[[nodiscard]] Scalar PolylineT<Scalar>::Length() const noexcept {
    Scalar length{0};
    for (std::size_t index = 0; index < SegmentCount(); ++index) {
        length += m_points[index].DistanceTo(m_points[index + 1]);
    }
    return length;
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> PolylineT<Scalar>::Area() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    return Core::AbsoluteValue(SignedAreaSum()) * Scalar{0.5};
}

template <typename Scalar>
[[nodiscard]] std::optional<Winding> PolylineT<Scalar>::Orientation() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    const Scalar sum = SignedAreaSum();
    if (sum > Scalar{0}) {
        return Winding::CounterClockwise;
    }
    if (sum < Scalar{0}) {
        return Winding::Clockwise;
    }
    return Winding::Degenerate;
}

template <typename Scalar>
[[nodiscard]] Linear::Box2T<Scalar> PolylineT<Scalar>::Box() const noexcept {
    return Linear::Box2T<Scalar>::FromPoints(m_points);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::Centroid() const noexcept {
    if (!IsClosed() || SignedAreaSum() == Scalar{0}) {
        return std::nullopt;
    }
    const std::size_t count = m_points.size() - 1;
    Linear::Vector2T<Scalar> sum{};
    for (std::size_t index = 0; index < count; ++index) {
        sum = sum + Linear::Vector2T<Scalar>{m_points[index].X, m_points[index].Y};
    }
    const Scalar divisor = static_cast<Scalar>(count);
    return Linear::Point2T<Scalar>{sum.X / divisor, sum.Y / divisor};
}

template <typename Scalar>
template <typename S>
    requires std::same_as<Scalar, double> && std::same_as<S, double>
[[nodiscard]] bool PolylineT<Scalar>::Contains(Linear::Point2T<S> point) const noexcept {
    if (!IsClosed()) {
        return false;
    }
    const auto onEdge = [](Linear::Point2 start, Linear::Point2 end, Linear::Point2 query) noexcept {
        if (Predicates::Orient2d(start, end, query) != 0) {
            return false;
        }
        if (start == end) {
            return query == start;
        }
        const Linear::Vector2 chord = end - start;
        const double lengthSquared = chord.LengthSquared();
        if (!(lengthSquared > 0.0)) {
            return query == start || query == end;
        }
        const double parameter = (query - start).Dot(chord) / lengthSquared;
        return parameter >= 0.0 && parameter <= 1.0;
    };
    for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
        if (onEdge(m_points[edge], m_points[edge + 1], point)) {
            return true;
        }
    }

    int winding = 0;
    for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
        const Linear::Point2& start = m_points[edge];
        const Linear::Point2& end = m_points[edge + 1];
        if (start.Y <= point.Y) {
            if (end.Y > point.Y && Predicates::Orient2d(start, end, point) > 0) {
                ++winding;
            }
        } else if (end.Y <= point.Y && Predicates::Orient2d(start, end, point) < 0) {
            --winding;
        }
    }
    return winding != 0;
}

template <typename Scalar>
[[nodiscard]] bool PolylineT<Scalar>::ContainsPoint(
    Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return ClosestBoundary(point).DistanceSquared
        <= tolerance.Resolve(BoundaryScale()) * tolerance.Resolve(BoundaryScale());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> PolylineT<Scalar>::ParameterOf(
    Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestBoundary(point).Parameter;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::StartTangent() const noexcept {
    return TangentAt(Scalar{0});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::EndTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()));
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::MidTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const std::size_t edge = EdgeIndex(t);
    return UnitEdge(m_points[edge], m_points[edge + 1]);
}

template <typename Scalar>
void PolylineT<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    for (Linear::Point2T<Scalar>& point : m_points) {
        point = point + vector;
    }
}

template <typename Scalar>
void PolylineT<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation =
        Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    for (Linear::Point2T<Scalar>& point : m_points) {
        point = rotation.TransformPoint(point);
    }
}

template <typename Scalar>
void PolylineT<Scalar>::Mirror(
    Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror =
        Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    for (Linear::Point2T<Scalar>& vertex : m_points) {
        vertex = mirror.TransformPoint(vertex);
    }
}

template <typename Scalar>
void PolylineT<Scalar>::Reverse() noexcept {
    if (IsClosed()) {
        std::reverse(m_points.begin() + 1, m_points.end() - 1);
        return;
    }
    std::reverse(m_points.begin(), m_points.end());
}

template <typename Scalar>
[[nodiscard]] PolylineT<Scalar> PolylineT<Scalar>::Clone() const {
    return *this;
}

template <typename Scalar>
[[nodiscard]] bool PolylineT<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    for (const Linear::Point2T<Scalar>& point : m_points) {
        if (!Detail::CoordinatesAreFinite(transform.TransformPoint(point))) {
            return false;
        }
    }
    for (Linear::Point2T<Scalar>& point : m_points) {
        point = transform.TransformPoint(point);
    }
    return true;
}

template <typename Scalar>
[[nodiscard]] std::optional<std::variant<Segment2T<Scalar>, PolylineT<Scalar>>>
PolylineT<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
    using Curve = std::variant<Segment2T<Scalar>, PolylineT<Scalar>>;
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
        const Scalar edgeMin = static_cast<Scalar>(edge);
        const Scalar edgeMax = edgeMin + Scalar{1};
        if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
            return Curve{Segment2T<Scalar>{Locate(interval.Min), Locate(interval.Max)}};
        }
    }

    std::vector<Linear::Point2T<Scalar>> corners;
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

template <typename Scalar>
PolylineT<Scalar>::PolylineT(std::vector<Linear::Point2T<Scalar>> points)
    : m_points(std::move(points)) {}

template <typename Scalar>
[[nodiscard]] Scalar PolylineT<Scalar>::SignedAreaSum() const noexcept {
    Scalar sum{0};
    for (std::size_t index = 0; index + 1 < m_points.size(); ++index) {
        const Linear::Vector2T<Scalar> current{m_points[index].X, m_points[index].Y};
        const Linear::Vector2T<Scalar> next{m_points[index + 1].X, m_points[index + 1].Y};
        sum += current.Cross(next);
    }
    return sum;
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> PolylineT<Scalar>::Locate(Scalar t) const noexcept {
    if (t == static_cast<Scalar>(SegmentCount())) {
        return m_points.back();
    }
    const auto index = static_cast<std::size_t>(t);
    const Scalar local = t - static_cast<Scalar>(index);
    return m_points[index] + (m_points[index + 1] - m_points[index]) * local;
}

template <typename Scalar>
[[nodiscard]] std::size_t PolylineT<Scalar>::EdgeIndex(Scalar t) const noexcept {
    if (t == static_cast<Scalar>(SegmentCount())) {
        return SegmentCount() - 1;
    }
    return static_cast<std::size_t>(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::UnitEdge(
    Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept {
    const Scalar length = from.DistanceTo(to);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return (to - from).Normalized();
}

template <typename Scalar>
[[nodiscard]] Scalar PolylineT<Scalar>::BoundaryScale() const noexcept {
    const Scalar diagonal = Box().Extent().Length();
    if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
        return diagonal;
    }
    return Scalar{1};
}

template <typename Scalar>
[[nodiscard]] typename PolylineT<Scalar>::BoundaryLocation PolylineT<Scalar>::ClosestBoundary(
    Linear::Point2T<Scalar> point) const noexcept {
    BoundaryLocation best{};
    for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
        const Segment2T<Scalar> segment{m_points[edge], m_points[edge + 1]};
        const Linear::Point2T<Scalar> candidate = segment.ClosestPoint(point);
        const Scalar distanceSquared = (point - candidate).LengthSquared();
        const Scalar parameter =
            BoundaryParameter(edge, m_points[edge], m_points[edge + 1], candidate);
        if (edge == 0 || distanceSquared < best.DistanceSquared
            || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
            best = BoundaryLocation{distanceSquared, parameter, candidate};
        }
    }
    return best;
}

template <typename Scalar>
[[nodiscard]] Scalar PolylineT<Scalar>::BoundaryParameter(
    std::size_t edge,
    Linear::Point2T<Scalar> from,
    Linear::Point2T<Scalar> to,
    Linear::Point2T<Scalar> closest) const noexcept {
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

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::PolylineT<double>;
template struct DragonGeo::Prim::PolylineT<float>;
template bool DragonGeo::Prim::PolylineT<double>::Contains<double>(
    DragonGeo::Linear::Point2T<double>) const noexcept;
