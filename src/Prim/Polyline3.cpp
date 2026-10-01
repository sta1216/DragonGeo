#include <DragonGeo/Prim/Polyline3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] std::optional<Polyline3T<Scalar>> Polyline3T<Scalar>::FromPoints(std::span<const Linear::Point3T<Scalar>> points) {
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

template <typename Scalar>
[[nodiscard]] std::size_t Polyline3T<Scalar>::PointCount() const noexcept {
    return m_points.size();
}

template <typename Scalar>
[[nodiscard]] Linear::Point3T<Scalar> Polyline3T<Scalar>::Point(std::size_t index) const {
    return m_points[index];
}

template <typename Scalar>
[[nodiscard]] std::size_t Polyline3T<Scalar>::SegmentCount() const noexcept {
    return m_points.size() - 1;
}

template <typename Scalar>
[[nodiscard]] Segment3T<Scalar> Polyline3T<Scalar>::Segment(std::size_t index) const {
    return Segment3T<Scalar>{Point(index), Point(index + 1)};
}

template <typename Scalar>
[[nodiscard]] bool Polyline3T<Scalar>::IsClosed() const noexcept {
    return PointCount() >= 2 && Point(0) == Point(PointCount() - 1);
}

template <typename Scalar>
[[nodiscard]] Linear::IntervalT<Scalar> Polyline3T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, static_cast<Scalar>(SegmentCount())};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::StartPoint() const noexcept {
    return Point(0);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::EndPoint() const noexcept {
    return Point(PointCount() - 1);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::MidPoint() const noexcept {
    return PointAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar>
[[nodiscard]] Scalar Polyline3T<Scalar>::Length() const noexcept {
    Scalar length{0};
    for (std::size_t index = 0; index < SegmentCount(); ++index) {
        length += m_points[index].DistanceTo(m_points[index + 1]);
    }
    return length;
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Polyline3T<Scalar>::Area() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    return (VectorAreaSum() * Scalar{0.5}).Length();
}

template <typename Scalar>
[[nodiscard]] std::optional<Winding> Polyline3T<Scalar>::Orientation() const noexcept {
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

template <typename Scalar>
[[nodiscard]] Linear::Box3T<Scalar> Polyline3T<Scalar>::Box() const noexcept {
    return Linear::Box3T<Scalar>::FromPoints(m_points);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::Centroid() const noexcept {
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

template <typename Scalar>
template <typename S>
    requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Polyline3T<Scalar>::Contains(Linear::Point3T<S> point) const noexcept {
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

template <typename Scalar>
[[nodiscard]] bool Polyline3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    return std::sqrt(ClosestBoundary(point).DistanceSquared) <= tolerance.Resolve(BoundaryScale());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Polyline3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestBoundary(point).Parameter;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::StartTangent() const noexcept {
    return TangentAt(Scalar{0});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::EndTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()));
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::MidTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const std::size_t edge = EdgeIndex(t);
    return UnitEdge(m_points[edge], m_points[edge + 1]);
}

template <typename Scalar>
void Polyline3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    for (Linear::Point3T<Scalar>& point : m_points) {
        point = point + vector;
    }
}

template <typename Scalar>
void Polyline3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation =
        Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    for (Linear::Point3T<Scalar>& point : m_points) {
        point = rotation.TransformPoint(point);
    }
}

template <typename Scalar>
void Polyline3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror =
        Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    for (Linear::Point3T<Scalar>& vertex : m_points) {
        vertex = mirror.TransformPoint(vertex);
    }
}

template <typename Scalar>
void Polyline3T<Scalar>::Reverse() noexcept {
    if (IsClosed()) {
        std::reverse(m_points.begin() + 1, m_points.end() - 1);
        return;
    }
    std::reverse(m_points.begin(), m_points.end());
}

template <typename Scalar>
[[nodiscard]] Polyline3T<Scalar> Polyline3T<Scalar>::Clone() const {
    return *this;
}

template <typename Scalar>
[[nodiscard]] bool Polyline3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
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

template <typename Scalar>
[[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>> Polyline3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
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

template <typename Scalar>
Polyline3T<Scalar>::Polyline3T(std::vector<Linear::Point3T<Scalar>> points)
        : m_points(std::move(points)) {}

template <typename Scalar>
[[nodiscard]] Linear::Vector3T<Scalar> Polyline3T<Scalar>::Position(Linear::Point3T<Scalar> point) noexcept {
    return {point.X, point.Y, point.Z};
}

template <typename Scalar>
[[nodiscard]] Linear::Vector3T<Scalar> Polyline3T<Scalar>::VectorAreaSum() const noexcept {
    Linear::Vector3T<Scalar> sum{};
    for (std::size_t index = 0; index + 1 < m_points.size(); ++index) {
        sum = sum + Position(m_points[index]).Cross(Position(m_points[index + 1]));
    }
    return sum;
}

template <typename Scalar>
[[nodiscard]] Linear::Point3T<Scalar> Polyline3T<Scalar>::Locate(Scalar t) const noexcept {
    if (t == static_cast<Scalar>(SegmentCount())) {
        return m_points.back();
    }
    const auto index = static_cast<std::size_t>(t);
    const Scalar local = t - static_cast<Scalar>(index);
    return m_points[index] + (m_points[index + 1] - m_points[index]) * local;
}

template <typename Scalar>
[[nodiscard]] std::size_t Polyline3T<Scalar>::EdgeIndex(Scalar t) const noexcept {
    if (t == static_cast<Scalar>(SegmentCount())) {
        return SegmentCount() - 1;
    }
    return static_cast<std::size_t>(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::UnitEdge(Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to) noexcept {
    const Scalar length = from.DistanceTo(to);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return (to - from).Normalized();
}

template <typename Scalar>
[[nodiscard]] Scalar Polyline3T<Scalar>::BoundaryScale() const noexcept {
    const Scalar diagonal = Box().Extent().Length();
    if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
        return diagonal;
    }
    return Scalar{1};
}

template <typename Scalar>
[[nodiscard]] typename Polyline3T<Scalar>::BoundaryLocation Polyline3T<Scalar>::ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept {
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

template <typename Scalar>
[[nodiscard]] Scalar Polyline3T<Scalar>::BoundaryParameter(std::size_t edge,
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

template <typename Scalar>
[[nodiscard]] int Polyline3T<Scalar>::DroppedAxis(Linear::Vector3 normal) noexcept {
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

template <typename Scalar>
[[nodiscard]] Linear::Point2 Polyline3T<Scalar>::Project(Linear::Point3 point, int dropped) noexcept {
    if (dropped == 0) {
        return Linear::Point2{point.Y, point.Z};
    }
    if (dropped == 1) {
        return Linear::Point2{point.X, point.Z};
    }
    return Linear::Point2{point.X, point.Y};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Polyline3T<double>;
template struct DragonGeo::Prim::Polyline3T<float>;
template bool DragonGeo::Prim::Polyline3T<double>::Contains<double>(DragonGeo::Linear::Point3T<double>) const noexcept;
