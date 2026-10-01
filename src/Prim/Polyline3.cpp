#include <DragonGeo/Prim/Polyline3.hpp>

#include <DragonGeo/Detail/CurveContainment.hpp>
#include <DragonGeo/Detail/CurveShape.hpp>

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
    return std::optional<Polyline3T>(Polyline3T{std::vector<Linear::Point3T<Scalar>>(points.begin(), points.end())});
}

template <typename Scalar> [[nodiscard]] std::size_t Polyline3T<Scalar>::PointCount() const noexcept {
    return m_points.size();
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Polyline3T<Scalar>::Point(std::size_t index) const {
    return m_points[index];
}

template <typename Scalar> [[nodiscard]] std::size_t Polyline3T<Scalar>::SegmentCount() const noexcept {
    return m_points.size() - 1;
}

template <typename Scalar> [[nodiscard]] Segment3T<Scalar> Polyline3T<Scalar>::Segment(std::size_t index) const {
    return Segment3T<Scalar>{Point(index), Point(index + 1)};
}

template <typename Scalar> [[nodiscard]] bool Polyline3T<Scalar>::IsClosed() const noexcept {
    return PointCount() >= 2 && Point(0) == Point(PointCount() - 1);
}

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Polyline3T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, static_cast<Scalar>(SegmentCount())};
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::StartPoint() const noexcept {
    return Point(0);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::EndPoint() const noexcept {
    return Point(PointCount() - 1);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::MidPoint() const noexcept {
    return PointAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar> [[nodiscard]] Scalar Polyline3T<Scalar>::Length() const noexcept {
    return Detail::ChainLength(m_points.data(), SegmentCount());
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Polyline3T<Scalar>::Area() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    return (VectorAreaSum() * Scalar{0.5}).Length();
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Polyline3T<Scalar>::Orientation() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    const Linear::Vector3T<Scalar> sum = VectorAreaSum();
    if (sum.LengthSquared() == Scalar{0}) {
        return Winding::Degenerate;
    }
    const Scalar component = Detail::DominantComponent(sum.X, sum.Y, sum.Z);
    if (component > Scalar{0}) {
        return Winding::CounterClockwise;
    }
    return Winding::Clockwise;
}

template <typename Scalar> [[nodiscard]] Linear::Box3T<Scalar> Polyline3T<Scalar>::Box() const noexcept {
    return Linear::Box3T<Scalar>::FromPoints(m_points);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Polyline3T<Scalar>::Centroid() const noexcept {
    if (!IsClosed() || VectorAreaSum().LengthSquared() == Scalar{0}) {
        return std::nullopt;
    }
    return Detail::AverageOf(m_points.data(), m_points.size() - 1);
}

template <typename Scalar> template <typename S> requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Polyline3T<Scalar>::Contains(Linear::Point3T<S> point) const noexcept {
    if (!IsClosed()) {
        return false;
    }

    const auto onBoundary = [&]() noexcept {
        for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
            if (Detail::PointOnSegment3(m_points[edge], m_points[edge + 1], point)) {
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
    while (along < uniqueCount && m_points[along] == m_points[origin]) {++along;
    }
    if (along < uniqueCount) {
        for (std::size_t off = along + 1; off < uniqueCount; ++off) {
            if (!Detail::PointsAreCollinear3(m_points[origin], m_points[along], m_points[off])) {
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
    const auto projected = [&](std::size_t index) noexcept { return Project(m_points[index], dropped); };
    return Detail::WindingNumber(SegmentCount(), query, projected) != 0;
}

template <typename Scalar>
[[nodiscard]] bool Polyline3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return Detail::DistanceWithin(ClosestBoundary(point).DistanceSquared, tolerance.Resolve(BoundaryScale()));
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Polyline3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    if (!Detail::DistanceWithin(boundary.DistanceSquared, tolerance.Resolve(BoundaryScale()))) {
        return std::nullopt;
    }
    return boundary.Parameter;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::StartTangent() const noexcept {
    return TangentAt(Scalar{0});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::EndTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()));
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::MidTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const std::size_t edge = EdgeIndex(t);
    return UnitEdge(m_points[edge], m_points[edge + 1]);
}

template <typename Scalar> void Polyline3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    Detail::TranslatePoints(m_points.data(), m_points.size(), vector);
}

template <typename Scalar>
void Polyline3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin, Linear::UnitVector3T<Scalar> axis, Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation = Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    Detail::ApplyTransform(m_points.data(), m_points.size(), rotation);
}

template <typename Scalar> void Polyline3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror = Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    Detail::ApplyTransform(m_points.data(), m_points.size(), mirror);
}

template <typename Scalar> void Polyline3T<Scalar>::Reverse() noexcept {
    Detail::ReverseChain(m_points.data(), m_points.size(), IsClosed());
}

template <typename Scalar> [[nodiscard]] Polyline3T<Scalar> Polyline3T<Scalar>::Clone() const {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Polyline3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
    return Detail::TryTransformPoints(m_points.data(), m_points.size(), transform);
}

template <typename Scalar>
[[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>>
Polyline3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
    const auto locate = [this](Scalar parameter) noexcept { return Locate(parameter); };
    return Detail::ChainSubcurve<Segment3T<Scalar>, Polyline3T<Scalar>>(SegmentCount(), interval, Domain(), m_points.data(), locate);
}

template <typename Scalar> Polyline3T<Scalar>::Polyline3T(std::vector<Linear::Point3T<Scalar>> points): m_points(std::move(points)) {}

template <typename Scalar> [[nodiscard]] Linear::Vector3T<Scalar> Polyline3T<Scalar>::Position(Linear::Point3T<Scalar> point) noexcept {
    return {point.X, point.Y, point.Z};
}

template <typename Scalar> [[nodiscard]] Linear::Vector3T<Scalar> Polyline3T<Scalar>::VectorAreaSum() const noexcept {
    Linear::Vector3T<Scalar> sum{};
    for (std::size_t index = 0; index + 1 < m_points.size(); ++index) {
        sum = sum + Position(m_points[index]).Cross(Position(m_points[index + 1]));
    }
    return sum;
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Polyline3T<Scalar>::Locate(Scalar t) const noexcept {
    return Detail::LocateOnChain(m_points.data(), SegmentCount(), t);
}

template <typename Scalar> [[nodiscard]] std::size_t Polyline3T<Scalar>::EdgeIndex(Scalar t) const noexcept {
    return Detail::EdgeIndexOnChain(SegmentCount(), t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Polyline3T<Scalar>::UnitEdge(Linear::Point3T<Scalar> from,
    Linear::Point3T<Scalar> to) noexcept {
    return Detail::UnitDirection(from, to);
}

template <typename Scalar> [[nodiscard]] Scalar Polyline3T<Scalar>::BoundaryScale() const noexcept {
    return Detail::DiagonalScale(Box());
}

template <typename Scalar>
[[nodiscard]] typename Polyline3T<Scalar>::BoundaryLocation Polyline3T<Scalar>::ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::ClosestOnChain<BoundaryLocation>(m_points.data(), m_points.size(), point, IsClosed());
}

template <typename Scalar> [[nodiscard]] Scalar Polyline3T<Scalar>::BoundaryParameter(std::size_t edge,
        Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to, Linear::Point3T<Scalar> closest) const noexcept {
    Scalar parameter = Detail::EdgeParameter(edge, from, to, closest);
    if (IsClosed() && parameter == static_cast<Scalar>(SegmentCount())) {
        parameter = Scalar{0};
    }
    return parameter;
}

template <typename Scalar> [[nodiscard]] int Polyline3T<Scalar>::DroppedAxis(Linear::Vector3 normal) noexcept {
    return Detail::DominantAxis(normal);
}

template <typename Scalar> [[nodiscard]] Linear::Point2 Polyline3T<Scalar>::Project(Linear::Point3 point, int dropped) noexcept {
    return Detail::DropAxis(point, dropped);
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Polyline3T<double>;
template struct DragonGeo::Prim::Polyline3T<float>;
template bool DragonGeo::Prim::Polyline3T<double>::Contains<double>(DragonGeo::Linear::Point3T<double>) const noexcept;
