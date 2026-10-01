#include <DragonGeo/Prim/Polyline.hpp>

#include <DragonGeo/Detail/CurveContainment.hpp>
#include <DragonGeo/Detail/CurveShape.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] std::optional<PolylineT<Scalar>> PolylineT<Scalar>::FromPoints(std::span<const Linear::Point2T<Scalar>> points) {
    if (points.size() < 2) {
        return std::nullopt;
    }
    for (const Linear::Point2T<Scalar>& point : points) {
        if (!Detail::CoordinatesAreFinite(point)) {
            return std::nullopt;
        }
    }
    return std::optional<PolylineT>(PolylineT{std::vector<Linear::Point2T<Scalar>>(points.begin(), points.end())});
}

template <typename Scalar> [[nodiscard]] std::size_t PolylineT<Scalar>::PointCount() const noexcept {
    return m_points.size();
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> PolylineT<Scalar>::Point(std::size_t index) const {
    return m_points[index];
}

template <typename Scalar> [[nodiscard]] std::size_t PolylineT<Scalar>::SegmentCount() const noexcept {
    return m_points.size() - 1;
}

template <typename Scalar> [[nodiscard]] Segment2T<Scalar> PolylineT<Scalar>::Segment(std::size_t index) const {
    return Segment2T<Scalar>{Point(index), Point(index + 1)};
}

template <typename Scalar> [[nodiscard]] bool PolylineT<Scalar>::IsClosed() const noexcept {
    return PointCount() >= 2 && Point(0) == Point(PointCount() - 1);
}

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> PolylineT<Scalar>::Domain() const noexcept {
    return {Scalar{0}, static_cast<Scalar>(SegmentCount())};
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::StartPoint() const noexcept {
    return Point(0);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::EndPoint() const noexcept {
    return Point(PointCount() - 1);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::MidPoint() const noexcept {
    return PointAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar> [[nodiscard]] Scalar PolylineT<Scalar>::Length() const noexcept {
    return Detail::ChainLength(m_points.data(), SegmentCount());
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> PolylineT<Scalar>::Area() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    return Core::AbsoluteValue(SignedAreaSum()) * Scalar{0.5};
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> PolylineT<Scalar>::Orientation() const noexcept {
    if (!IsClosed()) {
        return std::nullopt;
    }
    return Detail::WindingFromSign(SignedAreaSum());
}

template <typename Scalar> [[nodiscard]] Linear::Box2T<Scalar> PolylineT<Scalar>::Box() const noexcept {
    return Linear::Box2T<Scalar>::FromPoints(m_points);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> PolylineT<Scalar>::Centroid() const noexcept {
    if (!IsClosed() || SignedAreaSum() == Scalar{0}) {
        return std::nullopt;
    }
    return Detail::AverageOf(m_points.data(), m_points.size() - 1);
}

template <typename Scalar> template <typename S> requires std::same_as<Scalar, double> && std::same_as<S, double>
[[nodiscard]] bool PolylineT<Scalar>::Contains(Linear::Point2T<S> point) const noexcept {
    if (!IsClosed()) {
        return false;
    }
    for (std::size_t edge = 0; edge < SegmentCount(); ++edge) {
        if (Detail::PointOnSegment2(m_points[edge], m_points[edge + 1], point)) {
            return true;
        }
    }
    const auto vertex = [this](std::size_t index) noexcept { return m_points[index]; };
    return Detail::WindingNumber(SegmentCount(), point, vertex) != 0;
}

template <typename Scalar>
[[nodiscard]] bool PolylineT<Scalar>::ContainsPoint(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return ClosestBoundary(point).DistanceSquared <= tolerance.Resolve(BoundaryScale()) * tolerance.Resolve(BoundaryScale());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> PolylineT<Scalar>::ParameterOf(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    const Scalar allowance = tolerance.Resolve(BoundaryScale());
    if (!(boundary.DistanceSquared <= allowance * allowance)) {
        return std::nullopt;
    }
    return boundary.Parameter;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::StartTangent() const noexcept {
    return TangentAt(Scalar{0});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::EndTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()));
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::MidTangent() const noexcept {
    return TangentAt(static_cast<Scalar>(SegmentCount()) * Scalar{0.5});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const std::size_t edge = EdgeIndex(t);
    return UnitEdge(m_points[edge], m_points[edge + 1]);
}

template <typename Scalar> void PolylineT<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    Detail::TranslatePoints(m_points.data(), m_points.size(), vector);
}

template <typename Scalar> void PolylineT<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation = Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    Detail::ApplyTransform(m_points.data(), m_points.size(), rotation);
}

template <typename Scalar> void PolylineT<Scalar>::Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror = Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    Detail::ApplyTransform(m_points.data(), m_points.size(), mirror);
}

template <typename Scalar> void PolylineT<Scalar>::Reverse() noexcept {
    Detail::ReverseChain(m_points.data(), m_points.size(), IsClosed());
}

template <typename Scalar> [[nodiscard]] PolylineT<Scalar> PolylineT<Scalar>::Clone() const {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool PolylineT<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    return Detail::TryTransformPoints(m_points.data(), m_points.size(), transform);
}

template <typename Scalar> [[nodiscard]] std::optional<std::variant<Segment2T<Scalar>, PolylineT<Scalar>>>
PolylineT<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
    const auto locate = [this](Scalar parameter) noexcept { return Locate(parameter); };
    return Detail::ChainSubcurve<Segment2T<Scalar>, PolylineT<Scalar>>(SegmentCount(), interval, Domain(), m_points.data(), locate);
}

template <typename Scalar> PolylineT<Scalar>::PolylineT(std::vector<Linear::Point2T<Scalar>> points): m_points(std::move(points)) {}

template <typename Scalar> [[nodiscard]] Scalar PolylineT<Scalar>::SignedAreaSum() const noexcept {
    Scalar sum{0};
    for (std::size_t index = 0; index + 1 < m_points.size(); ++index) {
        const Linear::Vector2T<Scalar> current{m_points[index].X, m_points[index].Y};
        const Linear::Vector2T<Scalar> next{m_points[index + 1].X, m_points[index + 1].Y};
        sum += current.Cross(next);
    }
    return sum;
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> PolylineT<Scalar>::Locate(Scalar t) const noexcept {
    return Detail::LocateOnChain(m_points.data(), SegmentCount(), t);
}

template <typename Scalar> [[nodiscard]] std::size_t PolylineT<Scalar>::EdgeIndex(Scalar t) const noexcept {
    return Detail::EdgeIndexOnChain(SegmentCount(), t);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> PolylineT<Scalar>::UnitEdge(
    Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept {
    return Detail::UnitDirection(from, to);
}

template <typename Scalar> [[nodiscard]] Scalar PolylineT<Scalar>::BoundaryScale() const noexcept {
    return Detail::DiagonalScale(Box());
}

template <typename Scalar>
[[nodiscard]] typename PolylineT<Scalar>::BoundaryLocation PolylineT<Scalar>::ClosestBoundary(Linear::Point2T<Scalar> point) const noexcept {
    return Detail::ClosestOnChain<BoundaryLocation>(m_points.data(), m_points.size(), point, IsClosed());
}

template <typename Scalar> [[nodiscard]] Scalar PolylineT<Scalar>::BoundaryParameter(
    std::size_t edge, Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to, Linear::Point2T<Scalar> closest) const noexcept {
    Scalar parameter = Detail::EdgeParameter(edge, from, to, closest);
    if (IsClosed() && parameter == static_cast<Scalar>(SegmentCount())) {
        parameter = Scalar{0};
    }
    return parameter;
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::PolylineT<double>;
template struct DragonGeo::Prim::PolylineT<float>;
template bool DragonGeo::Prim::PolylineT<double>::Contains<double>(DragonGeo::Linear::Point2T<double>) const noexcept;
