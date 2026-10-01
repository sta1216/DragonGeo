#include <DragonGeo/Prim/Line2.hpp>

#include <DragonGeo/Prim/Ray2.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Line2T<Scalar>::Domain() const noexcept {
    return Linear::IntervalT<Scalar>::Unbounded();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Line2T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> Line2T<Scalar>::ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
    return Locate(ClosestParameter(point));
}

template <typename Scalar> [[nodiscard]] Scalar Line2T<Scalar>::DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar> [[nodiscard]] Scalar Line2T<Scalar>::Distance(Linear::Point2T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar> [[nodiscard]] Scalar Line2T<Scalar>::Length() const noexcept {
    return std::numeric_limits<Scalar>::infinity();
}

template <typename Scalar> [[nodiscard]] Linear::Box2T<Scalar> Line2T<Scalar>::Box() const noexcept {
    return Linear::Box2T<Scalar>::Empty();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Line2T<Scalar>::StartPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Line2T<Scalar>::EndPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Line2T<Scalar>::MidPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Line2T<Scalar>::StartTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Line2T<Scalar>::EndTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Line2T<Scalar>::MidTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Line2T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Direction;
}

template <typename Scalar> [[nodiscard]] bool Line2T<Scalar>::IsClosed() const noexcept {
    return false;
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Line2T<Scalar>::Area() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Line2T<Scalar>::Orientation() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Line2T<Scalar>::Centroid() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] bool Line2T<Scalar>::Contains(Linear::Point2T<Scalar>) const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] bool Line2T<Scalar>::ContainsPoint(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return Distance(point) <= tolerance.Resolve(Scalar{1});
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Line2T<Scalar>::ParameterOf(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestParameter(point);
}

template <typename Scalar> void Line2T<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    Origin = Origin + vector;
}

template <typename Scalar> void Line2T<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation = Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    const Linear::UnitVector2T<Scalar> direction = UnitDirection(rotation);
    Origin = rotation.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar> void Line2T<Scalar>::Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror = Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    const Linear::UnitVector2T<Scalar> direction = UnitDirection(mirror);
    Origin = mirror.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar> void Line2T<Scalar>::Reverse() noexcept {
    Direction = -Direction;
}

template <typename Scalar> [[nodiscard]] Line2T<Scalar> Line2T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Line2T<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    const Linear::Point2T<Scalar> moved = transform.TransformPoint(Origin);
    const Linear::Vector2T<Scalar> transformedDirection = transform * Direction.AsVector();
    if (!Detail::CoordinatesAreFinite(moved) || !Detail::CoordinatesAreFinite(transformedDirection)) {
        return false;
    }
    const auto unit = transformedDirection.Normalized();
    if (!unit.has_value()) {
        return false;
    }
    Origin = moved;
    Direction = *unit;
    return true;
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment2T<Scalar>> Line2T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    return Segment2T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment2T<Scalar>> Line2T<Scalar>::AsSegment(Scalar parameterStart, Scalar parameterEnd) const noexcept {
    if (!Core::IsFinite(parameterStart) || !Core::IsFinite(parameterEnd) || parameterStart == parameterEnd) {
        return std::nullopt;
    }
    const auto start = PointAt(parameterStart);
    const auto end = PointAt(parameterEnd);
    if (!start.has_value() || !end.has_value()) {
        return std::nullopt;
    }
    return Segment2T<Scalar>{*start, *end};
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> Line2T<Scalar>::Locate(Scalar t) const noexcept {
    return Origin + Direction * t;
}

template <typename Scalar> [[nodiscard]] Scalar Line2T<Scalar>::ClosestParameter(Linear::Point2T<Scalar> point) const noexcept {
    return Detail::ProjectParameter(Origin, Direction, point);
}

template <typename Scalar>
[[nodiscard]] Linear::UnitVector2T<Scalar> Line2T<Scalar>::UnitDirection(const Linear::Transform2T<Scalar>& transform) const noexcept {
    const Linear::Vector2T<Scalar> transformed = transform * Direction.AsVector();
    const auto unit = transformed.Normalized();
    if (unit.has_value()) {
        return *unit;
    }
    return Linear::UnitVector2T<Scalar>::FromNormalizedUnchecked(transformed);
}

template <typename Scalar> [[nodiscard]] Ray2T<Scalar> Line2T<Scalar>::AsRay() const noexcept {
    return {Origin, Direction};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Line2T<double>;
template struct DragonGeo::Prim::Line2T<float>;
