#include <DragonGeo/Prim/Line3.hpp>

#include <DragonGeo/Detail/CurveShape.hpp>
#include <DragonGeo/Prim/Ray3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Line3T<Scalar>::Domain() const noexcept {
    return Linear::IntervalT<Scalar>::Unbounded();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Line3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Line3T<Scalar>::ClosestPoint(Linear::Point3T<Scalar> point) const noexcept {
    return Locate(ClosestParameter(point));
}

template <typename Scalar> [[nodiscard]] Scalar Line3T<Scalar>::DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar> [[nodiscard]] Scalar Line3T<Scalar>::Distance(Linear::Point3T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar> [[nodiscard]] Scalar Line3T<Scalar>::Length() const noexcept {
    return std::numeric_limits<Scalar>::infinity();
}

template <typename Scalar> [[nodiscard]] Linear::Box3T<Scalar> Line3T<Scalar>::Box() const noexcept {
    return Linear::Box3T<Scalar>::Empty();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Line3T<Scalar>::StartPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Line3T<Scalar>::EndPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Line3T<Scalar>::MidPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Line3T<Scalar>::StartTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Line3T<Scalar>::EndTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Line3T<Scalar>::MidTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Line3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Direction;
}

template <typename Scalar> [[nodiscard]] bool Line3T<Scalar>::IsClosed() const noexcept {
    return false;
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Line3T<Scalar>::Area() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Line3T<Scalar>::Orientation() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Line3T<Scalar>::Centroid() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] bool Line3T<Scalar>::Contains(Linear::Point3T<Scalar>) const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] bool Line3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return Distance(point) <= tolerance.Resolve(Scalar{1});
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Line3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestParameter(point);
}

template <typename Scalar> void Line3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    Origin = Origin + vector;
}

template <typename Scalar> void Line3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin, Linear::UnitVector3T<Scalar> axis, Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation = Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    const Linear::UnitVector3T<Scalar> direction = Detail::RenormalizedDirection(Direction, rotation);
    Origin = rotation.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar> void Line3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror = Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    const Linear::UnitVector3T<Scalar> direction = Detail::RenormalizedDirection(Direction, mirror);
    Origin = mirror.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar> void Line3T<Scalar>::Reverse() noexcept {
    Direction = -Direction;
}

template <typename Scalar> [[nodiscard]] Line3T<Scalar> Line3T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Line3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
    return Detail::TryTransformFrame(Origin, Direction, transform);
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment3T<Scalar>> Line3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    return Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment3T<Scalar>> Line3T<Scalar>::AsSegment(Scalar parameterStart, Scalar parameterEnd) const noexcept {
    if (!Core::IsFinite(parameterStart) || !Core::IsFinite(parameterEnd) || parameterStart == parameterEnd) {
        return std::nullopt;
    }
    const auto start = PointAt(parameterStart);
    const auto end = PointAt(parameterEnd);
    if (!start.has_value() || !end.has_value()) {
        return std::nullopt;
    }
    return Segment3T<Scalar>{*start, *end};
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Line3T<Scalar>::Locate(Scalar t) const noexcept {
    return Origin + Direction * t;
}

template <typename Scalar> [[nodiscard]] Scalar Line3T<Scalar>::ClosestParameter(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::ProjectParameter(Origin, Direction, point);
}

template <typename Scalar>
[[nodiscard]] Linear::UnitVector3T<Scalar> Line3T<Scalar>::UnitDirection(const Linear::Transform3T<Scalar>& transform) const noexcept {
    return Detail::RenormalizedDirection(Direction, transform);
}

template <typename Scalar> [[nodiscard]] Ray3T<Scalar> Line3T<Scalar>::AsRay() const noexcept {
    return {Origin, Direction};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Line3T<double>;
template struct DragonGeo::Prim::Line3T<float>;
