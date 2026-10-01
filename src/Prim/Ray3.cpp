#include <DragonGeo/Prim/Ray3.hpp>

#include <DragonGeo/Prim/Line3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] Linear::IntervalT<Scalar> Ray3T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, std::numeric_limits<Scalar>::infinity()};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Ray3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar>
[[nodiscard]] Linear::Point3T<Scalar> Ray3T<Scalar>::ClosestPoint(Linear::Point3T<Scalar> point) const noexcept {
    return Locate(ClosestParameter(point));
}

template <typename Scalar>
[[nodiscard]] Scalar Ray3T<Scalar>::DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar>
[[nodiscard]] Scalar Ray3T<Scalar>::Distance(Linear::Point3T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar>
[[nodiscard]] Scalar Ray3T<Scalar>::Length() const noexcept {
    return std::numeric_limits<Scalar>::infinity();
}

template <typename Scalar>
[[nodiscard]] Linear::Box3T<Scalar> Ray3T<Scalar>::Box() const noexcept {
    return Linear::Box3T<Scalar>::Empty();
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Ray3T<Scalar>::StartPoint() const noexcept {
    return Origin;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Ray3T<Scalar>::EndPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Ray3T<Scalar>::MidPoint() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Ray3T<Scalar>::StartTangent() const noexcept {
    return Direction;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Ray3T<Scalar>::EndTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Ray3T<Scalar>::MidTangent() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Ray3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Direction;
}

template <typename Scalar>
[[nodiscard]] bool Ray3T<Scalar>::IsClosed() const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Ray3T<Scalar>::Area() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Winding> Ray3T<Scalar>::Orientation() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Ray3T<Scalar>::Centroid() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] bool Ray3T<Scalar>::Contains(Linear::Point3T<Scalar>) const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] bool Ray3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    return Distance(point) <= tolerance.Resolve(Scalar{1});
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Ray3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestParameter(point);
}

template <typename Scalar>
void Ray3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    Origin = Origin + vector;
}

template <typename Scalar>
void Ray3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin,
        Linear::UnitVector3T<Scalar> axis,
        Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation =
        Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    const Linear::UnitVector3T<Scalar> direction = UnitDirection(rotation);
    Origin = rotation.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar>
void Ray3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror =
        Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    const Linear::UnitVector3T<Scalar> direction = UnitDirection(mirror);
    Origin = mirror.TransformPoint(Origin);
    Direction = direction;
}

template <typename Scalar>
void Ray3T<Scalar>::Reverse() noexcept {
    Direction = -Direction;
}

template <typename Scalar>
[[nodiscard]] Ray3T<Scalar> Ray3T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar>
[[nodiscard]] bool Ray3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
    const Linear::Point3T<Scalar> moved = transform.TransformPoint(Origin);
    const Linear::Vector3T<Scalar> transformedDirection = transform * Direction.AsVector();
    if (!Detail::CoordinatesAreFinite(moved)
        || !Detail::CoordinatesAreFinite(transformedDirection)) {
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
[[nodiscard]] std::optional<Segment3T<Scalar>> Ray3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    return Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment3T<Scalar>> Ray3T<Scalar>::AsSegment(Scalar length) const noexcept {
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return Segment3T<Scalar>{Origin, Locate(length)};
}

template <typename Scalar>
[[nodiscard]] Linear::Point3T<Scalar> Ray3T<Scalar>::Locate(Scalar t) const noexcept {
    return Origin + Direction * t;
}

template <typename Scalar>
[[nodiscard]] Scalar Ray3T<Scalar>::ClosestParameter(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::ClampParameter(
        Detail::ProjectParameter(Origin, Direction, point), Domain());
}

template <typename Scalar>
[[nodiscard]] Linear::UnitVector3T<Scalar> Ray3T<Scalar>::UnitDirection(const Linear::Transform3T<Scalar>& transform) const noexcept {
    const Linear::Vector3T<Scalar> transformed = transform * Direction.AsVector();
    const auto unit = transformed.Normalized();
    if (unit.has_value()) {
        return *unit;
    }
    return Linear::UnitVector3T<Scalar>::FromNormalizedUnchecked(transformed);
}

template <typename Scalar>
[[nodiscard]] Line3T<Scalar> Ray3T<Scalar>::AsLine() const noexcept {
    return {Origin, Direction};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Ray3T<double>;
template struct DragonGeo::Prim::Ray3T<float>;
