#include <DragonGeo/Prim/Segment2.hpp>

#include <DragonGeo/Prim/Ray2.hpp>
#include <DragonGeo/Prim/Line2.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] Linear::IntervalT<Scalar> Segment2T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, Scalar{1}};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Segment2T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> Segment2T<Scalar>::ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
    return Locate(ClosestParameter(point));
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::Distance(Linear::Point2T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::Length() const noexcept {
    return A.DistanceTo(B);
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::LengthSquared() const noexcept {
    return (B - A).LengthSquared();
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Segment2T<Scalar>::Direction() const noexcept {
    if (A == B) {
        return std::nullopt;
    }
    return (B - A).Normalized();
}

template <typename Scalar>
[[nodiscard]] Linear::Box2T<Scalar> Segment2T<Scalar>::Box() const noexcept {
    return Linear::Box2T<Scalar>::FromCorners(A, B);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Segment2T<Scalar>::StartPoint() const noexcept {
    return A;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Segment2T<Scalar>::EndPoint() const noexcept {
    return B;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Segment2T<Scalar>::MidPoint() const noexcept {
    return PointAt(Scalar{0.5});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Segment2T<Scalar>::StartTangent() const noexcept {
    return Direction();
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Segment2T<Scalar>::EndTangent() const noexcept {
    return Direction();
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Segment2T<Scalar>::MidTangent() const noexcept {
    return Direction();
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Segment2T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Direction();
}

template <typename Scalar>
[[nodiscard]] bool Segment2T<Scalar>::IsClosed() const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Segment2T<Scalar>::Area() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Winding> Segment2T<Scalar>::Orientation() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Segment2T<Scalar>::Centroid() const noexcept {
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] bool Segment2T<Scalar>::Contains(Linear::Point2T<Scalar>) const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] bool Segment2T<Scalar>::ContainsPoint(Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    const Scalar expansion = tolerance.Resolve(Scalar{1});
    const Scalar parameter = SupportingParameter(point);
    if (!(parameter >= -expansion && parameter <= Scalar{1} + expansion)) {
        return false;
    }
    return DistanceToSupportingLine(point) <= tolerance.Resolve(Length());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Segment2T<Scalar>::ParameterOf(Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return SupportingParameter(point);
}

template <typename Scalar>
void Segment2T<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    A = A + vector;
    B = B + vector;
}

template <typename Scalar>
void Segment2T<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation =
        Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    A = rotation.TransformPoint(A);
    B = rotation.TransformPoint(B);
}

template <typename Scalar>
void Segment2T<Scalar>::Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror =
        Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    A = mirror.TransformPoint(A);
    B = mirror.TransformPoint(B);
}

template <typename Scalar>
void Segment2T<Scalar>::Reverse() noexcept {
    const Linear::Point2T<Scalar> start = A;
    A = B;
    B = start;
}

template <typename Scalar>
[[nodiscard]] Segment2T<Scalar> Segment2T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar>
[[nodiscard]] bool Segment2T<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    const Linear::Point2T<Scalar> movedA = transform.TransformPoint(A);
    const Linear::Point2T<Scalar> movedB = transform.TransformPoint(B);
    if (!Detail::CoordinatesAreFinite(movedA) || !Detail::CoordinatesAreFinite(movedB)) {
        return false;
    }
    const Linear::Vector2T<Scalar> chord = B - A;
    const Linear::Vector2T<Scalar> transformedChord = transform * chord;
    if (chord.Normalized().has_value()
        && (!Detail::CoordinatesAreFinite(transformedChord)
            || !transformedChord.Normalized().has_value())) {
        return false;
    }
    A = movedA;
    B = movedB;
    return true;
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment2T<Scalar>> Segment2T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    return Segment2T{Locate(interval.Min), Locate(interval.Max)};
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> Segment2T<Scalar>::Locate(Scalar t) const noexcept {
    return A + (B - A) * t;
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::SupportingParameter(Linear::Point2T<Scalar> point) const noexcept {
    const auto direction = Direction();
    if (!direction.has_value()) {
        return Scalar{0};
    }
    const Scalar length = Length();
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return Scalar{0};
    }
    return Detail::ProjectParameter(A, *direction, point) / length;
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::DistanceToSupportingLine(Linear::Point2T<Scalar> point) const noexcept {
    const auto direction = Direction();
    const Scalar length = Length();
    if (!direction.has_value() || !(length > Scalar{0}) || !Core::IsFinite(length)) {
        return point.DistanceTo(A);
    }
    return point.DistanceTo(Locate(SupportingParameter(point)));
}

template <typename Scalar>
[[nodiscard]] Scalar Segment2T<Scalar>::ClosestParameter(Linear::Point2T<Scalar> point) const noexcept {
    const auto direction = Direction();
    if (!direction.has_value()) {
        return Scalar{0};
    }
    const Scalar length = Length();
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return Scalar{0};
    }
    return Detail::ClampParameter(
        Detail::ProjectParameter(A, *direction, point) / length, Domain());
}

template <typename Scalar>
[[nodiscard]] std::optional<Ray2T<Scalar>> Segment2T<Scalar>::AsRay() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Ray2T<Scalar>{A, *unitDirection};
}

template <typename Scalar>
[[nodiscard]] std::optional<Line2T<Scalar>> Segment2T<Scalar>::AsLine() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Line2T<Scalar>{A, *unitDirection};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Segment2T<double>;
template struct DragonGeo::Prim::Segment2T<float>;
