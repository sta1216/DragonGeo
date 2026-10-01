#include <DragonGeo/Prim/Segment3.hpp>

#include <DragonGeo/Detail/CurveShape.hpp>
#include <DragonGeo/Prim/Ray3.hpp>
#include <DragonGeo/Prim/Line3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Segment3T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, Scalar{1}};
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Segment3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Segment3T<Scalar>::ClosestPoint(Linear::Point3T<Scalar> point) const noexcept {
    return Locate(ClosestParameter(point));
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::Distance(Linear::Point3T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::Length() const noexcept {
    return A.DistanceTo(B);
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::LengthSquared() const noexcept {
    return (B - A).LengthSquared();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Segment3T<Scalar>::Direction() const noexcept {
    if (A == B) {
        return std::nullopt;
    }
    return (B - A).Normalized();
}

template <typename Scalar> [[nodiscard]] Linear::Box3T<Scalar> Segment3T<Scalar>::Box() const noexcept {
    return Linear::Box3T<Scalar>::FromCorners(A, B);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Segment3T<Scalar>::StartPoint() const noexcept {
    return A;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Segment3T<Scalar>::EndPoint() const noexcept {
    return B;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Segment3T<Scalar>::MidPoint() const noexcept {
    return PointAt(Scalar{0.5});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Segment3T<Scalar>::StartTangent() const noexcept {
    return Direction();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Segment3T<Scalar>::EndTangent() const noexcept {
    return Direction();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Segment3T<Scalar>::MidTangent() const noexcept {
    return Direction();
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Segment3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Direction();
}

template <typename Scalar> [[nodiscard]] bool Segment3T<Scalar>::IsClosed() const noexcept {
    return false;
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Segment3T<Scalar>::Area() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Segment3T<Scalar>::Orientation() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Segment3T<Scalar>::Centroid() const noexcept {
    return std::nullopt;
}

template <typename Scalar> [[nodiscard]] bool Segment3T<Scalar>::Contains(Linear::Point3T<Scalar>) const noexcept {
    return false;
}

template <typename Scalar>
[[nodiscard]] bool Segment3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    const Scalar expansion = tolerance.Resolve(Scalar{1});
    const Scalar parameter = SupportingParameter(point);
    if (!(parameter >= -expansion && parameter <= Scalar{1} + expansion)) {
        return false;
    }
    return DistanceToSupportingLine(point) <= tolerance.Resolve(Length());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Segment3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return SupportingParameter(point);
}

template <typename Scalar> void Segment3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    A = A + vector;
    B = B + vector;
}

template <typename Scalar>
void Segment3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin, Linear::UnitVector3T<Scalar> axis, Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation = Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    A = rotation.TransformPoint(A);
    B = rotation.TransformPoint(B);
}

template <typename Scalar> void Segment3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror = Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    A = mirror.TransformPoint(A);
    B = mirror.TransformPoint(B);
}

template <typename Scalar> void Segment3T<Scalar>::Reverse() noexcept {
    const Linear::Point3T<Scalar> start = A;
    A = B;
    B = start;
}

template <typename Scalar> [[nodiscard]] Segment3T<Scalar> Segment3T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Segment3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
    return Detail::TryTransformChord(A, B, transform);
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment3T<Scalar>> Segment3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    return Segment3T{Locate(interval.Min), Locate(interval.Max)};
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Segment3T<Scalar>::Locate(Scalar t) const noexcept {
    return A + (B - A) * t;
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::SupportingParameter(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::ChordParameter(A, B, point);
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::DistanceToSupportingLine(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::DistanceToChordLine(A, B, point);
}

template <typename Scalar> [[nodiscard]] Scalar Segment3T<Scalar>::ClosestParameter(Linear::Point3T<Scalar> point) const noexcept {
    return Detail::ClampParameter(Detail::ChordParameter(A, B, point), Domain());
}

template <typename Scalar> [[nodiscard]] std::optional<Ray3T<Scalar>> Segment3T<Scalar>::AsRay() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Ray3T<Scalar>{A, *unitDirection};
}

template <typename Scalar> [[nodiscard]] std::optional<Line3T<Scalar>> Segment3T<Scalar>::AsLine() const noexcept {
    const auto unitDirection = Direction();
    if (!unitDirection.has_value()) {
        return std::nullopt;
    }
    return Line3T<Scalar>{A, *unitDirection};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Segment3T<double>;
template struct DragonGeo::Prim::Segment3T<float>;
