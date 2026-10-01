#include <DragonGeo/Prim/Plane.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Scalar PlaneT<Scalar>::SignedDistance(Linear::Point3T<Scalar> point) const noexcept {
    return (point - Origin).Dot(Normal.AsVector());
}

template <typename Scalar> [[nodiscard]] Scalar PlaneT<Scalar>::Distance(Linear::Point3T<Scalar> point) const noexcept {
    return Core::AbsoluteValue(SignedDistance(point));
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> PlaneT<Scalar>::ClosestPoint(Linear::Point3T<Scalar> point) const noexcept {
    return point - Normal * SignedDistance(point);
}

template <typename Scalar> [[nodiscard]] PlaneT<Scalar> PlaneT<Scalar>::Flipped() const noexcept {
    return PlaneT{Origin, -Normal};
}

template <typename Scalar> [[nodiscard]] Linear::Vector3T<Scalar> PlaneT<Scalar>::Project(Linear::Vector3T<Scalar> vector) const noexcept {
    return vector - Normal * Normal.AsVector().Dot(vector);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> PlaneT<Scalar>::Project(Linear::UnitVector3T<Scalar> direction) const noexcept {
    return Project(direction.AsVector()).Normalized();
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> PlaneT<Scalar>::Mirror(Linear::Point3T<Scalar> point) const noexcept {
    return point - Normal * (Scalar{2} * SignedDistance(point));
}

template <typename Scalar> [[nodiscard]] Linear::Vector3T<Scalar> PlaneT<Scalar>::Mirror(Linear::Vector3T<Scalar> vector) const noexcept {
    return vector - Normal * (Scalar{2} * Normal.AsVector().Dot(vector));
}

template <typename Scalar>
[[nodiscard]] bool PlaneT<Scalar>::Contains(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    return Core::AbsoluteValue(SignedDistance(point)) <= tolerance.Resolve(Scalar{1});
}

template <typename Scalar> void PlaneT<Scalar>::Offset(Scalar distance) noexcept {
    Origin = Origin + Normal * distance;
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::PlaneT<double>;
template struct DragonGeo::Prim::PlaneT<float>;
