#include <DragonGeo/Prim/Triangle2.hpp>

#include <DragonGeo/Detail/CurveContainment.hpp>
#include <DragonGeo/Detail/CurveShape.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::SignedArea() const noexcept {
    return Scalar{0.5} * (B - A).Cross(C - A);
}

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Triangle2T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, Scalar{3}};
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::MidPoint() const noexcept {
    return PointAt(Scalar{1.5});
}

template <typename Scalar> template <typename S> requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Triangle2T<Scalar>::Contains(Linear::Point2T<S> point) const noexcept {
    if (Predicates::Orient2d(A, B, C) == 0) {
        return Detail::PointOnSegment2(A, B, point) || Detail::PointOnSegment2(B, C, point) || Detail::PointOnSegment2(C, A, point);
    }
    const int ab = Predicates::Orient2d(A, B, point);
    const int bc = Predicates::Orient2d(B, C, point);
    const int ca = Predicates::Orient2d(C, A, point);
    return (ab >= 0 && bc >= 0 && ca >= 0) || (ab <= 0 && bc <= 0 && ca <= 0);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::StartPoint() const noexcept {
    return A;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::EndPoint() const noexcept {
    return A;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::StartTangent() const noexcept {
    return UnitEdge(A, B);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::EndTangent() const noexcept {
    return UnitEdge(C, A);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::MidTangent() const noexcept {
    return UnitEdge(B, C);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
    const int edge = EdgeIndex(t);
    return UnitEdge(vertices[edge], vertices[edge + 1]);
}

template <typename Scalar> [[nodiscard]] bool Triangle2T<Scalar>::IsClosed() const noexcept {
    return true;
}

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::Length() const noexcept {
    const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
    return Detail::ChainLength(vertices, 3);
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Triangle2T<Scalar>::Area() const noexcept {
    return Core::AbsoluteValue(SignedArea());
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Triangle2T<Scalar>::Orientation() const noexcept {
    return Detail::WindingFromSign(SignedArea());
}

template <typename Scalar> [[nodiscard]] Linear::Box2T<Scalar> Triangle2T<Scalar>::Box() const noexcept {
    const Linear::Point2T<Scalar> vertices[]{A, B, C};
    return Linear::Box2T<Scalar>::FromPoints(std::span<const Linear::Point2T<Scalar>>{vertices});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::Centroid() const noexcept {
    if (SignedArea() == Scalar{0}) {
        return std::nullopt;
    }
    return Linear::Point2T<Scalar>{(A.X + B.X + C.X) / Scalar{3}, (A.Y + B.Y + C.Y) / Scalar{3}};
}

template <typename Scalar>
[[nodiscard]] bool Triangle2T<Scalar>::ContainsPoint(Linear::Point2T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    return Detail::DistanceWithin(boundary.DistanceSquared, tolerance.Resolve(BoundaryScale()));
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Triangle2T<Scalar>::ParameterOf(Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    if (!Detail::DistanceWithin(boundary.DistanceSquared, tolerance.Resolve(BoundaryScale()))) {
        return std::nullopt;
    }
    return boundary.Parameter;
}

template <typename Scalar> void Triangle2T<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    A = A + vector;
    B = B + vector;
    C = C + vector;
}

template <typename Scalar> void Triangle2T<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation = Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    Detail::TransformVertices(A, B, C, rotation);
}

template <typename Scalar> void Triangle2T<Scalar>::Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror = Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    Detail::TransformVertices(A, B, C, mirror);
}

template <typename Scalar> void Triangle2T<Scalar>::Reverse() noexcept {
    const Linear::Point2T<Scalar> vertexB = B;
    B = C;
    C = vertexB;
}

template <typename Scalar> [[nodiscard]] Triangle2T<Scalar> Triangle2T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Triangle2T<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    return Detail::TryTransformTriangle(A, B, C, transform);
}

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::Distance(Linear::Point2T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> Triangle2T<Scalar>::ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
    if (ProjectsInside(point)) {
        return point;
    }
    return ClosestBoundary(point).Point;
}

template <typename Scalar> [[nodiscard]] std::optional<std::variant<Segment2T<Scalar>, PolylineT<Scalar>>>
Triangle2T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
    const auto locate = [this](Scalar parameter) noexcept { return Locate(parameter); };
    return Detail::TriangleSubcurve<Segment2T<Scalar>, PolylineT<Scalar>>(interval, Domain(), locate);
}

template <typename Scalar> [[nodiscard]] Linear::Point2T<Scalar> Triangle2T<Scalar>::Locate(Scalar t) const noexcept {
    return Detail::LocateOnTriangle(A, B, C, t);
}

template <typename Scalar> [[nodiscard]] int Triangle2T<Scalar>::EdgeIndex(Scalar t) noexcept {
    return Detail::TriangleEdgeIndex(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::UnitEdge(Linear::Point2T<Scalar> from,
    Linear::Point2T<Scalar> to) noexcept {
    return Detail::UnitDirection(from, to);
}

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::BoundaryScale() const noexcept {
    return Detail::DiagonalScale(Box());
}

template <typename Scalar> [[nodiscard]] bool Triangle2T<Scalar>::ProjectsInside(Linear::Point2T<Scalar> point) const noexcept {
    const Linear::Vector2T<Scalar> ab = B - A;
    const Linear::Vector2T<Scalar> ac = C - A;
    const Scalar denominator = ab.Cross(ac);
    if (denominator == Scalar{0} || !Core::IsFinite(denominator)) {
        return false;
    }
    const Linear::Vector2T<Scalar> ap = point - A;
    const Scalar alongB = ap.Cross(ac) / denominator;
    const Scalar alongC = ab.Cross(ap) / denominator;
    const Scalar alongA = Scalar{1} - alongB - alongC;
    return Core::IsFinite(alongA) && Core::IsFinite(alongB) && Core::IsFinite(alongC)
        && alongA >= Scalar{0} && alongB >= Scalar{0} && alongC >= Scalar{0};
}

template <typename Scalar>
[[nodiscard]] typename Triangle2T<Scalar>::BoundaryLocation Triangle2T<Scalar>::ClosestBoundary(Linear::Point2T<Scalar> point) const noexcept {
    const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
    return Detail::ClosestOnChain<BoundaryLocation>(vertices, 4, point, true);
}

template <typename Scalar> [[nodiscard]] Scalar Triangle2T<Scalar>::BoundaryParameter(int edge,
        Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to, Linear::Point2T<Scalar> closest) noexcept {
    Scalar parameter = Detail::EdgeParameter(edge, from, to, closest);
    if (parameter == Scalar{3}) {
        parameter = Scalar{0};
    }
    return parameter;
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Triangle2T<double>;
template struct DragonGeo::Prim::Triangle2T<float>;
template bool DragonGeo::Prim::Triangle2T<double>::Contains<double>(DragonGeo::Linear::Point2T<double>) const noexcept;
