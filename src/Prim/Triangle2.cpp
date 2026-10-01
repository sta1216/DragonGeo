#include <DragonGeo/Prim/Triangle2.hpp>

namespace DragonGeo::Prim {

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::SignedArea() const noexcept {
    return Scalar{0.5} * (B - A).Cross(C - A);
}

template <typename Scalar>
[[nodiscard]] Linear::IntervalT<Scalar> Triangle2T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, Scalar{3}};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::MidPoint() const noexcept {
    return PointAt(Scalar{1.5});
}

template <typename Scalar>
template <typename S>
    requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Triangle2T<Scalar>::Contains(Linear::Point2T<S> point) const noexcept {
    const auto onEdge = [](Linear::Point2 start, Linear::Point2 end,
                           Linear::Point2 query) noexcept {
        if (Predicates::Orient2d(start, end, query) != 0) {
            return false;
        }
        if (start == end) {
            return query == start;
        }
        const Linear::Vector2 chord = end - start;
        const double lengthSquared = chord.LengthSquared();
        if (!(lengthSquared > 0.0)) {
            return query == start || query == end;
        }
        const double parameter = (query - start).Dot(chord) / lengthSquared;
        return parameter >= 0.0 && parameter <= 1.0;
    };

    if (Predicates::Orient2d(A, B, C) == 0) {
        return onEdge(A, B, point) || onEdge(B, C, point) || onEdge(C, A, point);
    }
    const int ab = Predicates::Orient2d(A, B, point);
    const int bc = Predicates::Orient2d(B, C, point);
    const int ca = Predicates::Orient2d(C, A, point);
    return (ab >= 0 && bc >= 0 && ca >= 0) || (ab <= 0 && bc <= 0 && ca <= 0);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::StartPoint() const noexcept {
    return A;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::EndPoint() const noexcept {
    return A;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::StartTangent() const noexcept {
    return UnitEdge(A, B);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::EndTangent() const noexcept {
    return UnitEdge(C, A);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::MidTangent() const noexcept {
    return UnitEdge(B, C);
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
    const int edge = EdgeIndex(t);
    return UnitEdge(vertices[edge], vertices[edge + 1]);
}

template <typename Scalar>
[[nodiscard]] bool Triangle2T<Scalar>::IsClosed() const noexcept {
    return true;
}

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::Length() const noexcept {
    return A.DistanceTo(B) + B.DistanceTo(C) + C.DistanceTo(A);
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Triangle2T<Scalar>::Area() const noexcept {
    return Core::AbsoluteValue(SignedArea());
}

template <typename Scalar>
[[nodiscard]] std::optional<Winding> Triangle2T<Scalar>::Orientation() const noexcept {
    const Scalar signedArea = SignedArea();
    if (signedArea > Scalar{0}) {
        return Winding::CounterClockwise;
    }
    if (signedArea < Scalar{0}) {
        return Winding::Clockwise;
    }
    return Winding::Degenerate;
}

template <typename Scalar>
[[nodiscard]] Linear::Box2T<Scalar> Triangle2T<Scalar>::Box() const noexcept {
    const Linear::Point2T<Scalar> vertices[]{A, B, C};
    return Linear::Box2T<Scalar>::FromPoints(std::span<const Linear::Point2T<Scalar>>{vertices});
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point2T<Scalar>> Triangle2T<Scalar>::Centroid() const noexcept {
    if (SignedArea() == Scalar{0}) {
        return std::nullopt;
    }
    return Linear::Point2T<Scalar>{
        (A.X + B.X + C.X) / Scalar{3},
        (A.Y + B.Y + C.Y) / Scalar{3},
    };
}

template <typename Scalar>
[[nodiscard]] bool Triangle2T<Scalar>::ContainsPoint(Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    return std::sqrt(boundary.DistanceSquared) <= tolerance.Resolve(BoundaryScale());
}

template <typename Scalar>
[[nodiscard]] std::optional<Scalar> Triangle2T<Scalar>::ParameterOf(Linear::Point2T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestBoundary(point).Parameter;
}

template <typename Scalar>
void Triangle2T<Scalar>::Translate(Linear::Vector2T<Scalar> vector) noexcept {
    A = A + vector;
    B = B + vector;
    C = C + vector;
}

template <typename Scalar>
void Triangle2T<Scalar>::Rotate(Linear::Point2T<Scalar> center, Scalar radians) noexcept {
    const Linear::Transform2T<Scalar> rotation =
        Linear::Transform2T<Scalar>::RotationAbout(center, radians);
    A = rotation.TransformPoint(A);
    B = rotation.TransformPoint(B);
    C = rotation.TransformPoint(C);
}

template <typename Scalar>
void Triangle2T<Scalar>::Mirror(Linear::Point2T<Scalar> point, Linear::UnitVector2T<Scalar> unitNormal) noexcept {
    const Linear::Transform2T<Scalar> mirror =
        Linear::Transform2T<Scalar>::Reflection(point, unitNormal);
    A = mirror.TransformPoint(A);
    B = mirror.TransformPoint(B);
    C = mirror.TransformPoint(C);
}

template <typename Scalar>
void Triangle2T<Scalar>::Reverse() noexcept {
    const Linear::Point2T<Scalar> vertexB = B;
    B = C;
    C = vertexB;
}

template <typename Scalar>
[[nodiscard]] Triangle2T<Scalar> Triangle2T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar>
[[nodiscard]] bool Triangle2T<Scalar>::Transform(const Linear::Transform2T<Scalar>& transform) noexcept {
    const Linear::Point2T<Scalar> movedA = transform.TransformPoint(A);
    const Linear::Point2T<Scalar> movedB = transform.TransformPoint(B);
    const Linear::Point2T<Scalar> movedC = transform.TransformPoint(C);
    if (!Detail::CoordinatesAreFinite(movedA) || !Detail::CoordinatesAreFinite(movedB)
        || !Detail::CoordinatesAreFinite(movedC)) {
        return false;
    }
    A = movedA;
    B = movedB;
    C = movedC;
    return true;
}

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::DistanceSquared(Linear::Point2T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::Distance(Linear::Point2T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> Triangle2T<Scalar>::ClosestPoint(Linear::Point2T<Scalar> point) const noexcept {
    if (ProjectsInside(point)) {
        return point;
    }
    return ClosestBoundary(point).Point;
}

template <typename Scalar>
[[nodiscard]] std::optional<Segment2T<Scalar>> Triangle2T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const noexcept {
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    for (int edge = 0; edge < 3; ++edge) {
        const Scalar edgeMin = static_cast<Scalar>(edge);
        const Scalar edgeMax = edgeMin + Scalar{1};
        if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
            return Segment2T<Scalar>{Locate(interval.Min), Locate(interval.Max)};
        }
    }
    return std::nullopt;
}

template <typename Scalar>
[[nodiscard]] Linear::Point2T<Scalar> Triangle2T<Scalar>::Locate(Scalar t) const noexcept {
    const Linear::Point2T<Scalar> vertices[]{A, B, C, A};
    const int edge = static_cast<int>(t);
    const int index = edge >= 3 ? 2 : edge;
    const Scalar local = t - static_cast<Scalar>(index);
    return vertices[index] + (vertices[index + 1] - vertices[index]) * local;
}

template <typename Scalar>
[[nodiscard]] int Triangle2T<Scalar>::EdgeIndex(Scalar t) noexcept {
    if (t == Scalar{3}) {
        return 2;
    }
    const int edge = static_cast<int>(t);
    return edge >= 3 ? 2 : edge;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector2T<Scalar>> Triangle2T<Scalar>::UnitEdge(Linear::Point2T<Scalar> from, Linear::Point2T<Scalar> to) noexcept {
    const Scalar length = from.DistanceTo(to);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return (to - from).Normalized();
}

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::BoundaryScale() const noexcept {
    const Scalar diagonal = Box().Extent().Length();
    if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
        return diagonal;
    }
    return Scalar{1};
}

template <typename Scalar>
[[nodiscard]] bool Triangle2T<Scalar>::ProjectsInside(Linear::Point2T<Scalar> point) const noexcept {
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
    BoundaryLocation best{};
    for (int edge = 0; edge < 3; ++edge) {
        const Segment2T<Scalar> segment{vertices[edge], vertices[edge + 1]};
        const Linear::Point2T<Scalar> candidate = segment.ClosestPoint(point);
        const Scalar distanceSquared = (point - candidate).LengthSquared();
        const Scalar parameter =
            BoundaryParameter(edge, vertices[edge], vertices[edge + 1], candidate);
        if (edge == 0 || distanceSquared < best.DistanceSquared
            || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
            best = BoundaryLocation{distanceSquared, parameter, candidate};
        }
    }
    return best;
}

template <typename Scalar>
[[nodiscard]] Scalar Triangle2T<Scalar>::BoundaryParameter(int edge,
        Linear::Point2T<Scalar> from,
        Linear::Point2T<Scalar> to,
        Linear::Point2T<Scalar> closest) noexcept {
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
    if (parameter == Scalar{3}) {
        parameter = Scalar{0};
    }
    return parameter;
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Triangle2T<double>;
template struct DragonGeo::Prim::Triangle2T<float>;
template bool DragonGeo::Prim::Triangle2T<double>::Contains<double>(DragonGeo::Linear::Point2T<double>) const noexcept;
