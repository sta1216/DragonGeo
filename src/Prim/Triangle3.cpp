#include <DragonGeo/Prim/Triangle3.hpp>

namespace DragonGeo::Prim {

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::SignedArea() const noexcept {
    const Linear::Vector3T<Scalar> cross = (B - A).Cross(C - A);
    const Scalar length = cross.Length();
    if (length == Scalar{0}) {
        return Scalar{0};
    }
    Scalar component = cross.X;
    Scalar magnitude = Core::AbsoluteValue(cross.X);
    const Scalar absY = Core::AbsoluteValue(cross.Y);
    if (absY > magnitude) {
        component = cross.Y;
        magnitude = absY;
    }
    const Scalar absZ = Core::AbsoluteValue(cross.Z);
    if (absZ > magnitude) {
        component = cross.Z;
    }
    const Scalar sign = component < Scalar{0} ? Scalar{-1} : Scalar{1};
    return sign * (Scalar{0.5} * length);
}

template <typename Scalar> [[nodiscard]] Linear::IntervalT<Scalar> Triangle3T<Scalar>::Domain() const noexcept {
    return {Scalar{0}, Scalar{3}};
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::PointAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    return Locate(t);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::MidPoint() const noexcept {
    return PointAt(Scalar{1.5});
}

template <typename Scalar> template <typename S> requires std::same_as<Scalar, double> && std::same_as<S, double>
    [[nodiscard]] bool Triangle3T<Scalar>::Contains(Linear::Point3T<S> point) const noexcept {
    if (Predicates::Orient3d(A, B, C, point) != 0) {
        return false;
    }

    const auto collinear = [](Linear::Point3 first, Linear::Point3 second, Linear::Point3 third) noexcept {
        const auto orient = [](double ax, double ay, double bx, double by, double cx, double cy) noexcept {
            return Predicates::Orient2d(Linear::Point2{ax, ay}, Linear::Point2{bx, by}, Linear::Point2{cx, cy});
        };
        return orient(first.X, first.Y, second.X, second.Y, third.X, third.Y) == 0
            && orient(first.Y, first.Z, second.Y, second.Z, third.Y, third.Z) == 0
            && orient(first.Z, first.X, second.Z, second.X, third.Z, third.X) == 0;
    };
    const auto onEdge = [&](Linear::Point3 start, Linear::Point3 end, Linear::Point3 query) noexcept {
        if (start == end) {
            return query == start;
        }
        if (!collinear(start, end, query)) {
            return false;
        }
        const Linear::Vector3 chord = end - start;
        const double lengthSquared = chord.LengthSquared();
        if (!(lengthSquared > 0.0)) {
            return query == start || query == end;
        }
        const double parameter = (query - start).Dot(chord) / lengthSquared;
        return parameter >= 0.0 && parameter <= 1.0;
    };
    if (collinear(A, B, C)) {
        return onEdge(A, B, point) || onEdge(B, C, point) || onEdge(C, A, point);
    }

    const int dropped = DroppedAxis();
    const Triangle2 projected{Project(A, dropped), Project(B, dropped), Project(C, dropped)};
    return projected.Contains(Project(point, dropped));
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::StartPoint() const noexcept {
    return A;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::EndPoint() const noexcept {
    return A;
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Triangle3T<Scalar>::StartTangent() const noexcept {
    return UnitEdge(A, B);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Triangle3T<Scalar>::EndTangent() const noexcept {
    return UnitEdge(C, A);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Triangle3T<Scalar>::MidTangent() const noexcept {
    return UnitEdge(B, C);
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Triangle3T<Scalar>::TangentAt(Scalar t) const noexcept {
    if (!Detail::IsAcceptedParameter(t, Domain())) {
        return std::nullopt;
    }
    const Linear::Point3T<Scalar> vertices[]{A, B, C, A};
    const int edge = EdgeIndex(t);
    return UnitEdge(vertices[edge], vertices[edge + 1]);
}

template <typename Scalar> [[nodiscard]] bool Triangle3T<Scalar>::IsClosed() const noexcept {
    return true;
}

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::Length() const noexcept {
    return A.DistanceTo(B) + B.DistanceTo(C) + C.DistanceTo(A);
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Triangle3T<Scalar>::Area() const noexcept {
    return Core::AbsoluteValue(SignedArea());
}

template <typename Scalar> [[nodiscard]] std::optional<Winding> Triangle3T<Scalar>::Orientation() const noexcept {
    const Scalar signedArea = SignedArea();
    if (signedArea > Scalar{0}) {
        return Winding::CounterClockwise;
    }
    if (signedArea < Scalar{0}) {
        return Winding::Clockwise;
    }
    return Winding::Degenerate;
}

template <typename Scalar> [[nodiscard]] Linear::Box3T<Scalar> Triangle3T<Scalar>::Box() const noexcept {
    const Linear::Point3T<Scalar> vertices[]{A, B, C};
    return Linear::Box3T<Scalar>::FromPoints(std::span<const Linear::Point3T<Scalar>>{vertices});
}

template <typename Scalar> [[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::Centroid() const noexcept {
    if (SignedArea() == Scalar{0}) {
        return std::nullopt;
    }
    return Linear::Point3T<Scalar>{(A.X + B.X + C.X) / Scalar{3}, (A.Y + B.Y + C.Y) / Scalar{3}, (A.Z + B.Z + C.Z) / Scalar{3}};
}

template <typename Scalar>
[[nodiscard]] bool Triangle3T<Scalar>::ContainsPoint(Linear::Point3T<Scalar> point, Core::ToleranceT<Scalar> tolerance) const noexcept {
    const auto boundary = ClosestBoundary(point);
    return std::sqrt(boundary.DistanceSquared) <= tolerance.Resolve(BoundaryScale());
}

template <typename Scalar> [[nodiscard]] std::optional<Scalar> Triangle3T<Scalar>::ParameterOf(Linear::Point3T<Scalar> point,
        Core::ToleranceT<Scalar> tolerance) const noexcept {
    if (!ContainsPoint(point, tolerance)) {
        return std::nullopt;
    }
    return ClosestBoundary(point).Parameter;
}

template <typename Scalar> void Triangle3T<Scalar>::Translate(Linear::Vector3T<Scalar> vector) noexcept {
    A = A + vector;
    B = B + vector;
    C = C + vector;
}

template <typename Scalar>
void Triangle3T<Scalar>::Rotate(Linear::Point3T<Scalar> origin, Linear::UnitVector3T<Scalar> axis, Scalar radians) noexcept {
    const Linear::Transform3T<Scalar> rotation = Linear::Transform3T<Scalar>::RotationAbout(origin, axis, radians);
    A = rotation.TransformPoint(A);
    B = rotation.TransformPoint(B);
    C = rotation.TransformPoint(C);
}

template <typename Scalar> void Triangle3T<Scalar>::Mirror(Linear::Point3T<Scalar> point, Linear::UnitVector3T<Scalar> unitNormal) noexcept {
    const Linear::Transform3T<Scalar> mirror = Linear::Transform3T<Scalar>::Reflection(point, unitNormal);
    A = mirror.TransformPoint(A);
    B = mirror.TransformPoint(B);
    C = mirror.TransformPoint(C);
}

template <typename Scalar> void Triangle3T<Scalar>::Reverse() noexcept {
    const Linear::Point3T<Scalar> vertexB = B;
    B = C;
    C = vertexB;
}

template <typename Scalar> [[nodiscard]] Triangle3T<Scalar> Triangle3T<Scalar>::Clone() const noexcept {
    return *this;
}

template <typename Scalar> [[nodiscard]] bool Triangle3T<Scalar>::Transform(const Linear::Transform3T<Scalar>& transform) noexcept {
    const Linear::Point3T<Scalar> movedA = transform.TransformPoint(A);
    const Linear::Point3T<Scalar> movedB = transform.TransformPoint(B);
    const Linear::Point3T<Scalar> movedC = transform.TransformPoint(C);
    if (!Detail::CoordinatesAreFinite(movedA) || !Detail::CoordinatesAreFinite(movedB) || !Detail::CoordinatesAreFinite(movedC)) {
        return false;
    }
    A = movedA;
    B = movedB;
    C = movedC;
    return true;
}

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::DistanceSquared(Linear::Point3T<Scalar> point) const noexcept {
    return (point - ClosestPoint(point)).LengthSquared();
}

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::Distance(Linear::Point3T<Scalar> point) const noexcept {
    return std::sqrt(DistanceSquared(point));
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Triangle3T<Scalar>::ClosestPoint(Linear::Point3T<Scalar> point) const noexcept {
    if (const auto onFace = FacePoint(point)) {
        return *onFace;
    }
    return ClosestBoundary(point).Point;
}

template <typename Scalar>
[[nodiscard]] std::optional<std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>>
Triangle3T<Scalar>::Subcurve(Linear::IntervalT<Scalar> interval) const {
    using Curve = std::variant<Segment3T<Scalar>, Polyline3T<Scalar>>;
    if (!Detail::IsFiniteSubinterval(interval, Domain())) {
        return std::nullopt;
    }
    for (int edge = 0; edge < 3; ++edge) {
        const Scalar edgeMin = static_cast<Scalar>(edge);
        const Scalar edgeMax = edgeMin + Scalar{1};
        if (interval.Min >= edgeMin && interval.Max <= edgeMax) {
            return Curve{Segment3T<Scalar>{Locate(interval.Min), Locate(interval.Max)}};
        }
    }

    Linear::Point3T<Scalar> corners[4];
    int count = 0;
    corners[count++] = Locate(interval.Min);
    for (int vertex = 1; vertex <= 2; ++vertex) {
        const Scalar parameter = static_cast<Scalar>(vertex);
        if (parameter > interval.Min && parameter < interval.Max) {
            corners[count++] = Locate(parameter);
        }
    }
    corners[count++] = Locate(interval.Max);

    auto polyline = Polyline3T<Scalar>::FromPoints(std::span<const Linear::Point3T<Scalar>>{corners, static_cast<std::size_t>(count)});
    if (!polyline.has_value()) {
        return std::nullopt;
    }
    return Curve{std::move(*polyline)};
}

template <typename Scalar> [[nodiscard]] Linear::Point3T<Scalar> Triangle3T<Scalar>::Locate(Scalar t) const noexcept {
    const Linear::Point3T<Scalar> vertices[]{A, B, C, A};
    const int edge = static_cast<int>(t);
    const int index = edge >= 3 ? 2 : edge;
    const Scalar local = t - static_cast<Scalar>(index);
    return vertices[index] + (vertices[index + 1] - vertices[index]) * local;
}

template <typename Scalar> [[nodiscard]] int Triangle3T<Scalar>::EdgeIndex(Scalar t) noexcept {
    if (t == Scalar{3}) {
        return 2;
    }
    const int edge = static_cast<int>(t);
    return edge >= 3 ? 2 : edge;
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::UnitVector3T<Scalar>> Triangle3T<Scalar>::UnitEdge(Linear::Point3T<Scalar> from,
    Linear::Point3T<Scalar> to) noexcept {
    const Scalar length = from.DistanceTo(to);
    if (!(length > Scalar{0}) || !Core::IsFinite(length)) {
        return std::nullopt;
    }
    return (to - from).Normalized();
}

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::BoundaryScale() const noexcept {
    const Scalar diagonal = Box().Extent().Length();
    if (diagonal > Scalar{0} && Core::IsFinite(diagonal)) {
        return diagonal;
    }
    return Scalar{1};
}

template <typename Scalar>
[[nodiscard]] std::optional<Linear::Point3T<Scalar>> Triangle3T<Scalar>::FacePoint(Linear::Point3T<Scalar> point) const noexcept {
    const Linear::Vector3T<Scalar> ab = B - A;
    const Linear::Vector3T<Scalar> ac = C - A;
    const Linear::Vector3T<Scalar> normal = ab.Cross(ac);
    const Scalar normalLengthSquared = normal.LengthSquared();
    if (!(normalLengthSquared > Scalar{0}) || !Core::IsFinite(normalLengthSquared)) {
        return std::nullopt;
    }
    const Linear::Vector3T<Scalar> ap = point - A;
    const Scalar alongB = ap.Cross(ac).Dot(normal) / normalLengthSquared;
    const Scalar alongC = ab.Cross(ap).Dot(normal) / normalLengthSquared;
    const Scalar alongA = Scalar{1} - alongB - alongC;
    if (!Core::IsFinite(alongA) || !Core::IsFinite(alongB) || !Core::IsFinite(alongC)
        || alongA < Scalar{0} || alongB < Scalar{0} || alongC < Scalar{0}) {
        return std::nullopt;
    }
    return A + ab * alongB + ac * alongC;
}

template <typename Scalar>
[[nodiscard]] typename Triangle3T<Scalar>::BoundaryLocation Triangle3T<Scalar>::ClosestBoundary(Linear::Point3T<Scalar> point) const noexcept {
    const Linear::Point3T<Scalar> vertices[]{A, B, C, A};
    BoundaryLocation best{};
    for (int edge = 0; edge < 3; ++edge) {
        const Segment3T<Scalar> segment{vertices[edge], vertices[edge + 1]};
        const Linear::Point3T<Scalar> candidate = segment.ClosestPoint(point);
        const Scalar distanceSquared = (point - candidate).LengthSquared();
        const Scalar parameter = BoundaryParameter(edge, vertices[edge], vertices[edge + 1], candidate);
        if (edge == 0 || distanceSquared < best.DistanceSquared || (distanceSquared == best.DistanceSquared && parameter < best.Parameter)) {
            best = BoundaryLocation{distanceSquared, parameter, candidate};
        }
    }
    return best;
}

template <typename Scalar> [[nodiscard]] Scalar Triangle3T<Scalar>::BoundaryParameter(int edge,
        Linear::Point3T<Scalar> from, Linear::Point3T<Scalar> to, Linear::Point3T<Scalar> closest) noexcept {
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

template <typename Scalar> [[nodiscard]] int Triangle3T<Scalar>::DroppedAxis() const noexcept {
    const auto cross = (B - A).Cross(C - A);
    const double absX = Core::AbsoluteValue(cross.X);
    const double absY = Core::AbsoluteValue(cross.Y);
    const double absZ = Core::AbsoluteValue(cross.Z);
    if (absX != 0.0 || absY != 0.0 || absZ != 0.0) {
        if (absX >= absY && absX >= absZ) {
            return 0;
        }
        if (absY >= absZ) {
            return 1;
        }
        return 2;
    }
    const auto extent = [](double p, double q, double r) noexcept {
        const double lo = p < q ? p : q;
        const double hi = p < q ? q : p;
        const double low = lo < r ? lo : r;
        const double high = hi > r ? hi : r;
        return high - low;
    };
    const double rangeX = extent(A.X, B.X, C.X);
    const double rangeY = extent(A.Y, B.Y, C.Y);
    const double rangeZ = extent(A.Z, B.Z, C.Z);
    if (rangeX <= rangeY && rangeX <= rangeZ) {
        return 0;
    }
    if (rangeY <= rangeZ) {
        return 1;
    }
    return 2;
}

template <typename Scalar> [[nodiscard]] Linear::Point2 Triangle3T<Scalar>::Project(Linear::Point3 point, int dropped) noexcept {
    if (dropped == 0) {
        return Linear::Point2{point.Y, point.Z};
    }
    if (dropped == 1) {
        return Linear::Point2{point.X, point.Z};
    }
    return Linear::Point2{point.X, point.Y};
}

} // namespace DragonGeo::Prim

template struct DragonGeo::Prim::Triangle3T<double>;
template struct DragonGeo::Prim::Triangle3T<float>;
template bool DragonGeo::Prim::Triangle3T<double>::Contains<double>(DragonGeo::Linear::Point3T<double>) const noexcept;
