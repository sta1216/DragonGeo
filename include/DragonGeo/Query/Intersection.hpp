#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Linear/Box2.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/OrientedBox2.hpp>
#include <DragonGeo/Linear/OrientedBox3.hpp>
#include <DragonGeo/Predicates/Predicates.hpp>
#include <DragonGeo/Prim/Line2.hpp>
#include <DragonGeo/Prim/Line3.hpp>
#include <DragonGeo/Prim/Plane.hpp>
#include <DragonGeo/Prim/Ray2.hpp>
#include <DragonGeo/Prim/Ray3.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Segment3.hpp>
#include <DragonGeo/Prim/Triangle2.hpp>
#include <DragonGeo/Prim/Triangle3.hpp>
#include <DragonGeo/Query/CurveMeet.hpp>

namespace DragonGeo::Query {


/// 两条直线相交。平行且分离为 `None`，重合为 `Coincident`，其余为一个点。
/// 点的参数是沿各自方向的有符号距离。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Line2 first, Prim::Line2 second) noexcept;

/// 两条直线是否相交，包括重合。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 first, Prim::Line2 second) noexcept;

/// 两条线段相交。只共享端点或内部交点为 `Point`。共线且重叠长度为零也是 `Point`。
/// 正长度重叠为 `Overlap`，重叠段沿第一条线段的方向。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Segment2 first, Prim::Segment2 second) noexcept;

/// 两条线段是否相交，包括端点相接和共线重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 first, Prim::Segment2 second) noexcept;

/// 直线与线段相交。线段整段落在直线上时为 `Overlap`，重叠段沿直线方向。
/// 零长度线段落在直线上时为 `Point`。`noexcept`。
[[nodiscard]] CurveMeet2 Intersection(Prim::Line2 line, Prim::Segment2 segment) noexcept;

/// 直线与线段是否相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 line, Prim::Segment2 segment) noexcept;

/// 射线与平面。平行且不在平面上不相交。射线躺在平面上时相交，但没有单个交点。
/// 参数是沿射线方向的距离，且 `>= 0`。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Ray3 ray, Prim::Plane plane) noexcept;

/// 射线是否打到平面，包括整条射线躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Prim::Plane plane) noexcept;

/// 线段与平面。正长度线段躺在平面上时相交，`Intersection` 为空。
/// 零长度线段落在平面上时返回该点。参数落在 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Segment3 segment,
                                                                 Prim::Plane plane) noexcept;

/// 线段是否打到平面，包括整段躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Prim::Plane plane) noexcept;

/// 直线与平面。躺在平面上为 `Coincident`，平行且分离为 `None`，其余为一个点。
/// 参数是沿直线方向的有符号距离。`noexcept`。
[[nodiscard]] CurveMeet3 Intersection(Prim::Line3 line, Prim::Plane plane) noexcept;

/// 直线是否打到平面，包括躺在平面上。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Prim::Plane plane) noexcept;

/// 射线与三角形。横向穿过或点接触返回交点。共面重叠时相交，但没有单个交点。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Ray3 ray,
                                                                 Prim::Triangle3 triangle) noexcept;

/// 射线是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Prim::Triangle3 triangle) noexcept;

/// 线段与三角形。横向穿过返回交点，参数在 `[0, 1]`。共面重叠时 `Intersection` 为空。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Segment3 segment,
                                                                 Prim::Triangle3 triangle) noexcept;

/// 线段是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Prim::Triangle3 triangle) noexcept;

/// 直线与三角形。横向穿过返回交点。共面重叠时相交，`Intersection` 为空。`noexcept`。
[[nodiscard]] std::optional<ParameterPoint3> Intersection(Prim::Line3 line,
                                                                 Prim::Triangle3 triangle) noexcept;

/// 直线是否打到三角形，包括共面重叠。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Prim::Triangle3 triangle) noexcept;

/// 射线与轴对齐盒。空盒不相交。起点在体内时 `Enter` 为 0。相切时 `Enter == Exit`。
/// 两个参数都 `>= 0`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Ray2 ray, Linear::Box2 box) noexcept;

/// 射线是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray2 ray, Linear::Box2 box) noexcept;

/// 线段与轴对齐盒。参数落在 `[0, 1]`。起点在体内时 `Enter` 为 0。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Segment2 segment,
                                                                    Linear::Box2 box) noexcept;

/// 线段是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 segment, Linear::Box2 box) noexcept;

/// 直线与轴对齐盒。参数是有符号距离。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Line2 line, Linear::Box2 box) noexcept;

/// 直线是否打到轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line2 line, Linear::Box2 box) noexcept;

/// 射线与三维轴对齐盒。空盒不相交。约定与二维相同。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Ray3 ray, Linear::Box3 box) noexcept;

/// 射线是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, Linear::Box3 box) noexcept;

/// 线段与三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Segment3 segment,
                                                                    Linear::Box3 box) noexcept;

/// 线段是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, Linear::Box3 box) noexcept;

/// 直线与三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Line3 line, Linear::Box3 box) noexcept;

/// 直线是否打到三维轴对齐盒。空盒不相交。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Line3 line, Linear::Box3 box) noexcept;

/// 射线与二维有向盒。先用 `Coordinate.ToLocal` 变到盒的标架，再对以原点为中心、
/// 半轴为 `HalfExtent` 的盒做平板测试。不提供直线版本。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Ray2 ray,
                                                                    const Linear::OrientedBox2& box) noexcept;

/// 射线是否打到二维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray2 ray, const Linear::OrientedBox2& box) noexcept;

/// 线段与二维有向盒。参数仍是原线段上的 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval2> Intersection(Prim::Segment2 segment,
                                                                    const Linear::OrientedBox2& box) noexcept;

/// 线段是否打到二维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment2 segment, const Linear::OrientedBox2& box) noexcept;

/// 射线与三维有向盒。先 `Coordinate.ToLocal`，再对 `[-HalfExtent, HalfExtent]` 做平板测试。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Ray3 ray,
                                                                    const Linear::OrientedBox3& box) noexcept;

/// 射线是否打到三维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Ray3 ray, const Linear::OrientedBox3& box) noexcept;

/// 线段与三维有向盒。参数仍是原线段上的 `[0, 1]`。`noexcept`。
[[nodiscard]] std::optional<ParameterInterval3> Intersection(Prim::Segment3 segment,
                                                                    const Linear::OrientedBox3& box) noexcept;

/// 线段是否打到三维有向盒。`noexcept`。
[[nodiscard]] bool Intersects(Prim::Segment3 segment, const Linear::OrientedBox3& box) noexcept;

/// 两条线段的平方距离。相交时为 0。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Segment2 first, Prim::Segment2 second) noexcept;

/// 两条线段的距离，即 `DistanceSquared` 的平方根，非负。`noexcept`。
[[nodiscard]] double Distance(Prim::Segment2 first, Prim::Segment2 second) noexcept;

/// 两条三维线段的平方距离。相交时为 0。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Segment3 first, Prim::Segment3 second) noexcept;

/// 两条三维线段的距离，非负。`noexcept`。
[[nodiscard]] double Distance(Prim::Segment3 first, Prim::Segment3 second) noexcept;

/// 两个三角形的平方距离。相交时为 0；否则是顶点到另一三角形、以及边到另一三角形的边的最小值。`noexcept`。
[[nodiscard]] double DistanceSquared(Prim::Triangle3 first, Prim::Triangle3 second) noexcept;

/// 两个三角形的距离，非负。相交时为 0。`noexcept`。
[[nodiscard]] double Distance(Prim::Triangle3 first, Prim::Triangle3 second) noexcept;

} // namespace DragonGeo::Query
