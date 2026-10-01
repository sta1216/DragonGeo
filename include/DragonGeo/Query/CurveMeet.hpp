#pragma once

#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Point3.hpp>
#include <DragonGeo/Prim/Segment2.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

namespace DragonGeo::Query {

/// 两条曲线的相交形态。
///
/// `Point` 是一个交点。`Overlap` 是有限的正长度重叠。`Coincident` 是两条无穷曲线重合，没有有限的重叠段。`None` 是不相交，包括平行且分离。
enum class CurveMeet { None, Point, Overlap, Coincident };

/// 二维曲线相交。`Point` 时 `Point` 与两个参数有效。`Overlap` 时只有 `Overlap` 有效，重叠段沿第一个对象的方向。`Coincident` 与 `None` 不使用其余字段。
struct CurveMeet2 {
    CurveMeet Kind = CurveMeet::None;
    Linear::Point2 Point{};
    double ParameterOnFirst = 0;
    double ParameterOnSecond = 0;
    Prim::Segment2 Overlap{};
};

/// 三维曲线相交。字段含义与 `CurveMeet2` 相同。
struct CurveMeet3 {
    CurveMeet Kind = CurveMeet::None;
    Linear::Point3 Point{};
    double ParameterOnFirst = 0;
    double ParameterOnSecond = 0;
    Prim::Segment3 Overlap{};
};

/// 射线上的一个点。`Parameter` 沿第一个对象：射线是有符号距离且 `>= 0`，线段落在 `[0, 1]`，直线是有符号距离。
struct ParameterPoint2 {
    double Parameter = 0;
    Linear::Point2 Point{};
};

/// 三维版本的 `ParameterPoint2`。
struct ParameterPoint3 {
    double Parameter = 0;
    Linear::Point3 Point{};
};

/// 曲线穿入凸体的参数区间。`Enter <= Exit`。相切时两者相等。射线或线段的起点在体内时，`Enter` 是该对象允许的最小参数。
struct ParameterInterval2 {
    double Enter = 0;
    double Exit = 0;
    Linear::Point2 EnterPoint{};
    Linear::Point2 ExitPoint{};
};

/// 三维版本的 `ParameterInterval2`。
struct ParameterInterval3 {
    double Enter = 0;
    double Exit = 0;
    Linear::Point3 EnterPoint{};
    Linear::Point3 ExitPoint{};
};

} // namespace DragonGeo::Query
