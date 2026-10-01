#pragma once

#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Point3.hpp>

namespace DragonGeo::Predicates {

/// 三点定向。逆时针为 +1，顺时针为 -1，共线（含点重合）为 0。
///
/// 零表示代数上确实共线，不是“误差在阈值内”。函数不接受容差。坐标必须是有限数。NaN 或无穷时仍不抛异常，返回值不作规定。各坐标的指数跨度大到使中间积下溢到次正规数时，返回值不作规定。
///
/// 实现在库的编译单元里。精确路径可能分配内存。内存耗尽时按 noexcept 的规则终止。
[[nodiscard]] int Orient2d(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c) noexcept;

/// 四点定向。d 在平面 abc 的正侧为 +1，正侧由 (b - a) × (c - a) 决定，共面为 0。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] int Orient3d(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c, Linear::Point3 d) noexcept;

/// a、b、c 逆时针时，d 在圆内为 +1，圆外为 -1，圆上为 0。边界顺时针时符号相反。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] int Incircle(Linear::Point2 a, Linear::Point2 b, Linear::Point2 c, Linear::Point2 d) noexcept;

/// Orient3d(a, b, c, d) 为正时，e 在球内为 +1，球外为 -1，球上为 0。前四个点反向时符号相反。
///
/// 输入约定与 Orient2d 相同。
[[nodiscard]] int Insphere(Linear::Point3 a, Linear::Point3 b, Linear::Point3 c, Linear::Point3 d, Linear::Point3 e) noexcept;

} // namespace DragonGeo::Predicates
