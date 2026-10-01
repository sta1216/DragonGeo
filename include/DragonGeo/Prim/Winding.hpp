#pragma once

namespace DragonGeo::Prim {

/// 闭合曲线的绕行方向。有符号面积恰好为零是 `Degenerate`。
enum class Winding { CounterClockwise, Clockwise, Degenerate };

} // namespace DragonGeo::Prim
