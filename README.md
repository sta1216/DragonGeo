# GeoCore

A robust, efficient, and easy-to-use C++ geometry library.

GeoCore targets three capability domains — a general geometry toolkit, 2D
computational geometry, and 3D meshes and graphics — plus a B-rep interface
skeleton for exact solid modelling.

**Status:** early development. The API surface is still `unstable`.

## Requirements

- A C++20 compiler (MSVC 19.30+, GCC 11+, Clang 14+, AppleClang 14+)
- CMake 3.20 or newer

## Building

```bash
cmake --preset windows-vs      # or ninja-debug on Linux/macOS
cmake --build --preset windows-vs-debug
ctest --preset windows-vs-debug
```

## Design principles

- **Robust predicates over epsilon.** Orientation and in-circle decisions are
  exact, so degenerate input gets a correct answer rather than a plausible one.
- **Tolerance is an explicit parameter.** There is no global epsilon constant.
- **Types carry invariants.** `Point`, `Vector`, and `UnitVector` are distinct
  types, so a whole class of bugs is rejected at compile time.

See `docs/superpowers/specs/` for the full design.

## Quick start

```cpp
#include <GeoCore/GeoCore.hpp>
#include <iostream>

int main() {
    using namespace GeoCore::linear;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << dot(a, b) << '\n';   // 32

    // 归一化返回 optional —— 零向量无解这一事实由类型表达，
    // 调用者无法忽略这个分支。
    if (const auto unit = normalize(a)) {
        std::cout << unit->as_vector().length() << '\n';   // 1
    }
}
```

更多示例见 `examples/`。

## What works today

| Area | Status |
|---|---|
| `core` — 常量、数值工具、容差模型 | 完成 |
| `linear` — `Vector2/3/4`、`UnitVector2/3`、`Matrix2/3/4`、`Quaternion`、`Transform2/3` | 完成 |
| `predicates` — 精确方向 / 内切圆判定 | 计划 2 |
| `prim` / `query` — 几何原语与求交查询 | 计划 3 |
| `polygon` / `mesh` / `solid` 接口骨架 | 计划 4 |

尚未实现的能力以接口形式存在时会抛出 `std::logic_error`，绝不静默返回错误结果。
