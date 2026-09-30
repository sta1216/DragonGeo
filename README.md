# DragonGeo

A robust, efficient, and easy-to-use C++ geometry library.

DragonGeo targets three capability domains — a general geometry toolkit, 2D
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

These are the commitments the design is built around. One of the three is
target design rather than shipped behaviour today — the layer that carries it
is listed as planned in the table above.

- **Robust predicates over epsilon** *(target design: the `Predicates` layer is
  planned, not present)*. Orientation and in-circle decisions are exact, so
  degenerate input gets a correct answer rather than a plausible one.
- **Tolerance is an explicit parameter.** No global epsilon constant exists.
  Every entry point that needs a threshold takes a `Core::Tolerance`.
- **Types carry invariants.** `UnitVector` is a distinct type from `Vector`, so
  scaling one is a visible change of return type. `Point` is likewise split from
  `Vector` (both live in `Linear`), so adding two points does not compile, and
  every `Coordinate` construction path validates the frame it builds — a
  non-orthogonal or left-handed frame cannot be built through any public
  interface.

See `docs/superpowers/specs/` for the full design. 命名以该文档 §3.4 为准：文件名、类名、方法名、模块名大驼峰；参数和变量小驼峰；public 成员大驼峰；非 public 成员 `m_` + 小驼峰；指针 `p` + 大驼峰（智能指针可用 `up` / `sp` / `wp`）；常量全大写、下划线分词；注释用中文。

## Using DragonGeo from another project

The library is header-only; a consumer needs the include directories and,
on MSVC, the `/utf-8` option — both travel with the exported target, so
linking `DragonGeo::DragonGeo` is enough.

Installed package:

```bash
cmake --install build/windows-vs --config Release --prefix /opt/dragongeo
```

```cmake
find_package(DragonGeo REQUIRED)
target_link_libraries(your_target PRIVATE DragonGeo::DragonGeo)
```

Source tree:

```cmake
include(FetchContent)
FetchContent_Declare(DragonGeo
    GIT_REPOSITORY https://example.invalid/DragonGeo.git   # 替换为实际仓库地址
    GIT_TAG        main)                                  # 或某个发布标签
FetchContent_MakeAvailable(DragonGeo)
target_link_libraries(your_target PRIVATE DragonGeo::DragonGeo)
```

`ci/consumer/` is a minimal consumer of the *installed* package, built in CI;
it deliberately sets no charset option of its own, so that a regression in
what the package exports cannot hide.

## License

MIT — see [LICENSE](LICENSE).

## Quick start

```cpp
#include <DragonGeo/DragonGeo.hpp>
#include <iostream>

int main() {
    using namespace DragonGeo::Linear;

    const Vector3 a{1.0, 2.0, 3.0};
    const Vector3 b{4.0, 5.0, 6.0};

    std::cout << a.Dot(b) << '\n';   // 32

    // 归一化返回 optional —— 零向量无解这一事实由类型表达，
    // 调用者无法忽略这个分支。
    if (const auto unit = a.Normalized()) {
        std::cout << unit->AsVector().Length() << '\n';   // 1
    }
}
```

更多示例见 `examples/`。

## What works today

| Area | Status |
|---|---|
| `core` — 常量、数值工具、容差模型 | 完成 |
| `linear` — `Vector2/3/4`、`UnitVector2/3`、`Matrix2/3/4`、`Quaternion`、`Transform2/3`、`Point2/3`、`Interval`、`Box2/3`、`Coordinate2/3`、`OrientedBox2/3` | 完成 |
| `predicates` — 精确方向 / 内切圆判定 | 计划 2 |
| `prim` / `query` — 几何原语与求交查询 | 计划 3 |
| `polygon` / `mesh` / `solid` 接口骨架 | 计划 4 |

尚未实现的能力以接口形式存在时会抛出 `std::logic_error`，绝不静默返回错误结果。
