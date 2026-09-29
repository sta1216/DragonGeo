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
