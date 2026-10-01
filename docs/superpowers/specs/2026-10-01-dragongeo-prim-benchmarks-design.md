# DragonGeo Prim 与 Query 求交基准

日期：2026-10-01
状态：已实现。基线在 `benchmarks/PRIM_QUERY_BASELINE.md`。

给 `Prim` 里带计算的方法，以及 `Query::Intersection` 的每个重载，加 Google Benchmark。测完之后，只改复杂度不对或有明显浪费的实现。纳秒级常数不动。

线性代数基准仍是 `benchmarks/LinearBenchmarks.cpp` 与 `benchmarks/BASELINE.md`。本文不改那一组数字的含义。

## 1. 怎么组织

两个新文件 `benchmarks/PrimBenchmarks.cpp` 与 `benchmarks/QueryBenchmarks.cpp`，加进现有可执行文件 `DragonGeoBenchmarks`。`benchmarks/CMakeLists.txt` 在现有的 `add_executable` 上多列这两个源文件。仍由 `DRAGONGEO_BUILD_BENCHMARKS` 控制，默认不编。

只测 `double`。Prim 基准函数名以 `Prim` 开头，例如 `PrimSegment2ClosestPoint`。Query 基准函数名以 `Query` 开头，例如 `QuerySegment2Segment2`。`--benchmark_filter=Prim|Query` 因此不会带上线性代数那一组。输入和结果都经过 `benchmark::DoNotOptimize`，对象在循环外构造，循环内再读一次，避免被折成常数。

不测取值：`IsValid`、`Point`、`PointCount`、`Domain`、`Clone`、`StartPoint`、`EndPoint`、`MidPoint`、切向，以及 `operator==`。

二维和三维都测。两边是各自的 `.cpp`，只测一边就看不到另一边的回归。

## 2. 测哪些方法

几何是固定的、非退化的。点在线段或折线边上，三角形用内点，平面用面上的点和一个面外的点。每次调用都走真实计算，不走提前返回的空对象。

线段、射线、直线（`Segment2`/`Segment3`、`Ray2`/`Ray3`、`Line2`/`Line3`）各测：

- `ClosestPoint`
- `DistanceSquared`
- `ContainsPoint`
- `ParameterOf`
- `Box`
- `Transform`（二维绕点旋转，三维绕过原点的单位轴旋转，角度有限且结果保持有限。循环内先拷贝再变换，避免越转越远，也避免失败路径）

三角形（`Triangle2`、`Triangle3`）各测：

- `Contains`
- `ContainsPoint`
- `ClosestPoint`
- `Area`
- `ParameterOf`

平面（`Plane`）测：

- `SignedDistance`
- `Project`（向量）
- `Mirror`（点）
- `Contains`
- `ClosestPoint`

折线（`Polyline`、`Polyline3`）测：

- `Length`
- `Contains`（闭合，查询点在内部）
- `ContainsPoint`
- `ParameterOf`
- `Box`
- `Area`

这六项用 `->Arg(16)->Arg(1024)`，参数是点数。点沿单位圆均匀采样，首尾点相同，查询点用原点，这样闭合折线的内部包含和面积都走真实分支。边数是点数减一。`Transform` 只用 16 个点：1024 个点的拷贝会盖住变换本身。

`Polyline3` 的面积和包含用平面闭合折线。非平面没有填充，`Contains` 恒为假，不拿来当基准。

## 3. Query 求交

只测 `Query::Intersection`。`Intersects` 是对同一次求交结果判空或判 `Kind != None`，不单独测。`Distance` 与 `DistanceSquared` 不在这一轮。

几何都取真正相交的那一支，不测平行、相离或空盒的提前返回。有向盒的标架要带旋转，让 `ToLocal` 做真实变换，而不是平移标架。

二维：

- `Line2` 与 `Line2`
- `Segment2` 与 `Segment2`（内部相交，不是只共享端点）
- `Line2` 与 `Segment2`
- `Ray2`、`Segment2`、`Line2` 各与 `Box2`
- `Ray2`、`Segment2` 各与 `OrientedBox2`

三维：

- `Ray3`、`Segment3`、`Line3` 各与 `Plane`
- `Ray3`、`Segment3`、`Line3` 各与 `Triangle3`（横向穿过）
- `Ray3`、`Segment3`、`Line3` 各与 `Box3`
- `Ray3`、`Segment3` 各与 `OrientedBox3`

没有直线与有向盒的重载，不补。

上面每一条是命中。下面按内核补另外的出口，名字用后缀区分，例如 `QuerySegment2Segment2Overlap`。公共内核只补一次，不把同一出口复制到每个重载上。

| 内核 | 命中之外 |
|---|---|
| 直线 × 直线 | 平行相离、重合 |
| 线段 × 线段 | 同侧相离、共线重叠 |
| 直线 × 线段 | 同侧相离、线段整段在直线上 |
| 曲线 × 平面 | 平行相离、整条躺在平面上，用射线测。线段再加交点落在 `[0, 1]` 外 |
| 曲线 × 三角形 | 横向打空、共面并穿过三角形，用射线测。线段再加参数落在 `[0, 1]` 外 |
| 平板求盒 | 两个分量都非零的相离、方向有一个分量为 0 的命中。用 `Ray2` × `Box2`。有向盒的旋转标架只留在命中那条上 |

`DistanceSquared`：

- `Segment2`：最近点在两条线段内部、夹到端点、平行、其中一条长度为零
- `Segment3`：异面，最近点在两条线段内部且距离为正
- `Triangle3`：相交后得到 0、不共面且分开、共面且重叠、共面但分开

`Intersects` 的每个重载和 `Distance` 的每个重载各测一条命中。`Distance` 用距离为正的那对几何，让开方真的执行。分支不在这两个包装上重复。

## 4. 什么时候改实现

先跑基准，再看代码。满足下面任一条才改，改完行为必须与改前一致：

1. 折线从 16 个点增到 1024 个点后，该项耗时比超过 128。64 倍点数配上缓存造成的两倍出入，上限就是 128。比线性更快不改。Query 求交没有规模参数，不适用这一条。
2. 同一次查询把边表扫了两遍，或在循环里分配，或对同一个量反复开方。`ParameterOf` 先调 `ContainsPoint` 再调 `ClosestBoundary` 就是这种重复，测完后要核对并收成一次扫描。距离更近者优先，距离相等时参数更小者优先，这个次序保持不变。`Length` 按边累加距离必须开方，这不是浪费。求交里同一对谓词或同一组平板参数算了两遍，也按这一条收成一次。

不改的例子：单个线段的 `ClosestPoint` 比 `DistanceSquared` 慢几个纳秒；`Distance` 比 `DistanceSquared` 多一次开方。

改完跑受影响的 Catch2 测试，并重跑 `DragonGeoBenchmarks`。优化不得改变公开结果：空、退化、非有限、符号和返回类型保持原契约。

## 5. 基线

本机 Release、`--benchmark_min_time=0.5s` 跑一次，把环境表和完整输出写入 `benchmarks/PRIM_QUERY_BASELINE.md`。环境项与 `benchmarks/BASELINE.md` 相同：CPU、OS、编译器、优化开关、CMake、Google Benchmark 版本、采样参数。Prim 与 Query 分两节记录。

这份基线不包含反汇编取证。它用来以后看数量级有没有退步，不用来引用「某方法就是多少纳秒」。

复现：

```bash
cmake --preset windows-vs -DDRAGONGEO_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target DragonGeoBenchmarks
./build/windows-vs/benchmarks/Release/DragonGeoBenchmarks.exe --benchmark_filter="Prim|Query" --benchmark_min_time=0.5s
```

过滤名以基准前缀 `Prim` 与 `Query` 为准，这样不会重跑线性代数那一组。

## 6. 不做

- 在 `Intersects` 和 `Distance` 上重复求交或平方距离的分支。
- `float` 实例。
- 给每个取值方法加一条基准。
- 为了几个纳秒去改线性代数或谓词。
- 把基准数字写进断言。
