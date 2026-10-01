# DragonGeo Prim 与 Query 基准基线

`benchmarks/PrimBenchmarks.cpp` 与 `benchmarks/QueryBenchmarks.cpp` 的一次 Release 运行。数字只在这台机器、这次编译下有意义。线性代数那一组仍以 `benchmarks/BASELINE.md` 为准。

## 环境

与 `benchmarks/BASELINE.md` 同一台机器、同一套编译配置。本次 Google Benchmark 报告 20 逻辑核 × 2112 MHz。

| 项 | 值 |
| --- | --- |
| CPU | 12th Gen Intel(R) Core(TM) i7-12700 |
| OS | Windows 10 Enterprise 10.0.19045 |
| 编译器 | MSVC 19.39.33523，x64，Release `/O2` |
| Google Benchmark | v1.9.1 |
| 采样 | `--benchmark_min_time=0.5s` |

```bash
cmake --preset windows-vs -DDRAGONGEO_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target DragonGeoBenchmarks
./build/windows-vs/benchmarks/Release/DragonGeoBenchmarks.exe --benchmark_filter="Prim|Query" --benchmark_min_time=0.5s
```

## 结果

2026-10-01，无跳过项。夹具在计时前检查了预期分支，对不上会跳过。

```
PrimSegment2ClosestPoint                             16.6 ns         16.4 ns     44800000
PrimSegment2DistanceSquared                          15.0 ns         14.6 ns     40727273
PrimSegment2ContainsPoint                            48.9 ns         49.7 ns     14451613
PrimSegment2ParameterOf                              60.1 ns         60.0 ns     11200000
PrimSegment2Box                                      1.88 ns         1.88 ns    373333333
PrimSegment2Transform                                17.1 ns         17.3 ns     40727273
PrimSegment3ClosestPoint                             19.8 ns         19.9 ns     34461538
PrimSegment3DistanceSquared                          20.9 ns         20.9 ns     34461538
PrimSegment3ContainsPoint                            67.5 ns         67.0 ns     11200000
PrimSegment3ParameterOf                              93.1 ns         90.0 ns      7466667
PrimSegment3Box                                      2.83 ns         2.85 ns    235789474
PrimSegment3Transform                                35.8 ns         35.3 ns     19478261
PrimRay2ClosestPoint                                 2.45 ns         2.46 ns    280000000
PrimRay2DistanceSquared                              2.42 ns         2.40 ns    280000000
PrimRay2ContainsPoint                                3.32 ns         3.30 ns    213333333
PrimRay2ParameterOf                                  4.60 ns         4.65 ns    154482759
PrimRay2Box                                          1.35 ns         1.34 ns    560000000
PrimRay2Transform                                    12.3 ns         11.7 ns     56000000
PrimRay3ClosestPoint                                 3.11 ns         3.07 ns    224000000
PrimRay3DistanceSquared                              3.51 ns         3.53 ns    203636364
PrimRay3ContainsPoint                                4.09 ns         4.10 ns    179200000
PrimRay3ParameterOf                                  5.72 ns         5.78 ns    100000000
PrimRay3Box                                          1.53 ns         1.53 ns    448000000
PrimRay3Transform                                    33.3 ns         33.8 ns     20363636
PrimLine2ClosestPoint                                2.18 ns         2.20 ns    320000000
PrimLine2DistanceSquared                             2.22 ns         2.25 ns    320000000
PrimLine2ContainsPoint                               3.07 ns         3.07 ns    224000000
PrimLine2ParameterOf                                 4.23 ns         4.26 ns    172307692
PrimLine2Box                                         1.32 ns         1.31 ns    560000000
PrimLine2Transform                                   11.7 ns         11.7 ns     64000000
PrimLine3ClosestPoint                                3.40 ns         3.37 ns    213333333
PrimLine3DistanceSquared                             3.44 ns         3.40 ns    179200000
PrimLine3ContainsPoint                               3.98 ns         4.01 ns    179200000
PrimLine3ParameterOf                                 5.52 ns         5.44 ns    112000000
PrimLine3Box                                         1.59 ns         1.60 ns    448000000
PrimLine3Transform                                   33.3 ns         33.7 ns     21333333
PrimTriangle2Contains                                18.6 ns         18.4 ns     37333333
PrimTriangle2ContainsPoint                            189 ns          188 ns      3733333
PrimTriangle2ClosestPoint                             151 ns          150 ns      4480000
PrimTriangle2Area                                    1.55 ns         1.53 ns    407272727
PrimTriangle2ParameterOf                              185 ns          188 ns      4072727
PrimTriangle3Contains                                41.7 ns         41.7 ns     17230769
PrimTriangle3ContainsPoint                            181 ns          180 ns      4072727
PrimTriangle3ClosestPoint                             137 ns          138 ns      4977778
PrimTriangle3Area                                    8.67 ns         8.58 ns     74666667
PrimTriangle3ParameterOf                              173 ns          173 ns      4072727
PrimPlaneSignedDistance                              2.39 ns         2.41 ns    298666667
PrimPlaneProject                                     2.64 ns         2.61 ns    263529412
PrimPlaneMirror                                      3.27 ns         3.30 ns    213333333
PrimPlaneContains                                    2.86 ns         2.89 ns    248888889
PrimPlaneClosestPoint                                3.16 ns         3.21 ns    224000000
PrimPolylineLength/16                                71.8 ns         71.1 ns     11200000
PrimPolylineLength/1024                              4281 ns         4290 ns       149333
PrimPolylineContains/16                               105 ns          105 ns      6400000
PrimPolylineContains/1024                            5454 ns         5469 ns       100000
PrimPolylineContainsPoint/16                          986 ns          984 ns       746667
PrimPolylineContainsPoint/1024                      60090 ns        59989 ns        11200
PrimPolylineParameterOf/16                            863 ns          872 ns       896000
PrimPolylineParameterOf/1024                        56190 ns        54408 ns        11200
PrimPolylineBox/16                                   97.4 ns         95.2 ns      6400000
PrimPolylineBox/1024                                 5442 ns         5441 ns       112000
PrimPolylineArea/16                                  8.31 ns         8.37 ns     89600000
PrimPolylineArea/1024                                 467 ns          465 ns      1544828
PrimPolylineTransform                                80.1 ns         76.7 ns      8960000
PrimPolyline3Length/16                               96.1 ns         96.3 ns      7466667
PrimPolyline3Length/1024                             6372 ns         6278 ns       112000
PrimPolyline3Contains/16                              330 ns          330 ns      2036364
PrimPolyline3Contains/1024                          19409 ns        19496 ns        34462
PrimPolyline3ContainsPoint/16                         809 ns          820 ns       896000
PrimPolyline3ContainsPoint/1024                     50478 ns        51562 ns        10000
PrimPolyline3ParameterOf/16                           803 ns          795 ns       746667
PrimPolyline3ParameterOf/1024                       52153 ns        53125 ns        10000
PrimPolyline3Box/16                                   126 ns          123 ns      5600000
PrimPolyline3Box/1024                                6782 ns         6801 ns        89600
PrimPolyline3Area/16                                 60.9 ns         60.0 ns     11200000
PrimPolyline3Area/1024                               2414 ns         2400 ns       280000
PrimPolyline3Transform                                109 ns          107 ns      6400000
QueryLine2Line2                                      11.3 ns         11.2 ns     64000000
QueryLine2Line2Miss                                  13.7 ns         13.8 ns     49777778
QueryLine2Line2Coincident                            14.0 ns         13.8 ns     49777778
QuerySegment2Segment2                                28.0 ns         28.3 ns     24888889
QuerySegment2Segment2Miss                            33.2 ns         31.8 ns     23578947
QuerySegment2Segment2Overlap                         20.6 ns         20.4 ns     34461538
QueryLine2Segment2                                   16.0 ns         15.7 ns     40727273
QueryLine2Segment2Miss                               15.4 ns         15.3 ns     44800000
QueryLine2Segment2Overlap                            15.3 ns         15.3 ns     44800000
QueryRay2Box2                                        8.34 ns         8.37 ns     74666667
QueryRay2Box2Miss                                    8.93 ns         9.00 ns     74666667
QueryRay2Box2AxisAligned                             8.45 ns         8.54 ns     89600000
QuerySegment2Box2                                    8.76 ns         8.16 ns     74666667
QueryLine2Box2                                       7.92 ns         7.95 ns     74666667
QueryRay2OrientedBox2                                8.43 ns         8.37 ns     89600000
QuerySegment2OrientedBox2                            8.51 ns         8.58 ns     74666667
QueryRay3Plane                                       22.8 ns         23.0 ns     29866667
QueryRay3PlaneMiss                                   13.1 ns         13.1 ns     56000000
QueryRay3PlaneOnPlane                                13.3 ns         13.4 ns     56000000
QuerySegment3Plane                                   25.2 ns         25.6 ns     29866667
QuerySegment3PlaneOutside                            19.7 ns         19.5 ns     34461538
QueryLine3Plane                                      22.0 ns         22.0 ns     32000000
QueryRay3Triangle3                                   63.0 ns         62.8 ns     11200000
QueryRay3Triangle3Miss                               56.5 ns         55.8 ns     11200000
QueryRay3Triangle3Coplanar                           95.0 ns         94.2 ns      7466667
QuerySegment3Triangle3                               63.6 ns         62.8 ns     11200000
QuerySegment3Triangle3Outside                        34.2 ns         34.5 ns     20363636
QueryLine3Triangle3                                  59.6 ns         60.0 ns     11200000
QueryRay3Box3                                        18.9 ns         18.8 ns     37333333
QuerySegment3Box3                                    20.9 ns         20.9 ns     34461538
QueryLine3Box3                                       20.1 ns         19.9 ns     34461538
QueryRay3OrientedBox3                                26.9 ns         27.2 ns     23578947
QuerySegment3OrientedBox3                            26.4 ns         26.2 ns     28000000
QueryDistanceSquaredSegment2Interior                 7.80 ns         7.81 ns    112000000
QueryDistanceSquaredSegment2Clamp                    7.49 ns         7.50 ns     89600000
QueryDistanceSquaredSegment2Parallel                 6.81 ns         6.84 ns    112000000
QueryDistanceSquaredSegment2Zero                     6.08 ns         6.00 ns    112000000
QueryDistanceSquaredSegment3Skew                     23.1 ns         22.5 ns     32000000
QueryDistanceSquaredTriangle3Intersect               90.0 ns         90.0 ns      7466667
QueryDistanceSquaredTriangle3Separated                172 ns          173 ns      4072727
QueryDistanceSquaredTriangle3CoplanarOverlap         96.4 ns         96.3 ns      7466667
QueryDistanceSquaredTriangle3CoplanarSeparated        489 ns          487 ns      1445161
QueryIntersectsLine2Line2                            7.16 ns         7.25 ns    112000000
QueryIntersectsSegment2Segment2                      25.5 ns         25.1 ns     28000000
QueryIntersectsLine2Segment2                         15.4 ns         15.4 ns     49777778
QueryIntersectsRay2Box2                              8.05 ns         8.02 ns     89600000
QueryIntersectsSegment2Box2                          8.90 ns         8.72 ns     89600000
QueryIntersectsLine2Box2                             8.44 ns         8.37 ns     74666667
QueryIntersectsRay2OrientedBox2                      9.08 ns         9.07 ns     89600000
QueryIntersectsSegment2OrientedBox2                  9.08 ns         9.21 ns     74666667
QueryIntersectsRay3Plane                             18.9 ns         18.8 ns     37333333
QueryIntersectsSegment3Plane                         19.0 ns         19.3 ns     37333333
QueryIntersectsLine3Plane                            14.0 ns         13.8 ns     49777778
QueryIntersectsRay3Triangle3                         52.0 ns         51.6 ns     10000000
QueryIntersectsSegment3Triangle3                     54.6 ns         54.7 ns     10000000
QueryIntersectsLine3Triangle3                        51.7 ns         51.6 ns     10000000
QueryIntersectsRay3Box3                              19.2 ns         19.0 ns     34461538
QueryIntersectsSegment3Box3                          17.7 ns         17.4 ns     44800000
QueryIntersectsLine3Box3                             19.5 ns         19.5 ns     34461538
QueryIntersectsRay3OrientedBox3                      21.0 ns         21.3 ns     34461538
QueryIntersectsSegment3OrientedBox3                  26.0 ns         26.2 ns     28000000
QueryDistanceSegment2                                6.97 ns         6.98 ns     89600000
QueryDistanceSegment3                                28.7 ns         28.9 ns     24888889
QueryDistanceTriangle3                                177 ns          176 ns      4072727
```

## 规模

1024 个点相对 16 个点是 64 倍。耗时比超过 128 才算偏离线性。下面都用 Time 列。

| 基准 | 16 | 1024 | 倍数 |
| --- | --- | --- | --- |
| Polyline Length | 71.8 ns | 4281 ns | 60 |
| Polyline Contains | 105 ns | 5454 ns | 52 |
| Polyline ContainsPoint | 986 ns | 60090 ns | 61 |
| Polyline ParameterOf | 863 ns | 56190 ns | 65 |
| Polyline Box | 97.4 ns | 5442 ns | 56 |
| Polyline Area | 8.31 ns | 467 ns | 56 |
| Polyline3 Length | 96.1 ns | 6372 ns | 66 |
| Polyline3 Contains | 330 ns | 19409 ns | 59 |
| Polyline3 ContainsPoint | 809 ns | 50478 ns | 62 |
| Polyline3 ParameterOf | 803 ns | 52153 ns | 65 |
| Polyline3 Box | 126 ns | 6782 ns | 54 |
| Polyline3 Area | 60.9 ns | 2414 ns | 40 |

## 改过的实现

改之前，`ParameterOf` 先调用 `ContainsPoint`，再把最近点重算一遍。1024 点的折线因此大约是 `ContainsPoint` 的两倍：二维 111 µs 对 60 µs，三维 95 µs 对 52 µs。三角形同样，二维 370 ns 对 193 ns。

现在 `ParameterOf` 只走一次最近点，判定式与原来的 `ContainsPoint` 相同。改完后二维折线 56 µs，三维 52 µs，三角形二维 185 ns、三维 173 ns，和各自的 `ContainsPoint` 同一量级。距离更近者优先、距离相等时参数更小者优先，这个次序没变。相关测试 46 例、743 条断言通过。

其余项没有改。求交和求距离的相离、重叠、共面分支都在几十到几百纳秒，没有第二次扫描或循环内分配。
