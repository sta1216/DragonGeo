# DragonGeo 线性代数基准基线

阶段 2 的可执行基线：`benchmarks/LinearBenchmarks.cpp` 覆盖会被上层高频调用的
线性代数原子操作。**基准数字脱离环境没有意义**，所以先固定环境，再谈数字。

## 环境与构建配置

| 项 | 值 |
| --- | --- |
| CPU | 12th Gen Intel(R) Core(TM) i7-12700（Google Benchmark 报告 20 逻辑核 × 2112 MHz 基频） |
| OS | Windows 10 Enterprise 10.0.19045 |
| 编译器 | MSVC 19.39.33523（`_MSC_FULL_VER=193933523`；工具集目录 `14.39.33519`；Visual Studio 2022 **17.9.5**），x64 |
| 优化 | **Release**：`/MD /O2 /Ob2 /DNDEBUG`，另加全库统一的 `/W4 /permissive- /utf-8` |
| CMake | 3.29.0-rc4，生成器 Visual Studio 17 2022 |
| Google Benchmark | v1.9.1（FetchContent，仅开发期依赖，默认不参与构建） |
| 采样 | `--benchmark_min_time=0.5s`，每项至少 0.5 秒 |

版本号的来源写清楚，免得复现的人对不上：`_MSC_FULL_VER` 由
`cl` 预处理宏读出，VS 版本由 `vswhere -latest -products * -property
catalog_productDisplayVersion` 读出。**命令行构建时 MSBuild 自报的 `17.9.8`
是 MSBuild 的版本号，不是 Visual Studio 的版本号** —— 两者不同，别混用。
工具集目录名 `14.39.33519` 与 `_MSC_FULL_VER` 的末几位（33523）本就允许不一致。

复现命令（Git Bash，仓库根目录）：

```bash
cmake --preset windows-vs -DDRAGONGEO_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target DragonGeoBenchmarks
./build/windows-vs/benchmarks/Release/DragonGeoBenchmarks.exe --benchmark_min_time=0.5s
```

## 基线结果

本机、Release、`--benchmark_min_time=0.5s` 的一次完整运行
（同一二进制重复运行之间的波动约 5%，见文末"已知波动"）：

```
---------------------------------------------------------------------
Benchmark                           Time             CPU   Iterations
---------------------------------------------------------------------
BenchEmpty                     0.464 ns        0.460 ns   1493333333
BenchDot                        1.65 ns         1.65 ns    407272727
BenchCross                      1.86 ns         1.84 ns    373333333
BenchLength                     6.51 ns         6.56 ns    112000000
BenchNormalized                 8.72 ns         8.72 ns     89600000
BenchSubscript                  2.10 ns         2.09 ns    344615385
BenchMatrixVector              3.26 ns         3.22 ns    213333333
BenchMatrixInverse             51.2 ns         51.6 ns     10000000
BenchQuaternionRotate          2.92 ns         2.93 ns    224000000
BenchBoxContains               1.99 ns         1.99 ns    344615385
BenchCoordinateToParent       2.33 ns         2.34 ns    320000000
```

同一二进制复跑（审查后重跑确认量级不变）：`BenchEmpty` 0.436、`BenchDot` 1.52、
`BenchCross` 1.76、`BenchLength` 6.20、`BenchNormalized` 7.92、`BenchSubscript`
1.98、`BenchMatrixVector` 3.07、`BenchMatrixInverse` 48.5、
`BenchQuaternionRotate` 2.82、`BenchBoxContains` 1.93、
`BenchCoordinateToParent` 2.21 ns。

## 反空转验证

这一节要防的是**假快**，不是假慢。一个被常量传播掉的基准会给出零点几纳秒的
漂亮数字，而"越快越好"的直觉会把它当成好消息。所以数字之下必须先有证据。

### 1. 零点：`BenchEmpty`

`BenchEmpty` = 空循环 + 一次 `DoNotOptimize(sink)`，**0.464 ns**。它是这套
基准的分辨率下限：比它明显快的东西只能是"编译器把活干掉了"。

### 2. 各基准与零点的比值

| 基准 | Time | 相对 `BenchEmpty` |
| --- | --- | --- |
| BenchEmpty | 0.464 ns | 1.0× |
| BenchDot | 1.65 ns | 3.6× |
| BenchCross | 1.86 ns | 4.0× |
| BenchSubscript | 2.10 ns | 4.5× |
| BenchBoxContains | 1.99 ns | 4.3× |
| BenchCoordinateToParent | 2.33 ns | 5.0× |
| BenchQuaternionRotate | 2.92 ns | 6.3× |
| BenchMatrixVector | 3.26 ns | 7.0× |
| BenchLength | 6.51 ns | 14.0× |
| BenchNormalized | 8.72 ns | 18.8× |
| BenchMatrixInverse | 51.2 ns | 110× |

**每个基准都明显慢于零点**（最接近的 `dot` 也有 3.6×），没有出现"零点几纳秒"
那一类被折叠的信号。

**但这条判据必要而不充分 —— 不要把它当作"没被折叠"的证明。** 一个同形状的
"纯空气"基准（算式被提出循环、循环里只剩屏障）实测 **1.29 ns ≈ 3.0× 零点**，
照样会通过"明显慢于零点"。比值只能用来快速筛查"整项被折叠成常数"这类最粗暴的
失败；真正的证据是第 3、4 节，反面校准见第 5 节。

### 3. 直接反证 A：同形平凡表达式重测（临时探针，验完已撤回）

上一条只能排除"整项被折叠成常数"。**它不能排除**"算式被提到循环外"或者
"测到的其实全是 `DoNotOptimize` 的开销"。这两个问题用同一招解决：把每个基准
的算式替换成一个**平凡表达式**，保持 `DoNotOptimize` 的次数与实参类型完全同形，
其余一字不动。若耗时几乎不变，原基准测的就是空气；耗时之差才是算式本身的成本。

探针跑在同一二进制里（探针版临时加进 `LinearBenchmarks.cpp`，测完已撤回；
下面右列是探针版的实测值，该次运行 `BenchEmpty` 为 0.471 ns）：

| 基准 | 原样 | 同形平凡探针 | 差值 = 算式本身 | 判断 |
| --- | --- | --- | --- | --- |
| BenchDot | 1.61 ns | 1.38 ns | **0.23 ns** | 算式未被消除，但已被屏障盖住 |
| BenchCross | 1.81 ns | 1.37 ns | 0.44 ns | 同上，接近分辨率下限 |
| BenchSubscript | 2.10 ns | 0.978 ns | 1.12 ns | 可测 |
| BenchLength | 6.74 ns | 0.944 ns | 5.80 ns | 可测 |
| BenchNormalized | 8.53 ns | 0.902 ns | 7.63 ns | 可测 |
| BenchMatrixVector | 3.25 ns | 1.38 ns | 1.87 ns | 可测 |
| BenchMatrixInverse | 51.3 ns | 0.913 ns | 50.4 ns | 可测 |
| BenchQuaternionRotate | 3.08 ns | 1.44 ns | 1.64 ns | 可测 |
| BenchBoxContains | 2.03 ns | 1.62 ns | 0.41 ns | 接近分辨率下限 |
| BenchCoordinateToParent | 2.33 ns | 1.37 ns | 0.96 ns | 可测 |

**没有一个基准的差值为零**，即每一处算式都真的在循环里执行了。

### 4. 直接反证 B：反汇编检查循环体

时序证据之外，直接把基准 TU 编到汇编（`cl /O2 /FAs`，产物在 `build/asm_probe/`，
不进仓库）核对循环体：

- `BenchDot` 的循环里是 **6 次 `movsd` 内存加载 + 3 次 `mulsd` + 2 次 `addsd`**，
  从 `a`、`b` 的栈槽真读真算，最后把结果写进临时槽 —— 没有折叠成常数 `32`。
  `DoNotOptimize(a)`/`DoNotOptimize(b)` 迫使两个向量留在栈上，阻断了常量传播。
- `BenchLength` 的循环里是 **3 次 `divsd` + `sqrtpd` + 一次对 `sqrt` 的非内联调用**。
- `BenchMatrixInverse` 的 `inverse()` 没有内联，循环里是**每次迭代一次真实的
  出线调用**（返回 `std::optional<Matrix3>`），不存在被提出循环的迹象。
- `BenchSubscript` 的循环里是 **3 次非内联的 `call Vector3T<double>::operator[](int)`
  + 2 次屏障** —— 它的 2.1 ns 主要是**调用 + 屏障**开销，不是"下标访问本身的成本"
  （MSVC 没有内联这个成员）。
- `BenchNormalized` 的循环里有一次**非内联的 `call Vector3T<double>::length`**，
  再叠加屏障。

这解释了差值表的形状：`length`/`normalized` 的 6~9 ns 不是屏障开销，而是
"按最大分量缩放防溢出 + 3 次除法 + sqrt"这个算法本身的成本（见
`Vector3T::length` 的实现）。

### 5. 反面校准：折叠输入与提出循环（对照实验）

前四条都是"证明基准没坏"。反过来再问一句：**这些检测手段真的抓得住坏基准吗？**
于是造几个"应该被抓出来"的坏基准当对照。对照组跑在 `build/calib/` 的临时二进制里
（`#include` 提交版基准源码再加变体，**不进仓库**），与提交版基准同进程交替测量：

| 变体 | 实测 | 与提交版对比 | 判定 |
| --- | --- | --- | --- |
| `folded_dot`：字面量 `const` 输入、只 `DoNotOptimize` 结果（**计划原来的缺陷**） | 0.645 ns | 提交版 `BenchDot` 1.51 ns，快 2.3×，贴近零点 0.437 | **时序即可判定** |
| `hoisted_dot`：算式提到循环外，循环内屏障形状不变 | 1.29 ns | 比提交版 1.51 ns 只快 15% | **时序判定不了，只能靠反汇编** |
| `hoisted_length`：算式提到循环外 | 0.862 ns | 提交版 `BenchLength` 6.19 ns，差 7.2× | 时序即可判定 |
| `hoisted_matrix_inverse`：算式提到循环外 | 0.879 ns | 提交版 50.2 ns，差 57× | 时序即可判定 |
| `folded_length`：字面量输入、只 `DoNotOptimize` 结果 | 6.17 ns | 与提交版 6.19 ns 相同 | **该结构折叠不掉** |
| `folded_matrix_inverse`：字面量输入、只 `DoNotOptimize` 结果 | 50.5 ns | 与提交版 50.2 ns 相同 | **该结构折叠不掉** |

（这一轮校准里 `BenchEmpty` = 0.437 ns。审查者独立造的同款对照给出
0.665 / 1.34 / 0.878 / 0.905 ns，与本表逐项一致，差异在运行波动之内。）

三点结论，比单个数字有用：

1. **`length` / `normalized` / `matrix_inverse` 这类"贵"的基准，时序就能判定**：
   空转版本会快 7× 以上，没有判定歧义。
2. **`dot` / `cross` 这类"便宜"的基准，时序判定不了**：纯空气版本只比真版本快
   15%，落在"看起来很正常"的范围里 —— 它们的可信度只能建立在反汇编（第 4 节）上。
3. **计划原本担心的缺陷是真实存在的**：字面量输入 + 只 `DoNotOptimize` 结果，
   `dot` 会被折叠（0.645 vs 1.51，快 2.3×）。反过来说，`length()` 与 `inverse()`
   **即使**用纯字面量输入也不会被折叠：前者结构上有非内联 `sqrt` 调用与分支，
   后者结构上就是不透明调用 —— 这两类**不可能空转**，所以它们对输入写法不敏感。

### 6. 结论

`BenchEmpty` 之上每一项都有可归因的真实成本，反证 A、B 与反面校准三者一致，
并且反面校准划清了"哪些基准时序就能判定、哪些只能靠反汇编"。本基线可作为后续
阶段的回归参考。

## 解读与注意事项（不要跳过）

1. **MSVC 下 `DoNotOptimize` 是"不透明函数调用 + `_ReadWriteBarrier`"。**
   Google Benchmark 在 MSVC 上没有内联汇编可用，`DoNotOptimize(x)` 展开为
   `internal::UseCharPointer(&x)`（库外函数，不可内联）+ 编译器屏障。实测每个
   屏障约 **0.45 ns**：零点（1 个屏障）0.464 ns、两屏障探针约 0.94 ns、
   三屏障探针约 1.4 ns。**因此表中的绝对值包含屏障成本**，`dot`/`cross` 这类
   个位数纳秒的项要按差值表去读。
2. **`dot`/`cross` 的真实算式成本低于这套装置的可靠分辨率。** 它们的差值只有
   0.23 ns 与 0.44 ns，量级与运行间波动相当。诚实的结论是：这两个基准能证明
   "没有被优化掉"和"没有数量级级别的退化"，但**测不出** `dot` 本身的指令成本。
   不要引用 1.65 ns 当作"dot 的开销"。
3. **`matrix_inverse` 的 51 ns 是真账。** 探针显示同形屏障只值 0.913 ns，差值
   50.4 ns 全部来自 `inverse()`：双侧平衡的逐行/逐列求最大元、行列式、9 个余
   子式的除法，且它是不内联的调用。若上层要在紧循环里做大量求逆，这里是明白
   的热点。
4. **`length`/`normalized` 比 `dot` 贵一个数量级**（6.5 / 8.7 ns，差值 5.8 /
   7.6 ns），原因是防溢出的缩放算法本身要做 3 次除法，再加 sqrt。
   `length_squared()` 没有基准 —— 若上层只需要比较长度，用它。
5. **`BenchSubscript` 的 2.1 ns 是"调用 + 屏障"，不是"下标本身的成本"。**
   反汇编显示它每次迭代要做 3 次**非内联的** `operator[](int)` 调用（MSVC 没内联
   这个 `constexpr` 成员），外加 2 次 `DoNotOptimize` 屏障；`BenchNormalized`
   的循环里则有一次非内联的 `length()` 调用。数字是真的（探针差值为正），但
   解读时不要把 `subscript` 的 2 ns 当成"下标访问要 2 ns"。这与第 2 条"`dot`
   只有上界"是同一类诚实要求。
6. **比值不是证明。** 见第 2 节的限定与第 5 节的反面校准：一个纯空气基准
   （1.29 ns ≈ 3.0× 零点）照样"明显慢于零点"。要判定某项没被优化掉，用
   `build/calib/` 那套反面校准，或直接看反汇编。
7. **已知波动**：同一二进制重复运行，各项约 ±5%~8%（例：`BenchDot` 1.51~1.72 ns、
   `BenchEmpty` 0.436~0.464 ns、`BenchMatrixInverse` 48.5~53.0 ns）。跑基线时
   关注趋势与数量级，别对着末位数字做判断。
