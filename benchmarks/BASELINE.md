# GeoCore 线性代数基准基线

阶段 2 的可执行基线：`benchmarks/linear_benchmarks.cpp` 覆盖会被上层高频调用的
线性代数原子操作。**基准数字脱离环境没有意义**，所以先固定环境，再谈数字。

## 环境与构建配置

| 项 | 值 |
| --- | --- |
| CPU | 12th Gen Intel(R) Core(TM) i7-12700（Google Benchmark 报告 20 逻辑核 × 2112 MHz 基频） |
| OS | Windows 10 Enterprise 10.0.19045 |
| 编译器 | MSVC 19.39.33519（Visual Studio 2022 17.9.8），x64 |
| 优化 | **Release**：`/MD /O2 /Ob2 /DNDEBUG`，另加全库统一的 `/W4 /permissive- /utf-8` |
| CMake | 3.29.0-rc4，生成器 Visual Studio 17 2022 |
| Google Benchmark | v1.9.1（FetchContent，仅开发期依赖，默认不参与构建） |
| 采样 | `--benchmark_min_time=0.5s`，每项至少 0.5 秒 |

复现命令（Git Bash，仓库根目录）：

```bash
cmake --preset windows-vs -DGEOCORE_BUILD_BENCHMARKS=ON
cmake --build --preset windows-vs-release --target GeoCoreBenchmarks
./build/windows-vs/benchmarks/Release/GeoCoreBenchmarks.exe --benchmark_min_time=0.5s
```

## 基线结果

本机、Release、`--benchmark_min_time=0.5s` 的一次完整运行
（同一二进制重复运行之间的波动约 5%，见文末"已知波动"）：

```
---------------------------------------------------------------------
Benchmark                           Time             CPU   Iterations
---------------------------------------------------------------------
bench_empty                     0.464 ns        0.460 ns   1493333333
bench_dot                        1.65 ns         1.65 ns    407272727
bench_cross                      1.86 ns         1.84 ns    373333333
bench_length                     6.51 ns         6.56 ns    112000000
bench_normalized                 8.72 ns         8.72 ns     89600000
bench_subscript                  2.10 ns         2.09 ns    344615385
bench_matrix_vector              3.26 ns         3.22 ns    213333333
bench_matrix_inverse             51.2 ns         51.6 ns     10000000
bench_quaternion_rotate          2.92 ns         2.93 ns    224000000
bench_box_contains               1.99 ns         1.99 ns    344615385
bench_coordinate_to_parent       2.33 ns         2.34 ns    320000000
```

## 反空转验证

这一节要防的是**假快**，不是假慢。一个被常量传播掉的基准会给出零点几纳秒的
漂亮数字，而"越快越好"的直觉会把它当成好消息。所以数字之下必须先有证据。

### 1. 零点：`bench_empty`

`bench_empty` = 空循环 + 一次 `DoNotOptimize(sink)`，**0.464 ns**。它是这套
基准的分辨率下限：比它明显快的东西只能是"编译器把活干掉了"。

### 2. 各基准与零点的比值

| 基准 | Time | 相对 `bench_empty` |
| --- | --- | --- |
| bench_empty | 0.464 ns | 1.0× |
| bench_dot | 1.65 ns | 3.6× |
| bench_cross | 1.86 ns | 4.0× |
| bench_subscript | 2.10 ns | 4.5× |
| bench_box_contains | 1.99 ns | 4.3× |
| bench_coordinate_to_parent | 2.33 ns | 5.0× |
| bench_quaternion_rotate | 2.92 ns | 6.3× |
| bench_matrix_vector | 3.26 ns | 7.0× |
| bench_length | 6.51 ns | 14.0× |
| bench_normalized | 8.72 ns | 18.8× |
| bench_matrix_inverse | 51.2 ns | 110× |

**每个基准都明显慢于零点**（最接近的 `dot` 也有 3.6×），没有出现"零点几纳秒"
那一类被折叠的信号。

### 3. 直接反证 A：同形平凡表达式重测（临时探针，验完已撤回）

上一条只能排除"整项被折叠成常数"。**它不能排除**"算式被提到循环外"或者
"测到的其实全是 `DoNotOptimize` 的开销"。这两个问题用同一招解决：把每个基准
的算式替换成一个**平凡表达式**，保持 `DoNotOptimize` 的次数与实参类型完全同形，
其余一字不动。若耗时几乎不变，原基准测的就是空气；耗时之差才是算式本身的成本。

探针跑在同一二进制里（探针版临时加进 `linear_benchmarks.cpp`，测完已撤回；
下面右列是探针版的实测值，该次运行 `bench_empty` 为 0.471 ns）：

| 基准 | 原样 | 同形平凡探针 | 差值 = 算式本身 | 判断 |
| --- | --- | --- | --- | --- |
| bench_dot | 1.61 ns | 1.38 ns | **0.23 ns** | 算式未被消除，但已被屏障盖住 |
| bench_cross | 1.81 ns | 1.37 ns | 0.44 ns | 同上，接近分辨率下限 |
| bench_subscript | 2.10 ns | 0.978 ns | 1.12 ns | 可测 |
| bench_length | 6.74 ns | 0.944 ns | 5.80 ns | 可测 |
| bench_normalized | 8.53 ns | 0.902 ns | 7.63 ns | 可测 |
| bench_matrix_vector | 3.25 ns | 1.38 ns | 1.87 ns | 可测 |
| bench_matrix_inverse | 51.3 ns | 0.913 ns | 50.4 ns | 可测 |
| bench_quaternion_rotate | 3.08 ns | 1.44 ns | 1.64 ns | 可测 |
| bench_box_contains | 2.03 ns | 1.62 ns | 0.41 ns | 接近分辨率下限 |
| bench_coordinate_to_parent | 2.33 ns | 1.37 ns | 0.96 ns | 可测 |

**没有一个基准的差值为零**，即每一处算式都真的在循环里执行了。

### 4. 直接反证 B：反汇编检查循环体

时序证据之外，直接把基准 TU 编到汇编（`cl /O2 /FAs`，产物在 `build/asm_probe/`，
不进仓库）核对循环体：

- `bench_dot` 的循环里是 **6 次 `movsd` 内存加载 + 3 次 `mulsd` + 2 次 `addsd`**，
  从 `a`、`b` 的栈槽真读真算，最后把结果写进临时槽 —— 没有折叠成常数 `32`。
  `DoNotOptimize(a)`/`DoNotOptimize(b)` 迫使两个向量留在栈上，阻断了常量传播。
- `bench_length` 的循环里是 **3 次 `divsd` + `sqrtpd` + 一次对 `sqrt` 的非内联调用**。
- `bench_matrix_inverse` 的 `inverse()` 没有内联，循环里是**每次迭代一次真实的
  出线调用**（返回 `std::optional<Matrix3>`），不存在被提出循环的迹象。

这解释了差值表的形状：`length`/`normalized` 的 6~9 ns 不是屏障开销，而是
"按最大分量缩放防溢出 + 3 次除法 + sqrt"这个算法本身的成本（见
`Vector3T::length` 的实现）。

### 5. 结论

`bench_empty` 之上每一项都有可归因的真实成本，反证 A、B 双向一致。本基线可
作为后续阶段的回归参考。

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
5. **已知波动**：同一二进制重复运行，各项约 ±5%（例：`bench_dot` 1.61~1.72 ns，
   `bench_matrix_inverse` 51.2~53.0 ns）。跑基线时关注趋势与数量级，别对着末位
   数字做判断。
