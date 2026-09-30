# 阶段 3 账本

计划：`docs/superpowers/plans/2026-09-30-dragongeo-stage3-predicates.md`

构建日志和一次性探针留在被 gitignore 的 `.superpowers/` 里，不进仓库。这里只保留后面阶段还会用到的决定。

- 四个谓词的符号以 spec §4.5 为准。`Insphere` 用相对第五个点的四项 lift 混合积，不采用列顺序为 `(x, y, z, lift, 1)` 的 5×5 行列式符号，后者正好相反。
- `Incircle` 与 `Insphere` 的永久量取相减之前的各项绝对值。已经相消的绝对值会把误差界缩得过小。
- 无误差变换的中间结果经 `volatile` 写回 `double`。消费方若打开浮点收缩，乘加融合会丢掉 `TwoProduct` 的误差项。在 MSVC `/fp:fast /arch:AVX2` 下，收缩前误差项为 0，写回后为 `2^-104`。
- 展开用 `std::vector`，不设固定上限。精确路径可能分配内存；耗尽时按 `noexcept` 终止。
- 非有限坐标，以及指数跨度大到中间积下溢成次正规数时，返回值不作规定。
- `ScaleShift` 的测试名不能含未闭合的 `[`，否则 Catch2 发现脚本会吞掉下一条测试。
- 阶段 2 在 spec §8 标成已完成，阶段 3 为当前阶段。
- 实现完成后改为编译进静态库：公共头只留四个声明，展开头放在 `src/Predicates/`，不随安装包发布。`DragonGeo` 从 `INTERFACE` 改为 `STATIC`。
- `Orient3d` 与 `Incircle` 各补了偏离一个 ulp、以及幅度 `2^50` 的用例。这两组都会让过滤弃权，符号由精确路径给出。
