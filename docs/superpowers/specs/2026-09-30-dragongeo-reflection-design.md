# DragonGeo 反射变换

日期：2026-09-30

`Transform2` / `Transform3` 增加反射工厂。总设计 §4.6 与决策 19 指向本文。

## 范围

做两件事：

- 一般反射：过一点、沿一个单位法向。二维是关于一条直线，三维是关于一个平面。法向垂直于这条直线或这个平面，沿法向的分量取反。
- 过原点的便捷工厂。二维保住一根坐标轴，三维保住一个坐标平面。它们是一般工厂在原点、法向取坐标轴时的特化。

`Linear` 仍是头文件。新工厂写在 `Transform2.hpp` / `Transform3.hpp` 里，`constexpr`、`noexcept`，不返回 `optional`，不接收容差。

## 公式

列向量、行主序，与现有 `Transform` 一致。平移在最后一列，最后一行保持 `(0, …, 0, 1)`。

单位法向 `n`、平面（或直线）上一点 `p`：

- 线性部分是 `I − 2 n nᵀ`
- 平移列是 `2 (n · p) n`

这就是 `x' = x − 2 ((x − p) · n) n`。`n` 与 `−n` 得到同一个矩阵。同一镜面上的点（差向量与 `n` 垂直）得到同一个矩阵。

`UnitVector` 的不变量是弱的。调用方交给工厂的法向须是单位向量；长度不对时结果不是等距，工厂不做检查，与 `Transform3::Rotation` 对轴的态度相同。

点走完整仿射（`TransformPoint` / `operator*(Point)`）。方向只走线性部分（`operator*(Vector)`），镜面离原点的平移不改变方向。

## 工厂

| 工厂 | 保住的集合 | 法向 |
|---|---|---|
| `Transform2::Reflection(Point2, UnitVector2)` | 过该点、垂直于法向的直线 | 参数 |
| `Transform2::ReflectionX()` | 过原点的 X 轴 | +Y |
| `Transform2::ReflectionY()` | 过原点的 Y 轴 | +X |
| `Transform3::Reflection(Point3, UnitVector3)` | 过该点、垂直于法向的平面 | 参数 |
| `Transform3::ReflectionYZ()` | 过原点的 YZ 平面 | +X |
| `Transform3::ReflectionZX()` | 过原点的 ZX 平面 | +Y |
| `Transform3::ReflectionXY()` | 过原点的 XY 平面 | +Z |

便捷工厂与一般工厂在上表的法向上相等，并且与一次负缩放相等：

- `ReflectionX()` 等于 `Scaling({1, −1})`，把 `(x, y)` 变成 `(x, −y)`
- `ReflectionY()` 等于 `Scaling({−1, 1})`，把 `(x, y)` 变成 `(−x, y)`
- `ReflectionYZ()` 等于 `Scaling({−1, 1, 1})`
- `ReflectionZX()` 等于 `Scaling({1, −1, 1})`
- `ReflectionXY()` 等于 `Scaling({1, 1, −1})`

名字用 `Reflection`，与 `Translation`、`Rotation`、`Scaling` 同一串。头文件注释用中文，写明法向取反，以及便捷工厂保住的是哪根轴或哪个平面。

## 不变式

- 镜面上的点不动。
- 从镜面上一点出发、沿法向走出的点，落到法向的另一侧、距离相同。
- 线性部分行列式为 −1。
- 反射是对合：再施加上一次回到原处。轴对齐工厂满足 `t * t == Identity()`。一般工厂用点的往返验收，比较带容差。
- `Inverse()` 有值。逆就是这次反射本身：`Inverse()->TransformPoint(p)` 与 `t.TransformPoint(p)` 把同一个点送到同一位置。
- `Coordinate2::FromTransform` / `Coordinate3::FromTransform` 继续拒绝这些变换（正交且行列式为 −1）。右手标架的定义不变。

## 测试

补在 `tests/Linear/Transform2Test.cpp` 与 `tests/Linear/Transform3Test.cpp`，标签沿用 `[linear][transform2]` / `[linear][transform3]`。每个新工厂都有用例。double 别名覆盖行为；工厂是模板，float 不单列一套。

每个维度至少覆盖：

- 便捷工厂的坐标动作，以及与上表负缩放、与一般工厂的相等
- 法向取反得到同一个变换
- 镜面不通过原点时：镜面上的点不动，沿法向的位移被取反；方向与「法向相同、点在原点」的反射一致
- 同一镜面上两个点得到同一个变换
- `t * t` 把点送回；`Inverse()` 同样把点送回
- `Coordinate::FromTransform` 对其中一个反射返回 `nullopt`

## 不做

- 关于一个点的中心对称。二维全轴取负是 180° 旋转；三维全轴取负是点反演，不是单次平面反射。
- 放宽 `Coordinate` 去接受左手系。
- `TransformNormal`。法线的逆转置仍按总设计 §4.6，用到时再算。
- `Box`、`OrientedBox`、线段、多边形上的镜像成员。
