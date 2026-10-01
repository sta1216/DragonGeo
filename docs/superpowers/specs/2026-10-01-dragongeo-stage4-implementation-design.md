# DragonGeo 阶段 4 实现方案

日期：2026-10-01
状态：已确认。第 1 步的实现计划见 `docs/superpowers/plans/2026-10-01-dragongeo-stage4-step1.md`。第 2–5 步在第 1 步落地后再写各自的计划。确认之前不写 `Prim` / `Query` 代码的约束已经解除，但只按该计划的任务顺序写。

需求合同仍是 `2026-09-30-dragongeo-stage4-design.md`。本文只决定怎么落地，不改类型语义。两者冲突时，以需求合同为准，并在本文改正。

---

## 1. 选定方案

两波交付。

**波 1** 只放类型壳：数据、工厂、`IsValid`，以及不依赖算法的访问器。头文件和 `double` / `float` 别名一次定下来。

**波 2** 按需求合同第 7 节的五步填曲线协议、转换、检查和 `Query`。每一步带上该步自己的测试。后面的类型还没出现时，不写依赖它的函数。

不采用「先把全部协议声明出来、调用时抛异常」作为默认策略。几何方法只在实现的那一步出现在头文件里。唯一的例外是需求合同已经写明的 NURBS：骨架上不求值就能做的方法随类型一起实现；`PointAt`、`ParameterOf`、`TangentAt`、三个端点、三个切向、`Length`、`Area`、`Orientation`、`ContainsPoint`、`Contains`、`Centroid`、`Subcurve` 必须声明，并调用 `ThrowNotImplemented`。含 NURBS、因而无法判断闭合或无法求值的转换同样抛 `std::logic_error`。

---

## 2. 无虚基类

曲线协议是一组同名方法，不是公共基类。`Query` 是具体类型上的自由函数。`Clone` 按值拷贝。

这样定，是因为返回类型本来就不是同一个类型。`Subcurve` 在一条直线边上是线段，跨过顶点才是折线；圆的子区间是圆弧。矩形在变换后不再轴对齐时，结果是 `Polygon`。这些类型在编译期就确定。公共虚基类会把它们收成基类指针，调用方再向下转换。

`CurveCollection` 存放第 4.1 节的全部二维曲线，但它自己不是一条曲线，没有统一的起点、参数域或子曲线。它用 `std::variant` 按值存放成员。`MultiPolygon` 只用 `variant<Polygon, CurvePolygon>`。阶段 5 的布尔和三角剖分接收 `Prim::Polygon` 这个具体类型。

重复算法放在 `DragonGeo::Detail` 的函数模板里，各具体类型做薄包装。调用点仍然是具体类型，没有虚调用，小函数保持可内联。`Prim` 与解析查询继续 header-only。GJK/EPA 的声明在 `Query` 头里，实现进 `DragonGeo` 静态库。

---

## 3. 效率

- 求交、包含、距离按具体类型重载，编译期选定实现。
- 自交检查本阶段用同一条环上的边两两比较，不写扫描线。`HasSelfIntersection` 只看同一条环。洞是否穿出外环由 `HolesAreValid` 回答。
- 每条曲线的 `Length()` 都返回非负 `Scalar`，无界为 `+inf`，不接收容差。`Ellipse2::Length` 在内部用固定的 `Core::Tolerance{}` 做自适应求积。迭代上限内仍达不到时返回当前估计，并在方法注释里写明上限。
- NURBS 的求值方法与其它曲线使用同一套签名和空值条件。本阶段这些函数体抛 `std::logic_error`，表示定义尚未实现。实现求值后去掉抛异常，签名不变。
- GJK/EPA 的迭代上限和停止条件写在 `src/Query/ConvexDistance.cpp` 内部，不做成调用参数。达到上限时返回当前有限估计，不抛异常。测试只覆盖距离已知、且能在上限内收敛的分离体与简单重叠。
- 参与 GJK 的 `Polygon` 由调用方保证无洞、凸、且至少三个不共线顶点。函数不先扫描拒绝。
- `MultiPolygon` 不检查成员重叠。面积和重心按成员相加，重叠会重复计算。重叠检查留到阶段 5。

---

## 4. 易用性

- 头文件里出现的方法就是当前能用的方法。除第 1 节写明的 NURBS 与相关转换外，不提供一调用就抛异常的几何入口。
- 形状类型仍是 `Scalar` 模板，并提供 `float` 别名，用于存储。用到 `Orient2d` / `Orient3d` 的方法和全部 `Query` 只对 `double` 提供。`float` 几何要查询时，调用方用自己的坐标构造对应的 `double` 类型再调用。本阶段不提供 `float` 到 `double` 的转换函数，库也不做隐式的跨精度查询。
- 不依赖谓词的转换对 `float` 别名同样提供。
- 绕点旋转调用已有的 `Transform2::RotationAbout` / `Transform3::RotationAbout`。点到盒的距离调用已有的 `Box2` / `Box3` 方法，不在 `Prim` 里再写一套。
- 容差参数用 `Core::ToleranceT<Scalar>`。谓词比较不接收容差。
- 不为直线增加「指定另一个原点再变成射线」的重载。要换原点时，调用方先改 `Origin`，再调用 `AsRay`。

---

## 5. 可扩展性

- 一种几何类型一个头文件，由 `Prim.hpp` 汇总。新曲线是新类型加上同一组方法名，不修改已有类型的字段布局。
- `Detail` 里的模板只服务已有的具体类型。不在本阶段做「用户可注册的曲线插件」。
- 阶段 5 只依赖已经稳定的 `Prim::Polygon` 和检查函数，不依赖虚接口。

---

## 6. 波 2 的五步

与需求合同第 7 节相同。每一步结束时，该步新出现的公开方法都有直接调用它的测试。

1. `Segment2/3`、`Ray2/3`、`Line2/3`、`Plane`、`Triangle2/3`，以及它们做得到的曲线协议、`AsRay` / `AsLine` / `AsSegment`。查询覆盖需求合同第 6 节里点名的直线、射线、线段、平面、三角形、`Box`、`OrientedBox` 组合，以及三角形与三角形的距离。
2. `Circle2`、`Arc2`、`Circle3`、`Arc3`、`Ellipse2`、`EllipseArc2`、`Rectangle2`，这些曲线的协议，以及 `AsCircle` / `AsArc` / `AsEllipse` / `AsEllipseArc`。查询：圆与圆、圆与线段、矩形与矩形。
3. `Polyline`、带一层洞的 `Polygon`，二者的曲线协议，`AsPolygon` / `AsPolyline`，`HasSelfIntersection`、`HolesAreValid`、`HasDegenerateEdge`、`Polygon::IsConvex`、`ConvexHull`。
4. `NurbsCurve2/3`、`CurvePolyline`、`CurvePolygon`、`CurveCollection`、扁平的 `MultiPolygon`、`ThrowNotImplemented`，以及曲线折线与曲线多边形的互转和 `ToPolyline` / `ToPolygon`。这一步没有新的几何求交。
5. `Sphere`、`Cylinder`、`Capsule`、`Disk`、`Frustum`，以及它们在需求合同第 6 节中的射线、线段、包含、距离。最后做 GJK/EPA。

波 1 可以按这五步的类型分批合入，不必等全部类型壳都写完再合并。某一批类型进入波 1 时，只提交该批的工厂和 `IsValid`。波 2 的同一步再补协议和查询。不要在波 1 提前声明波 2 的方法。

---

## 7. 测试

沿用仓库规则和需求合同第 8 节。波 1 只断言工厂、`IsValid` 和访问器。波 2 补协议、相交种类、转换和凸包。NURBS 上合同点名要抛异常的方法，测试直接调用并断言抛出 `std::logic_error`。

---

## 8. 本阶段仍然不做

三角剖分、布尔、偏移、简化、曲线离散（除 `ToPolyline` / `ToPolygon`）、SVG、JSON、`Polygon3`、扫描线自交、`MultiPolygon` 重叠检查、用户可扩展的曲线注册表。
