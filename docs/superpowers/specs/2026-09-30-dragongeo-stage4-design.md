# DragonGeo 阶段 4 设计

日期：2026-09-30
状态：本轮需求已于 2026-10-01 冻结。实现方案尚未讨论，因此还不写实现计划，也不写代码。第 10 节是冻结时的建议，还没有全部并入前面的合同；讨论实现方案时先处理那一节。

本文是 `2026-09-29-dragongeo-design.md` 的阶段 4 合同。类型清单以那份文档的 §5.1、§5.2、§5.6 为准。本文补上字段、构造、不变量、查询结果，以及分步交付的边界。两者若有冲突，在本文被确认之前以本文的细化说明为准，确认后回写总览。

---

## 1. 需要确认的裁决

下面这些选择会决定后续每一步的接口。其余章节是它们的展开。

1. **形状类型放在 `DragonGeo::Prim`。** 直线多边形的类型是 `Prim::Polygon`。阶段 5 的布尔、三角剖分仍放在 `DragonGeo::Polygon` 这个算法命名空间里，接收 `Prim::Polygon`。总览 §3.4 里的例子 `DragonGeo::Polygon::Polygon` 不用于这个类型。
2. **`Rectangle2` 轴对齐。** 一个角点加上沿世界 +X、+Y 的正宽与正高。旋转后的矩形继续用已有的 `OrientedBox2`，本阶段不另造旋转矩形。
3. **带不变量的类型用私有数据和返回 `std::optional` 的工厂。** 半径非正、弧的扫掠角越界、折线不连续、NURBS 结构不合法，都得到 `std::nullopt`，不抛异常。`Segment`、`Ray`、`Line`、`Plane`、`Triangle` 没有额外不变量，保持聚合，公开字段。
4. **角度用弧度。** 二维从世界 +X 起、逆时针为正。三维圆弧在平面内从给定的零角方向起，绕法向按右手定则增加。扫掠角可以为负，表示顺时针。圆弧与椭圆弧要求 `0 < |Sweep| < 2π`。整圆是 `Circle2` / `Circle3`，不是扫掠角为 `2π` 的弧。
5. **椭圆参数是离心角，不是弧长角。** 椭圆弧的两点构造只生成逆时针、落在 `(0, 2π)` 内的扫掠。顺时针弧用起始角加负扫掠角构造。
6. **相等是表示相等。** `operator==` 逐字段精确比较。它不判断“是不是同一条直线”。直线上的基点不同，两个 `Line2` 就不相等。
7. **单形状上的问题写成方法，两个形状之间的问题写成 `Query` 自由函数。** 点到线段的距离是 `Segment2::Distance`。两条线段是否相交是 `Query::Intersects`。已有的 `Box::Contains`、`OrientedBox::Contains` 保持不动，不再包一层自由函数。
8. **`Intersects` 按闭集回答，并且该用谓词的地方用精确谓词。** 交点坐标是普通 `double` 运算，不保证正好落在两端对象上。重合、共线重叠、共面重叠用结果里的种类表示，不伪装成一个点。
9. **`Polyline` 存放 `Segment2` 序列。** 连续指后一段的 `A` 与前一段的 `B` 精确相等，不容差。调用方要传递同一个点值。`Polygon` 的每一环存放顶点，不把首顶点在末尾再存一次；最后一条边由类型闭合。
10. **曲线协议是一组同名方法，不是公共基类。** 第 4.1 节列出全部二维、三维曲线。`Query` 继续不用虚函数。`Clone` 就是按值拷贝。
11. **洞在 `Polygon` 和 `CurvePolygon` 上，只有一层。** 一个外环加零个或多个洞环。洞自己不再带洞。洞里的岛是 `MultiPolygon` 里的另一个多边形。`MultiPolygon` 不嵌套，只按顺序存放 `Polygon` 与 `CurvePolygon`。构造时不拒绝自交，也不拒绝落在外面的洞。这些情况由第 4.2 节的检查函数报告。
12. **凸包点数不够时没有多边形。** `ConvexHull` 在不共线的点少于 3 个时返回 `std::nullopt`。结果是严格凸的、逆时针的 `Polygon`，边上的共线点不保留。
13. **查询只提供 `double`。** 形状类型仍是 `Scalar` 模板，并提供 `float` 别名，以便存储。用到 `Orient2d` 等谓词的方法只在 `Scalar` 为 `double` 时存在。
14. **有限圆柱是带平底的实心体。** 轴是两端盖圆心之间的 `Segment3`，零长度轴构造失败。胶囊允许零长度轴，此时它是一个球。`Disk` 是三维里的填充圆盘（曲面），不列入体积包含。
15. **`Frustum` 是直角截头棱锥。** 近、远两个矩形互相平行，由一个 `Coordinate3` 加上深度和两组半宽半高决定。不含斜视锥。
16. **GJK/EPA 编译进静态库。** 声明在头文件。二维只接受 `Box2`、`OrientedBox2`、凸的 `Polygon`。三维只接受 `Box3`、`OrientedBox3`、`Sphere`、`Capsule`、`Frustum`。两端维度必须相同。参与 GJK 的 `Polygon` 必须没有洞、必须凸，且至少有三个不共线顶点。不满足时结果不作规定，函数不先检查。
17. **中点、切向和子曲线都按参数，不按弧长。** 中点是参数区间的中点。椭圆上它一般不是弧长中点。
18. **`Transformed` 不改变具体类型，矩形是例外。** 圆和圆弧只接受相似变换：平移、旋转、镜像、均匀缩放。非均匀缩放返回 `std::nullopt`。要得到椭圆时，调用方从椭圆类型出发。椭圆接受可逆仿射，结果仍是椭圆。线段和三角形接受任意仿射，三角形结果仍是三角形。变换把方向缩成零向量，或结果非有限时，返回 `std::nullopt`。矩形在结果仍然轴对齐且面积为正时保持 `Rectangle2`，否则 `Transformed`、`Rotated`、`Mirrored` 返回 `Polygon`。
19. **`Rectangle2`、`Triangle2`、`Triangle3` 是没有洞的闭合多边形。** 它们实现第 4.1 节。子曲线看参数区间落在边界的哪一段：落在同一条直线边上时是线段，跨过顶点时才是折线。三维三角形的折线类型是 `Polyline3`。
20. **`CurveCollection` 不是一条曲线。** 它按顺序存放第 4.1 节里的全部二维曲线，含 `Polygon`、`CurvePolygon`、`Rectangle2`、`Triangle2`、射线和直线。它只实现集合上有定义的那部分协议：包围盒、长度、面积、包含、重心、移动、镜像、旋转、`Transform`、反向、克隆、是否有效。没有单一的起点、切向、参数域或子曲线。
21. **能唯一确定的转换直接返回目标类型，缺了范围就由调用方补上。** 线段可以变成射线或直线。射线和直线变成线段时必须给出有限的参数范围。圆变成圆弧、椭圆变成椭圆弧时必须给出起始角和扫掠角。
22. **多边形的几何检查不改变对象。** `IsValid` 仍只看结构：有限坐标、环的顶点数、连续性。自交和洞是否有效由单独的函数回答。检查失败的多边形仍然可以求包含和面积。

---

## 2. 范围

阶段 4 交付 §5.1 的原语、§5.6 的类型关系，以及 §5.2 里标记为 ★ 的查询。交付顺序见第 7 节。后一步开始时，前一步的类型和查询已经可用。

闭合曲线的面积、方向、重心、长度，以及曲线上的移动、镜像、旋转和 `Transform`，都在本阶段跟对应类型一起交付。自交和洞的有效性检查也在本阶段，见第 4.2 节。阶段 5 才做：三角剖分、布尔、偏移、简化。曲线与曲线的求交只做第 6 节列出的那些。曲线离散里，只有 `CurvePolyline` / `CurvePolygon` 按给定偏差退化为折线或多边形；其它离散、SVG、JSON 不在本阶段。

`Plane`，以及球、圆柱、胶囊、圆盘、视锥不是曲线，不实现第 4.1 节。`MultiPolygon` 也不是一条曲线；它只存放 `Polygon` 与 `CurvePolygon`。`CurveCollection` 则存放第 4.1 节的全部二维曲线，并只实现第 4.1 节末尾列出的集合操作。本阶段没有 `Polygon3`。`Polyline3` 只用来表达 `Triangle3` 的子曲线，不能变成多边形。第 7 节是依赖顺序，供实现方案讨论使用，本身还不是开工计划。

---

## 3. 分层与文件

```
include/DragonGeo/Prim/          一个类型一个头，另有 Prim.hpp 汇总
include/DragonGeo/Query/         自由函数与结果类型
include/DragonGeo/Detail/NotImplemented.hpp
src/Query/ConvexDistance.cpp     GJK/EPA，模式与 Predicates 相同
```

`Prim` 与解析查询是 header-only。GJK/EPA 的声明在 `Query` 头里，实现进入已有的 `DragonGeo` 静态库。`DragonGeo.hpp` 在本阶段结束时包含 `Prim.hpp` 与 `Query.hpp`。

命名空间是 `DragonGeo::Prim` 与 `DragonGeo::Query`。未实现入口是 `DragonGeo::Detail::ThrowNotImplemented(const char* function, const char* reason)`，抛 `std::logic_error`，消息含函数名与原因。总览 §6.1 草稿里的 `throw_not_implemented` 是改名之前的写法，新代码不用它。

每个形状模板都提供双精度与单精度别名，例如 `using Segment2 = Segment2T<double>`、`using Segment2f = Segment2T<float>`。查询函数不是模板，参数用双精度别名。

---

## 4. 共用约定

**参数。** 线段 `PointAt(t) = (1 - t) * A + t * B`，`t` 在 `[0, 1]`。射线与直线都带单位方向：`Origin + t * Direction`。`t` 是沿方向的有符号距离。射线上的点要求 `t ≥ 0`。

**退化。** 零长度线段允许存在。零面积三角形允许存在。它们的长度、距离、包含按下面各节的定义给出结果，不在构造时拒绝。

**非有限值。** 工厂遇到非有限坐标、半径或角度时返回 `std::nullopt`。聚合类型可以含有 `NaN` 或无穷，与 `Point` 相同。查询碰到非有限输入时不抛异常，返回值不作规定，与谓词相同。

**空盒。** `Box2` / `Box3` 已有包含与相交。本阶段只补 `ClosestPoint`、`Distance`、`DistanceSquared`。空盒没有最近点，三个方法都返回 `std::nullopt`。非空盒对任何有限点都有最近点：逐轴夹紧到 `[Min, Max]`。

**距离。** 几何距离非负。同时提供 `DistanceSquared`，供精确比较，避免先开方再平方。`Plane::SignedDistance` 是 `Normal · (Point - Origin)`，可正可负。`Plane::Distance` 是它的绝对值。

**包含。** 区域包含把边界算在内部。`Polygon::Contains` 先对外环用非零环绕数，再排除严格落在某个洞内部的点。外环边界和洞的边界都算内部。

**谓词与容差。** 线段相交的布尔结果、点在三角形内、点在多边形内、凸包的转向，都走 `Orient2d` / `Orient3d`，不接收容差。构造坐标系那种“是否垂直、是否共圆”的数值检查才接收 `Tolerance`，默认 `{}`。本阶段需要容差的工厂只有 `Arc3` 的零角方向与法向垂直，以及椭圆弧的两点构造。点是否在曲线轨迹上另见第 4.1 节，那里使用 `Tolerance`。

### 4.1 曲线统一能力

下列类型实现同一组方法。二维：`Segment2`、`Ray2`、`Line2`、`Circle2`、`Arc2`、`Ellipse2`、`EllipseArc2`、`NurbsCurve2`、`Polyline`、`CurvePolyline`、`Polygon`、`CurvePolygon`、`Rectangle2`、`Triangle2`。三维：`Segment3`、`Ray3`、`Line3`、`Circle3`、`Arc3`、`NurbsCurve3`、`Triangle3`。

`Polygon` 与 `CurvePolygon` 的点、切向、参数和子曲线都指外环。面积、包含、重心和方向把洞算进去。

| 能力 | 方法 | 结果 |
|---|---|---|
| 起点、终点、中点 | `StartPoint`、`EndPoint`、`MidPoint` | `std::optional<Point>`。无界端没有点 |
| 起点、终点、中点切向 | `StartTangent`、`EndTangent`、`MidTangent` | `std::optional<UnitVector>`。导数为零或该端不存在时为空 |
| 是否闭合 | `IsClosed` | 闭合曲线的起点与终点是同一点 |
| 长度 | `Length` | 非负。无界曲线为 `+inf`。椭圆周长见下文 |
| 面积 | `Area` | `std::optional<Scalar>`。非闭合为空。闭合时是填充面积，非负；有洞时减去洞的面积 |
| 方向 | `Orientation` | `std::optional<int>`。非闭合为空。逆时针 `+1`，顺时针 `-1`，退化 `0` |
| 参数区间 | `Domain` | `Interval` |
| 参数位置的坐标 | `PointAt` | 参数不在 `Domain` 内时返回 `std::nullopt` |
| 轨迹上一点的参数 | `ParameterOf(point, tolerance = {})` | `std::optional<Scalar>`。先在轨迹上找最近点，距离超过 `ContainsPoint` 的同一容差时为空。有多个参数时取 `Domain` 内最小的一个。闭合曲线的接缝点返回 `Domain` 的下端，不返回上端 |
| 参数位置的切向 | `TangentAt` | 同样越界为空；导数为零时为空 |
| 点在曲线轨迹上 | `ContainsPoint(point, tolerance = {})` | 点到曲线的距离不超过 `tolerance.Resolve(尺度)`。尺度取曲线的特征长度：半径、半轴或包围盒对角线 |
| 点在填充区域内 | `Contains` | 非闭合曲线恒为 false。闭合曲线包含边界。圆是圆盘，椭圆是椭圆盘，三维圆是其所在平面上的圆盘 |
| 包围盒 | `Bounds` | 对应维度的 `Box`。无界曲线中没有有限包围盒的，返回规范空盒 |
| 重心 | `Centroid` | `std::optional<Point>`。非闭合，或闭合但面积为 0 时为空。有洞时按填充区域计算 |
| 移动 | `Translated(vector)` | 同类型 |
| 镜像 | `Mirrored(point, unitNormal)` | 同类型。法向取已有反射工厂的约定：二维过点沿单位法向，三维同样 |
| 旋转 | `Rotated(...)` | 二维绕点旋转，参数是中心与弧度。三维绕轴旋转，参数是轴上一点、单位轴与弧度 |
| 变换 | `Transformed(transform)` | `std::optional<同类型>`，规则见裁决 18 |
| 反向 | `Reversed` | 同类型。参数方向相反，起点与终点对调。闭合曲线的 `Orientation` 变号，接缝上的点保持不变 |
| 子曲线 | `Subcurve(interval)` | 区间必须落在 `Domain` 内且非空，否则 `std::nullopt`。结果类型见下表 |
| 克隆 | `Clone` | 按值返回同类型的副本 |
| 是否有效 | `IsValid` | 有限坐标、类型自身的不变量、折线连续、多边形每环至少三个顶点。零长度线段有效。公开字段被写成 `NaN` 的聚合无效 |

参数域：

- 线段是 `[0, 1]`。`PointAt(t) = (1 - t) * A + t * B`。
- 折线与多边形外环把每一段算作参数长度 1。`n` 段的域是 `[0, n]`。多边形在 `0` 与 `n` 处是同一顶点。
- 矩形是四条边，域为 `[0, 4]`，默认从 `Origin` 起按逆时针。三角形是三条边，域为 `[0, 3]`，顺序是 `A`、`B`、`C`。二者在域的两端是同一顶点。参数中点落在边界上，不是填充区域的重心。
- 圆是 `[0, 2π]`。参数是从接缝起、沿当前方向走过的角度。默认接缝在局部 +X，方向为逆时针。
- 圆弧与椭圆弧是 `[0, |Sweep|]`。参数 0 在起点，增加方向就是扫掠方向。
- 射线是 `[0, +inf)`。直线是无界区间。二者都没有中点，直线也没有起点和终点。
- NURBS 使用节点域 `[knots[Degree], knots[ControlCount]]`。

中点是 `PointAt` 在有限参数域中点处的值。它不是弧长中点。

子曲线的结果由原曲线和参数区间共同决定。区间必须落在 `Domain` 内、非空，并且结果类型能够表示这段轨迹，否则返回 `std::nullopt`。

| 原曲线 | 子曲线 |
|---|---|
| `Line`、`Ray`、`Segment` | `Segment`。区间必须有限。射线和直线的整个无界参数域不能表示成线段，返回空 |
| `Circle`、`Arc` | `Arc`。圆上的区间长度必须大于 0 且小于 `2π`。整周期不是圆弧，返回空 |
| `Ellipse`、`EllipseArc` | `EllipseArc`。椭圆上的区间同样必须短于整周期 |
| `NurbsCurve` | 同维度的 `NurbsCurve`。本阶段抛 `std::logic_error` |
| `Polyline` | 区间落在同一段上时是 `Segment`，跨过接头时是 `Polyline` |
| `CurvePolyline` | 区间落在同一段上时，按该段自己的规则返回 `Segment`、`Arc`、`EllipseArc` 或 `NurbsCurve`。跨过接头时是 `CurvePolyline` |
| `Polygon`、`CurvePolygon` | 只取外环，不带洞。直线环与 `Polyline` 同一规则，曲线环与 `CurvePolyline` 同一规则。整圈外环是一条闭合折线 |
| `Rectangle2`、`Triangle2` | 与 `Polyline` 同一规则。一条边上是 `Segment`，跨过顶点是 `Polyline` |
| `Triangle3` | 与上面相同，三维类型是 `Segment3` 与 `Polyline3` |

同一原曲线可能对应两种结果时，`Subcurve` 返回 `std::optional<std::variant<...>>`，里面只放该行写出的那两种类型。只有一种结果时，返回 `std::optional<该类型>`。

椭圆周长没有初等闭式。`Ellipse2::Length(Tolerance tolerance = {})` 对周长积分做自适应求积，直到误差估计不超过 `tolerance.Resolve(SemiAxisX + SemiAxisY)`。达不到容差时是否返回空，见第 10 节，实现方案讨论时确定。其余有闭式的曲线，`Length()` 不接收容差。圆的面积是 `π r²`，椭圆盘的面积是 `π * SemiAxisX * SemiAxisY`。多边形面积是外环鞋带公式的绝对值减去各洞的面积。

`ContainsPoint` 对直线和线段使用点到直线的距离，线段还要求参数落在 `[0, 1]` 的容差扩张之内。扩张量是 `tolerance.Resolve(1)`。区域上的 `Contains` 仍是精确比较，不接收容差。

NURBS 骨架上，不求值也能实现的方法现在就实现：`Domain`、`Bounds`（全部控制点的包围盒）、`IsValid`、`Clone`、`Reversed`（反转控制点、权重，并镜像节点）、`Translated`、`Rotated`、`Mirrored`、`Transformed`（变换控制点；不可逆时返回空）。`IsClosed` 在本阶段恒为 false，因为周期性要求求值或一套尚未定义的周期节点约定。`PointAt`、`ParameterOf`、`TangentAt`、三个端点、三个切向、`Length`、`Area`、`Orientation`、`ContainsPoint`、`Contains`、`Centroid`、`Subcurve` 调用 `ThrowNotImplemented`。

`CurveCollection` 按成员逐个转发能定义的操作：

| 方法 | 集合上的含义 |
|---|---|
| `Bounds` | 合并各成员的包围盒。空集合，或任一成员没有有限包围盒时，返回规范空盒 |
| `Length` | 各成员长度之和。有无界成员时为 `+inf`。成员抛异常时，集合也抛 |
| `Area` | 闭合成员的面积之和。没有闭合成员时为空。重叠会重复计算 |
| `Contains` | 任一成员包含该点 |
| `ContainsPoint` | 任一点在某成员的轨迹上 |
| `Centroid` | 按面积加权的闭合成员重心。总面积为 0 时为空。重叠会重复计算 |
| `Translated`、`Mirrored`、`Rotated` | 逐个作用，结果仍是 `CurveCollection` |
| `Transformed` | 逐个作用。成员自己的结果若换成了另一种二维曲线，集合收下那种曲线。任一成员返回空，则集合返回空 |
| `Reversed` | 逐个反向，成员顺序不变 |
| `Clone`、`IsValid` | 拷贝；每个成员都有效时集合才有效 |

集合不提供 `StartPoint`、`EndPoint`、`MidPoint`、三处切向、`IsClosed`、`Orientation`、`Domain`、`PointAt`、`ParameterOf`、`TangentAt`、`Subcurve`。这些都要求一条连续曲线。

### 4.2 类型转换与多边形检查

转换失败时返回 `std::nullopt`，不改原对象。含 NURBS、因而无法判断闭合或无法求值的转换抛 `std::logic_error`。不依赖谓词的转换对 `float` 别名同样提供。下面的多边形检查只对 `double` 提供。

**直线、射线、线段。** 二维与三维同一规则。

| 方法 | 结果 |
|---|---|
| `Segment::AsRay`、`Segment::AsLine` | 起点是 `A`，方向从 `A` 指向 `B`。`A == B` 时为空 |
| `Ray::AsLine` | 原点和方向原样带走 |
| `Line::AsRay` | 用这条直线自己存放的原点和方向。这是表示上的选择，换一个基点会得到另一条射线 |
| `Ray::AsSegment(length)`、`Line::AsSegment(parameterStart, parameterEnd)` | 结果必须是有限且长度不为 0 的线段。`length` 要有限且大于 0。直线的两个参数都要有限，并且不能相等。参数的先后决定线段方向 |

**圆与圆弧，椭圆与椭圆弧。** 二维与三维的圆使用同一套名字。

| 方法 | 结果 |
|---|---|
| `Arc::AsCircle`、`EllipseArc::AsEllipse` | 去掉起止角，得到支撑曲线。二维方向取 `+1`。三维圆保留圆心、法向和半径 |
| `Circle2::AsArc(startAngle, sweep)`、`Ellipse2::AsEllipseArc(startAngle, sweep)` | 起始角和扫掠角必须构成合法的弧，否则为空。角度约定与对应弧类型相同 |
| `Circle3::AsArc(zeroDirection, startAngle, sweep, tolerance = {})` | 与 `Arc3` 的工厂使用同一套垂直检查 |

**折线与多边形，曲线折线与曲线多边形。**

| 方法 | 结果 |
|---|---|
| `Polyline::AsPolygon`、`CurvePolyline::AsCurvePolygon` | 必须闭合，并且至少能形成三个顶点。结果没有洞。不闭合或顶点不够时为空 |
| `Polygon::AsPolyline`、`CurvePolygon::AsCurvePolyline` | 只在没有洞时成功，结果是外环构成的闭合折线。有洞时为空 |
| `Polygon` 的外环和每一个洞、`CurvePolygon` 的每一环 | `AsPolyline` 或 `AsCurvePolyline` 始终可以取出这一环，不受其它环影响 |

**按偏差退化。** `CurvePolyline::ToPolyline(deviation)` 得到 `Polyline`。`CurvePolygon::ToPolygon(deviation)` 得到 `Polygon`，洞逐环保留。`deviation` 是绝对距离：原曲线上每一点到结果折线的距离不超过它。圆弧和椭圆弧用弓高判断，弓高不超过 `deviation` 即满足。已有的直线段原样保留，不加点，也不删掉调用方放进来的顶点。它不是 `Tolerance` 的相对项。`deviation` 必须有限且大于 0，否则为空。任一环退化后不足三个顶点时，`ToPolygon` 为空。环上有 NURBS 段时抛 `std::logic_error`。

**多边形检查。** `Polygon` 与 `CurvePolygon` 提供下面三个函数。构造函数不调用它们。`CurvePolygon` 的环上有 NURBS 段时，三个函数都抛 `std::logic_error`。只对 `double` 提供，因为判断走精确谓词。

| 方法 | 为真的条件 |
|---|---|
| `HasSelfIntersection` | 同一条环里，两条不相邻的边在内部相交、有正长度重叠，或一个顶点落在另一条不相邻边的内部。相邻边可以共顶点，也可以共线。外环和洞各自检查，环与环之间不算自交 |
| `HolesAreValid` | 没有洞时为真。每个洞的顶点都在外环内部或边界上，内外的判断与 `Contains` 相同，外环自交时也按这个规则计算。洞的边不穿越外环或其它洞，也不与它们正长度重叠；只在顶点相接是允许的。一个洞的内部不包含另一个洞。环的顺逆时针不参与这项判断 |
| `HasDegenerateEdge` | 某一环上有零长度边 |

三角形只有三条边，每两条都相邻，所以 `Triangle2` / `Triangle3` 不另报自交。矩形的四条边只在顶点相接。这两个类型没有洞，不提供 `HolesAreValid`。

---

## 5. 类型

### 5.1 直线、射线、线段、平面、三角形

这些是聚合。字段如下。

| 类型 | 字段 |
|---|---|
| `Segment2` / `Segment3` | `A`、`B`，两个点 |
| `Ray2` / `Ray3` | `Origin`，`Direction`（单位向量） |
| `Line2` / `Line3` | `Origin`，`Direction`（单位向量） |
| `Plane` | `Origin`，`Normal`（单位向量） |
| `Triangle2` / `Triangle3` | `A`、`B`、`C` |

线段、射线、直线实现第 4.1 节。线段另外有 `LengthSquared`、`Direction`（`A == B` 时返回 `std::nullopt`）、`Distance`、`DistanceSquared`、`ClosestPoint`。最近点把参数夹在 `[0, 1]`。射线与直线另外有 `Distance`、`DistanceSquared`、`ClosestPoint`。射线把参数夹在 `t ≥ 0`，直线不夹。`LengthSquared` 只属于线段：无界曲线的长度平方没有有限值。

平面的方法：`SignedDistance`、`Distance`、`ClosestPoint`、`Flipped`（法向取反，原点不变）。`Flipped` 与原平面表示不相等，点集相同。

`Triangle2` 与 `Triangle3` 是没有洞的闭合多边形，实现第 4.1 节。另外有 `SignedArea`（逆时针为正，零面积为 0）。`Contains` 仅 `double`，边界算内部。零面积时，点在某条退化边上才算包含，退化边用精确共线加参数范围判断。`Triangle3::Contains` 还要求 `Orient3d` 为 0；不共面就是不包含。`Subcurve` 落在同一条边上时返回 `Segment`，跨过顶点时返回折线：二维为 `Polyline`，三维为 `Polyline3`。`Transformed` 与 `Rotated` 的结果仍是三角形。

`Polyline3` 是 `Segment3` 的连续序列，规则与 `Polyline` 相同，并实现第 4.1 节里三维曲线做得到的部分。它主要作为 `Triangle3` 的子曲线，不另造一套查询。

`Plane::Flipped`、线段的 `Bounds`、三角形的 `SignedArea` 与 `Contains` 都属于对应类型的公开方法，测试要直接调用。

### 5.2 圆、圆弧、椭圆、椭圆弧、矩形

`Circle2` 的数据是圆心、半径，以及参数方向 `+1` 或 `-1`。半径必须有限且大于 0。默认方向 `+1`，接缝在世界 +X。访问器是 `Center`、`Radius`。它始终闭合。`Contains` 是闭圆盘。`Reversed` 只翻转参数方向。

`Arc2` 在同一组圆心、半径上增加 `StartAngle` 与 `Sweep`。`StartAngle` 保留调用方传入的值，不折进 `[0, 2π)`，这样表示相等可预期。`IsClosed` 为 false。`AsCircle` 去掉角度后得到方向为 `+1` 的整圆。

`Circle3` 的数据是圆心、单位法向、半径。法向同时是参数方向：逆着法向看去，参数增加为逆时针。`Reversed` 翻转法向。`Contains` 是该平面上的闭圆盘，不共面的点不包含。`Arc3` 再增加与法向垂直的 `ZeroDirection`、`StartAngle`、`Sweep`。工厂 `Arc3::FromCircle(circle, zeroDirection, startAngle, sweep, tolerance = {})` 在 `zeroDirection · normal` 不能视为 0 时返回 `std::nullopt`。`Arc3` 不闭合。

`Ellipse2` 的数据是中心、`SemiAxisX`、`SemiAxisY`、`Rotation`，以及参数方向 `+1` 或 `-1`。两个半轴都有限且大于 0。`Rotation` 是局部 X 轴相对世界 +X 的逆时针角。半轴相等时它仍是椭圆，不是圆。参数是离心角：局部坐标 `(SemiAxisX cos θ, SemiAxisY sin θ)`，再旋转和平移。它始终闭合。`Contains` 是闭椭圆盘。

`EllipseArc2` 在椭圆上增加 `StartAngle` 与 `Sweep`，规则与 `Arc2` 相同。`FromPoints(ellipse, start, end, tolerance = {})` 要求两点都在椭圆上；扫掠取从起点到终点的逆时针离心角，结果落在 `(0, 2π)`。两点对应同一离心角时返回 `std::nullopt`。

`Rectangle2` 的数据是角点 `Origin`、`Width`、`Height`，以及参数方向 `+1` 或 `-1`。宽与高有限且大于 0。默认方向 `+1`。四个顶点按该方向为 `Origin`、`Origin + (Width, 0)`、`Origin + (Width, Height)`、`Origin + (0, Height)`；`+1` 时这是逆时针。它是没有洞的闭合多边形，实现第 4.1 节。一条边上的 `Subcurve` 返回 `Segment2`，跨过顶点时返回 `Polyline`。`FromCorners(a, b)` 与角点顺序无关，原点取分量最小值，方向为 `+1`；零面积或非有限时返回 `std::nullopt`。没有空矩形，也没有 `Merged`。平移结果仍是 `Rectangle2`。`Rotated`、`Mirrored`、`Transformed` 的返回类型是 `std::optional<std::variant<Rectangle2, Polygon>>`。四个像点仍然构成轴对齐正面积矩形时，变体里是 `Rectangle2`，否则是 `Polygon`。像点非有限或面积不为正时，返回空。

### 5.3 折线与多边形

`Polyline::FromSegments(span<const Segment2>)` 至少要有一段，并且 `segments[i].B == segments[i + 1].A`。失败返回 `std::nullopt`。方法有 `SegmentCount`、`Segment(index)`、`PointCount`、`Point(index)`，以及第 4.1 节。不要求闭合。首段起点与末段终点是同一个点值时 `IsClosed` 为 true，此时面积和方向按这一环计算。闭合的折线不会自动变成 `Polygon`。要得到多边形时调用 `AsPolygon`。

`Polygon` 有一个外环和零个或多个洞环。`FromVertices(span<const Point2>)` 构造没有洞的多边形，至少三个顶点。`FromRings(outer, span<const Ring>)` 增加洞。`Ring` 就是顶点序列，规则与外环相同。若某一环的最后一个顶点与第一个相等，返回 `std::nullopt`，避免闭合边有两种写法。连续重复顶点允许存在，表示零长度边。自交允许存在，构造时不检查，也不检查洞是否在外环内部，也不检查洞与洞是否重叠。这些检查是 `HasSelfIntersection`、`HolesAreValid` 和 `HasDegenerateEdge`，见第 4.2 节。方法还有 `VertexCount`、`Vertex`、`Edge`（都指外环；最后一条边连接末顶点与首顶点）、`HoleCount`、`Hole(index)`，以及第 4.1 节。`IsConvex` 仅 `double`。有洞时 `IsConvex` 为 false。

`IsConvex` 在无洞时看外环每个顶点的转向。顶点 `i` 的转向是 `Orient2d(Vertex(i - 1), Vertex(i), Vertex(i + 1))`，下标按顶点数循环。全部转向 `≥ 0`，或全部 `≤ 0`，即为凸。共线转向（0）可以混在其中。三点共线的退化多边形因此是凸的。它不另外查找自交。

`Contains` 对退化外环仍有定义：不在边界上的点环绕数为 0；边界上的点算内部。严格落在洞内的点不算内部。

`Reversed` 反转外环顶点顺序，并反转每一个洞的顶点顺序。洞仍然是洞，不升成外环。

### 5.4 曲线段、曲线串、集合、区域

曲线段没有新的几何类型。别名：

```cpp
using CurveSegment2 = std::variant<Segment2, Arc2, EllipseArc2, NurbsCurve2>;
```

`CurvePolyline::FromSegments` 至少一段。相邻解析段必须精确相接。某一侧是 `NurbsCurve2` 的接头不检查。方法有 `SegmentCount`、`Segment(index)`，以及第 4.1 节。整条折线的点、切向、长度、面积在遇到 NURBS 段时抛 `std::logic_error`。`Bounds` 合并各段包围盒；NURBS 段用其控制点包围盒。

`CurvePolygon` 有一个外环和零个或多个洞环，每环都是闭合的 `CurveSegment2` 序列。解析段必须首尾相接。含 NURBS 段的环在构造时接受，几何闭合留到求值实现之后。洞的规则与 `Polygon` 相同：只有一层，构造时不检验包含关系。`Reversed` 反转每一环的参数方向。依赖求值的曲线方法在环上含有 NURBS 段时抛 `std::logic_error`。

`CurveCollection` 按插入顺序存放第 4.1 节的全部二维曲线：`Segment2`、`Ray2`、`Line2`、`Circle2`、`Arc2`、`Ellipse2`、`EllipseArc2`、`NurbsCurve2`、`Polyline`、`CurvePolyline`、`Polygon`、`CurvePolygon`、`Rectangle2`、`Triangle2`。别名 `CurveItem2` 是这个 `variant`。重复成员保留。相等按顺序与表示比较。不检查相连，也不建立空间索引。三维曲线不放进来。它实现第 4.1 节末尾的集合操作，不实现单条曲线才有的参数方法。`MultiPolygon` 仍然只表示区域列表，不替代这个集合。

`MultiPolygon` 按插入顺序存放 `Polygon` 与 `CurvePolygon`。可以是空列表。没有嵌套，没有第三种成员。圆盘、椭圆盘、三角形和矩形要放进来时，调用方先写成 `Polygon` 或 `CurvePolygon`。成员之间的重叠不检查。`Contains` 在任一成员包含该点时为真。`Area` 是成员面积之和，重叠会重复计算。`Bounds` 是成员包围盒的合并。移动、镜像、旋转和变换逐个作用到成员上。它不实现起点、切向和参数。

### 5.5 NURBS 骨架

`NurbsCurve2` 与 `NurbsCurve3` 的工厂接收次数、控制点、节点、权重。

- 次数 `≥ 1`。
- 控制点个数 `≥ 次数 + 1`，全部分量有限。
- 节点个数 `= 控制点数 + 次数 + 1`，有限且非降。允许重节点。不要求钳制。
- 权重个数与控制点相同，每个权重有限且 `> 0`。全 1 表示多项式 B 样条，调用方显式传入。

失败返回 `std::nullopt`。访问器：`Degree`、`ControlCount`、`Control`、`Knot`、`Weight`。曲线协议的哪些方法现在实现、哪些抛异常，见第 4.1 节。`Evaluate`、`Derivative`、`Split` 是 `PointAt`、`TangentAt`、`Subcurve` 的旧称，不另设入口。

### 5.6 球、圆柱、胶囊、圆盘、视锥

`Sphere` 是实心球：球心与有限正半径。`Contains` 为闭球。射线交在球面上。

`Cylinder` 是有限实心圆柱，两端为平面圆盖。数据是轴 `Segment3` 与半径。轴的两个端点必须不同，否则返回 `std::nullopt`。`Contains` 含侧面与两个盖。

`Capsule` 是到一条线段的距离不超过半径的点集。轴段允许零长度，这时点集是球，类型仍是 `Capsule`。`Contains` 含两端半球。

`Disk` 是三维填充圆盘：圆心、单位法向、有限正半径。它是平面区域，不是体积。本阶段的查询只有射线与圆盘的相交，没有“点在圆盘内”的体积包含。点是否在圆盘上由相交结果表达。

`Frustum` 的工厂是 `FromFrame(frame, depth, nearHalfWidth, nearHalfHeight, farHalfWidth, farHalfHeight)`。`frame` 的原点在近矩形中心，Z 轴指向远矩形，X 是宽，Y 是高。`depth` 与四个半尺寸都有限且大于 0。八个角由这个数据决定，近、远平面平行，体是凸的。`Contains` 把点变到该标架后，取局部坐标 `(x, y, z)`。`z` 必须落在 `[0, depth]`。记 `s = z / depth`，该深度处的半宽是 `NearHalfWidth + (FarHalfWidth - NearHalfWidth) * s`，半高同样插值。`|x|`、`|y|` 不超过这两个半尺寸时算内部，边界算内部。

---

## 6. 查询

结果类型放在 `Query`。下面的函数都是 `noexcept` 的，除了会抛 `std::logic_error` 的 NURBS 入口。解析查询不分配内存。

```cpp
struct ParameterPoint2 {
    double Parameter = 0;
    Linear::Point2 Point{};
};

struct ParameterPoint3 {
    double Parameter = 0;
    Linear::Point3 Point{};
};

enum class CurveMeet { None, Point, Overlap, Coincident };

struct CurveMeet2 {
    CurveMeet Kind = CurveMeet::None;
    Linear::Point2 Point{};
    double ParameterOnFirst = 0;
    double ParameterOnSecond = 0;
    Linear::Segment2 Overlap{};
};

struct CurveMeet3 {
    CurveMeet Kind = CurveMeet::None;
    Linear::Point3 Point{};
    double ParameterOnFirst = 0;
    double ParameterOnSecond = 0;
    Linear::Segment3 Overlap{};
};

struct ParameterInterval2 {
    double Enter = 0;
    double Exit = 0;
    Linear::Point2 EnterPoint{};
    Linear::Point2 ExitPoint{};
};

struct ParameterInterval3 {
    double Enter = 0;
    double Exit = 0;
    Linear::Point3 EnterPoint{};
    Linear::Point3 ExitPoint{};
};
```

`CurveMeet::Point` 时只有 `Point` 与两个参数有效。`Overlap` 时只有 `Overlap` 有效，重叠段的方向沿第一个对象的方向。`Coincident` 表示两个无穷对象重合，没有有限的重叠段。`None` 表示不相交，或两者平行且不重合。

`ParameterInterval2` / `ParameterInterval3` 用于射线、线段或直线与凸体边界。`Enter ≤ Exit`。参数的含义跟第一个对象走：射线上是沿单位方向的距离，且两者都 `≥ 0`；线段上落在 `[0, 1]`；直线上是有符号距离。射线或线段的起点在体内时，`Enter` 取该对象允许的最小参数，`EnterPoint` 为这个参数对应的点。相切时 `Enter == Exit`。没有交点时外层 `std::optional` 为空。

### 6.1 布尔结果与几何结果

`Intersects` 为真的情况包括只碰到边界、共线重叠和共面重叠。`Intersection` 返回上面的结构，不在“没有一个点可以代表全部交点”时编造一个点。

| 查询 | `Intersection` 的结果 |
|---|---|
| `Line2` × `Line2` | `CurveMeet2`。平行且分离为 `None`，重合为 `Coincident`，其余为一个点 |
| `Segment2` × `Segment2` | `CurveMeet2`。只共享端点或线段内部的交点为 `Point`；正长度重叠为 `Overlap` |
| `Line2` × `Segment2` | `CurveMeet2`。线段落在直线上为 `Overlap`，重叠段就是该线段 |
| `Circle2` × `Circle2` | `CircleMeet2`。0、1、2 个点。两圆表示同一点集时 `Kind` 为 `Coincident`，`Count` 为 0 |
| `Circle2` × `Segment2` | `CircleMeet2`。0、1、2 个点。没有 `Coincident` |
| `Rectangle2` × `Rectangle2` | 返回 `Box2`。分离时为规范空盒。仅边界接触时可以是退化的非空盒 |
| `Ray3` 或 `Segment3` × `Plane` | 一个 `ParameterPoint3`，或空。线段与平面共面且有重叠时，`Intersects` 为真，`Intersection` 为空 |
| `Line3` × `Plane` | `CurveMeet3`。一个点，或 `Coincident`（直线躺在平面上），或 `None` |
| `Ray3` / `Segment3` / `Line3` × `Triangle3` | 横向穿过或点接触时为一个 `ParameterPoint3`。共面重叠时 `Intersects` 为真，`Intersection` 为空 |
| `Ray2` / `Segment2` / `Line2` × `Box2` | `ParameterInterval2`。空盒不相交 |
| `Ray3` / `Segment3` / `Line3` × `Box3` | `ParameterInterval3`。空盒不相交 |
| `Ray2` / `Segment2` × `OrientedBox2`，以及 `Ray3` / `Segment3` × `OrientedBox3` | 对应维度的 `ParameterInterval`。不对直线提供 |
| `Ray3` / `Segment3` × `Sphere` | `ParameterInterval3`，交点在球面上 |
| `Ray3` × `Cylinder` / `Capsule` / `Disk` | 圆柱与胶囊用 `ParameterInterval3`。圆盘最多一个 `ParameterPoint3`。线段版本留到后续阶段 |
| `Triangle3` × `Triangle3` 的距离 | 只有 `Distance` 与 `DistanceSquared`，本阶段不返回交线 |

二维矩形相交返回 `Box2`，是因为点接触和边接触不是 `Rectangle2`（它要求正面积）。

`CurveMeet2`、`CurveMeet3`、`CircleMeet2` 和矩形相交得到的 `Box2` 按值返回，没有交点时看 `Kind` 或空盒。`ParameterPoint` 与 `ParameterInterval` 包在 `std::optional` 里，空表示没有可返回的点或区间。

圆与圆的多点结果：

```cpp
struct CircleMeet2 {
    CurveMeet Kind = CurveMeet::None;
    int Count = 0;
    Linear::Point2 Points[2]{};
};
```

`Count` 只在 `Kind == Point` 时为 1 或 2。相切是 1。

### 6.2 距离与投影

方法形式：

- 点到 `Line2/3`、`Ray2/3`、`Segment2/3`、`Plane`、`Triangle2/3`、`Sphere`：该类型上的 `Distance`、`DistanceSquared`、`ClosestPoint`。
- 点到 `Box2/3`：加在包围盒上，空盒返回 `std::nullopt`。
- `Segment2` × `Segment2`、`Segment3` × `Segment3`、`Triangle3` × `Triangle3`：`Query::Distance` 与 `Query::DistanceSquared`。

最近点在目标对象上。线段与射线使用第 4 节的参数夹紧。三角形取到三条边或面内的最近点。球取径向投影。查询点与球心重合时，最近点为 `Center + Radius * UnitVector3::XAxis`。

### 6.3 包含

| 问题 | 位置 |
|---|---|
| 点在 `Box2/3`、`OrientedBox2/3` 内 | 已有方法，不改语义 |
| 点在 `Triangle2/3` 内 | 三角形上的 `Contains` |
| 点在 `Polygon` 内 | `Polygon::Contains`。外环非零环绕数，再排除洞的内部 |
| 点在 `Sphere`、`Cylinder`、`Capsule`、`Frustum` 内 | 各类型上的 `Contains` |
| 点在凸多边形内 | 由 `Polygon::Contains` 覆盖，不单写一个只接受凸多边形的函数 |

### 6.4 凸包

`Query::ConvexHull(std::span<const Point2>) -> std::optional<Polygon>`。

算法是 Andrew 单调链，转向用 `Orient2d`。去掉重复点和严格落在最终边内部的共线点。输出逆时针。序列的第一个顶点是 X 最小者，X 相同则取 Y 最小者，然后按逆时针列出，不重复首顶点。不共线的点少于 3 个时返回 `std::nullopt`。

### 6.5 GJK 与 EPA

```cpp
struct ConvexDistance3 {
    double Distance = 0;
    Linear::Point3 PointOnFirst{};
    Linear::Point3 PointOnSecond{};
};

struct ConvexContact3 {
    double Depth = 0;
    Linear::UnitVector3 Normal{};
    Linear::Point3 Point{};
};
```

二维使用对应的 `ConvexDistance2` / `ConvexContact2`。

`Distance` 始终返回一个值。分离时 `Distance > 0`，两个点是见证点。接触或重叠时 `Distance == 0`。`Contact` 只在穿透深度大于 0 时有值。`Normal` 指向从第一个对象到第二个对象的方向：把第二个对象沿 `Normal` 移动 `Depth` 后，穿透消失。

迭代上限与停止条件写在 `ConvexDistance.cpp` 内部，不做成调用参数。达到上限时仍返回当前有限估计，不抛异常。这个结果是近似值。测试只覆盖距离已知、且能在上限内收敛的分离体与简单重叠。

`Polygon` 参与二维 GJK 的前提是它没有洞、它是凸的，且至少有三个不共线的顶点。调用方用凸包的输出，或自行保证。函数不扫描顶点来拒绝有洞、凹或退化的多边形。

---

## 7. 分步交付

每一步只加入该步列出的类型与查询。后面的类型没出现之前，不写依赖它的函数。

**第 1 步。** `Segment2/3`、`Ray2/3`、`Line2/3`、`Plane`、`Triangle2/3`，以及它们之间的 `AsRay`、`AsLine`、`AsSegment`。直线、射线、线段带上第 4.1 节里它们做得到的部分。`Box2/3` 的最近点与距离。查询：直线与直线、线段与线段、直线与线段；同维度的射线、线段、直线与平面、三角形、`Box2`、`Box3`；同维度的射线和线段与 `OrientedBox2`、`OrientedBox3`。距离与投影覆盖这些类型之间第 6.2 节已经点名的组合，以及三角形与三角形的距离。

**第 2 步。** `Circle2`、`Arc2`、`Circle3`、`Arc3`、`Ellipse2`、`EllipseArc2`、`Rectangle2`，这些曲线的第 4.1 节，以及 `AsCircle`、`AsArc`、`AsEllipse`、`AsEllipseArc`。查询：圆与圆、圆与线段、矩形与矩形。

**第 3 步。** `Polyline`、`Polygon`（含洞）、二者的曲线协议、`AsPolygon`、`AsPolyline`、`HasSelfIntersection`、`HolesAreValid`、`HasDegenerateEdge`、`Polygon::IsConvex`、`ConvexHull`。凸包的结果没有洞。

**第 4 步。** `NurbsCurve2/3`、`CurvePolyline`、`CurvePolygon`（含洞）、`CurveCollection`、扁平的 `MultiPolygon`、`ThrowNotImplemented`，以及曲线折线与曲线多边形的互转和 `ToPolyline` / `ToPolygon`。这一步没有新的几何求交。

**第 5 步。** `Sphere`、`Cylinder`、`Capsule`、`Disk`、`Frustum`，以及它们在第 6 节中的射线、线段、包含、距离。最后做 GJK/EPA。

---

## 8. 测试

沿用仓库规则：每个新的公开方法都有直接调用它的用例。构造失败、空盒、零长度线段、零面积三角形、共线重叠、共面重叠、扫掠角为 0、扫掠角绝对值为 `2π`、非有限半径、NURBS 结构非法、NURBS 求值抛异常，都要有断言。

曲线协议按类型抽查：线段的中点与两端切向、直线没有起点和中点、圆的反向使方向变号而圆盘包含不变、圆弧不是闭合所以面积为空、椭圆参数中点不同于弧长中点、多边形的洞会减小面积并排除洞内的点、洞边界仍算包含、`Subcurve` 越界为空、直线与射线的有限子区间是线段、圆和圆弧的子区间是圆弧、矩形一条边上的子曲线是线段、跨过顶点才是折线、轨迹上的点能取回参数且接缝点取参数下端、不在轨迹上时参数为空、非均匀缩放作用在圆上得到空。

转换与检查另有直接用例：零长度线段不能变成射线，射线按正长度变成线段，有洞的多边形不能整份变成一条折线，偏差不是正数时退化为空，不相邻边相交时 `HasSelfIntersection` 为真，洞穿出外环时 `HolesAreValid` 为假，相邻边共线不算自交。

`Intersects` 的共线、共面、端点相接用例锁定布尔值。`Intersection` 的对应用例锁定 `Kind`，不只锁定一个交点坐标。凸包用例锁定起点选择、逆时针、共线点被去掉，以及少于 3 个不共线点时返回空。

谓词相关的包含与相交用精确构造的共线、共面输入，不用容差比较符号。距离用 `DistanceSquared` 与精确有理值比较；开方后的 `Distance` 只在需要检查非负和平方根时抽查。

---

## 9. 实现方案尚未开始

本轮需求已经冻结。第 7 节只记录依赖顺序。讨论并写成实现计划之前，不新增 `Prim` 与 `Query` 的头文件。

## 10. 冻结时的建议

这些建议写在这里，避免下次讨论时重新发明。还没有并进第 1–8 节的，以本节为准先讨论；已经写进前面合同的，如果要改，从这里提出。

**已经写进合同的澄清：**

1. `HasSelfIntersection` 只看同一条环。洞穿出外环由 `HolesAreValid` 负责，这样两个函数不会对同一件事给出两个名字。
2. `ParameterOf` 取的是轨迹上最近点的参数。容差内贴着曲线的点也能得到参数。
3. 圆弧和椭圆弧按弓高退化。直线段上的顶点保持原样。
4. 矩形旋转或镜像后若不再轴对齐，返回类型是 `Rectangle2` 与 `Polygon` 的变体，避免调用方猜。
5. 不增加 `Polygon3`。`Polyline3` 不能收进 `CurveCollection`，那个集合只有二维曲线。

**建议在实现方案里采纳，目前还不是合同：**

1. 自交检查用边的两两比较即可，本阶段不写扫描线。顶点数要到实现方案里再估；在那之前不为它单独立项。
2. 椭圆周长的自适应求积若在内部迭代上限内达不到容差，返回 `std::nullopt`，不要交回一个没有误差保证的长度。
3. 不为直线再加“指定另一个原点再变成射线”的重载。换原点的调用方可以先自己改 `Origin`，再调用 `AsRay`。
4. `MultiPolygon` 不检查成员重叠，面积继续允许重复计算。重叠检查留到阶段 5 的布尔运算。
5. 第 7 节的五步就是实现顺序。每一步带上该步类型的转换和检查，不要等类型都齐了再补。
