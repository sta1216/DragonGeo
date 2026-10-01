# DragonGeo 阶段 4 曲线协议修订

日期：2026-10-01
状态：已按讨论写入需求合同 `2026-09-30-dragongeo-stage4-design.md`。代码尚未按本文修改。

本文只记录相对冻结合同的改动。实现以改过的需求合同为准。`Core` 与 `Linear` 仍是 header-only。矩形、圆、椭圆在后面的步骤里使用同一套方法名，不另起一种写法。

---

## 1. 折线存点

`Polyline` 与 `Polyline3` 的数据是 `std::vector<Point>`。工厂 `FromPoints` 至少接受两个分量有限的点，否则为空。边是相邻两点，`Segment(index)` 现拼，不另存线段，也不再检查端点是否接上。`Point(0) == Point(PointCount - 1)` 时 `IsClosed` 为真。

多边形环仍然不把首顶点存在末尾。`AsPolyline` 取出环时在末尾补上首顶点。`AsPolygon` 丢掉与首点相同的末点。闭合折线的面积不要把这个重复末点再算成一个新顶点。

两条折线都是曲线，实现合同第 4.1 节。`Polyline3` 不能变成多边形。

## 2. 变换改自己

| 方法 | 返回 | 行为 |
|---|---|---|
| `Translate`、`Rotate`、`Mirror`、`Reverse` | `void` | 直接改当前对象 |
| `Transform` | `bool` | 成功则改自己。失败则对象保持原样 |
| `Clone` | 同类型 | 按值拷贝。要留下修改前的曲线，先调用它 |

失败的条件：结果非有限；射线或直线的方向无法重新归一化；圆或圆弧遇到非相似变换；椭圆遇到不可逆变换；矩形的像不再是轴对齐正面积矩形。矩形不会在这次调用里变成 `Polygon`。

`CurveCollection` 与 `MultiPolygon` 不是曲线。它们的 `Translate`、`Rotate`、`Mirror`、`Reverse` 逐个改成员。`Transform` 先确认每个成员都能成功，再改；任一成员会失败时，整个对象不变并返回 `false`。

## 3. 包围盒与方向

曲线上的包围盒方法是 `Box()`，返回 `Box2` 或 `Box3`。无界曲线没有有限包围盒时返回规范空盒。类型名 `Box2` / `Box3` 不变。

闭合方向：

```cpp
namespace DragonGeo::Prim {
enum class Winding { CounterClockwise, Clockwise, Degenerate };
}
```

`Orientation()` 返回 `std::optional<Winding>`。不闭合则为空。面积为零是 `Degenerate`。圆、椭圆、矩形把参数方向存成这个枚举，默认 `CounterClockwise`。`Degenerate` 不能存进这些类型。

## 4. 放进静态库

阶段 4 的 `Prim` 与 `Query` 在头文件里只放声明和结果类型。定义放进已有的 `DragonGeo` 静态库：`src/Prim/` 与 `src/Query/`。`double` 与 `float` 显式实例化。用到谓词的方法和全部 `Query` 仍只有 `double`。

放进库里的方法不是 `constexpr`。聚合字段，以及只判断分量是否有限的 `IsValid`，留在头文件里。

## 5. 平面

平面不是曲线。保留 `SignedDistance`、`Distance`、`ClosestPoint`、`Flipped`。`Flipped` 仍返回新平面。

新增：

- `Project(Vector)`：去掉法向分量。
- `Project(UnitVector)`：投影后再归一化。与法线平行时为空。
- `Mirror(Point)`、`Mirror(Vector)`：关于平面反射，返回新值，不改平面。
- `Contains(Point, tolerance = {})`：有符号距离的绝对值不超过 `tolerance.Resolve(1)`。
- `Offset(distance)`：原点沿法线移动，直接改自己。

## 6. 还没改的代码

第 1 步已经落地的线段、射线、直线、三角形、`Polyline3` 和查询仍是改名之前的 header-only 实现。`Polyline3` 还存着线段序列，也还没有第 4.1 节的全部方法。按本文改代码之前，先审这份修订。
