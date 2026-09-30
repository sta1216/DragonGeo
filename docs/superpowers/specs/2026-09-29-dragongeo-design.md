# DragonGeo 几何库设计

日期：2026-09-29
状态：待审阅

---

## 1. 目标与定位

一个面向**对外发布**的 C++ 几何库，覆盖三个能力域 —— 通用几何工具箱、2D 计算几何、3D 网格与图形学 —— 并提供一层 3D 精确实体建模（B-rep/NURBS）的**接口骨架**，其核心算法明确留待后续实现。

**成功标准**：

1. 使用者能在十分钟内完成接入并跑通第一个示例，无需理解内部架构。
2. 在退化输入（共线点、重合点、自交多边形、切点）上给出**正确结果**，而非崩溃或静默错误。
3. API 面在实现之前即已定型，使用者能读到完整的接口、语义与限制。

**首要取舍顺序**：`易用性 > 正确性 > 完整性 > 极致性能`。

理由：作为对外发布的库，采纳率的首要障碍是"难用"而非"不够快"；而几何库一旦在退化输入上失效，其余所有优点都不再被信任。

**非目标**（本设计明确不做）：

- 不做精确 B-rep 布尔运算、曲面求交（SSI）、倒角、抽壳的**算法实现**（仅留接口）。
- 不做 CAD 级容差推理、装配约束、参数化特征建模。
- 不做渲染、可视化、文件格式 IO（示例中可演示与外部工具对接，但不作为库的能力）。
- 不引入第三方运行时依赖。

---

## 2. 范围与优先级

全库能力按三档标记：

| 标记 | 含义 |
|---|---|
| ★ 核心 | 早期实现，属于库的基本可用性 |
| ☆ 进阶 | 后续实现，**但接口骨架在早期一并定型** |
| 骨架 | 仅接口、数据结构与文档，方法体为明确的未实现 |

**关键原则：接口先行。** 所有标记为 ☆ 与骨架的能力，其签名、文档、数据类型在早期阶段即定型并纳入 API 审查，实现按优先级逐步填充。这样做的收益是 API 面积与文档可在投入实现成本之前被评估和纠正；代价是早期会存在一批抛异常的入口，须由能力矩阵文档明确标注。

---

## 3. 架构分层

严格单向依赖的命名空间分层。每个 namespace 对应 `include/DragonGeo/` 下的一个目录，**边界即目录边界**。

```
Solid        B-rep 骨架：拓扑结构 + 曲面/曲线接口（求交算法留空）
  ↑
Mesh         三角网格、邻接关系、BVH 加速结构
Polygon      2D 多边形算法：布尔、三角剖分、凸包、偏移
  ↑
Query        求交 / 距离 / 投影 / 包含 —— 跨原语的自由函数
  ↑
Prim         几何原语：射线、线段、直线、平面、三角形、球、圆柱、胶囊、圆盘、视锥
             以及后续的矩形、圆与圆弧、椭圆、多段线、NURBS 曲线
  ↑
Predicates   Orient2d / Orient3d / Incircle / Insphere（过滤 + 自适应精确）
  ↑
Linear       数值与仿射基础：Vector2/3/4、UnitVector2/3、Matrix2/3/4、Quaternion、
             Transform2/3、Point2/3、Interval、Box2/3、OrientedBox2/3、Coordinate2/3
  ↑
Core         Scalar、Tolerance、常量、数值工具
```

### 3.1 依赖规则

1. **依赖只向上，不出现环。** `Linear` 不知道"射线"是什么，`Predicates` 不知道"多边形"是什么。上层可自由使用下层，下层永不反向依赖。
2. **`Query` 是无状态自由函数层。** 所有求交/距离/投影均为 `Intersect(a, b) -> std::optional<...>` 形式的自由函数，不引入虚函数分派。它是三大能力域共享的底座。
3. **`Polygon` / `Mesh` / `Solid` 三者平级、互不依赖。** 将来若需要"2D 截面挤出为 3D"一类跨域功能，应作为新的上层模块引入，而非污染现有层。

### 3.2 目录结构

```
include/DragonGeo/  公共头文件，与 namespace 一一对应；内部实现置于 */Detail/
src/              编译单元：Predicates、Polygon、Mesh、bvh
tests/            单元测试 + 性质测试 + 退化用例回归集
examples/         可独立编译运行的示例
benchmarks/       性能基线
docs/             设计文档与 API 文档
cmake/            包配置模板（find_package 支持）
```

### 3.3 构建产物

对外只暴露单一 `DragonGeo::DragonGeo` target，使用者无需做链接选择题。内部拆分为多个 OBJECT 库以并行编译。

核心类型（`Core` / `Linear` / `Prim`）为 header-only；算法层（`Predicates` / `Query` 部分 / `Polygon` / `Mesh` / `Solid`）编译为库。这样在保持类型零开销的同时，避免精确谓词与 BVH 造成使用者的编译时间灾难。

### 3.4 命名规范

> 2026-09-30 起以本节为准。本节之前的实施计划、以及后文里尚未改写的旧代码片段，若仍使用 `snake_case` 函数名或尾随下划线的成员名，那是改名之前的记录，**新代码不得照抄**。

**文件名、类名、方法名、自由函数名、类型别名：大驼峰（PascalCase）。** 优先完整拼写。例：`Vector3Test.cpp`、`DistanceTo`、`ScalarType`、`Orient2d`、`FromZAxis`。运算符保持 `operator+` 这种写法，不改成单词。

缩写仅在完整名称**超过 3 个词或 20 个字符**时允许，且须是行业内无歧义的写法。已知允许的缩写仅两个：`BSpline`、`BVH`。其余一律完整拼写。

| 使用 | 而非 | 说明 |
|---|---|---|
| `Vector3` | `Vec3` | |
| `Matrix4` | `Mat4` | |
| `Quaternion` | `Quat` | |
| `UnitVector3` | `UnitVec3` | |
| `Rectangle` | `Rect` | |
| `AxisAlignedBox3` | `AABB3` | 全称 `AxisAlignedBoundingBox3` 为 4 词 24 字符，超限故缩去 Bounding |
| `OrientedBox3` | `OBB3` | 同上 |
| `BoundingVolumeHierarchy` | — | 2 词 25 字符，超字符上限，允许缩写为 `BVH` |

**函数参数、局部变量：小驼峰（camelCase）。** 例：`angleRadians`、`scaledX`、`roundTrip`。函数内部的 `const` 局部量仍是变量，用小驼峰，不因为加了 `const` 就写成全大写。

**public 数据成员：大驼峰。** 例：`X`、`Y`、`Z`、`W`、`Min`、`Max`、`Abs`、`Rel`、`Data`、`Frame`、`Matrix`、`HalfExtent`。

**非 public 数据成员：`m_` + 小驼峰。** 例：`m_value`、`m_origin`、`m_x`。不要再用尾随下划线。

**指针：`p` + 大驼峰。** 裸指针写作 `pNode`。智能指针允许换前缀，后面仍是大驼峰：`unique_ptr` 用 `up`（`upNode`），`shared_ptr` 用 `sp`（`spNode`），`weak_ptr` 用 `wp`（`wpNode`）。

**常量：全大写，单词之间用下划线。** 范围是命名空间作用域的 `constexpr` / `const`、类作用域的编译期常量、枚举量，以及宏。例：`PI`、`HALF_PI`、`TWO_PI`、`DIMENSION`。宏在这条规则上再加库前缀：`DRAGONGEO_UPPER_SNAKE`。

**注释用中文。** 句子里可以嵌入公式、命令和标识符。

**模块名（namespace，以及与之对应的目录）：大驼峰。** 例：`DragonGeo::Core`、`DragonGeo::Linear`、`DragonGeo::Predicates`、`DragonGeo::Polygon`、`DragonGeo::Detail`。目录与 namespace 同名，如 `include/DragonGeo/Linear/`。模块里若有同名类型，类型写在该命名空间之内，例如类是 `DragonGeo::Polygon::Polygon`。

**工具链保留名不改：** `CMakeLists.txt`，以及程序入口 `main.cpp`。

---

## 4. 数值核心与容差模型

### 4.1 核心洞察：谓词层不使用容差

精确谓词返回的是**确定的符号**，而非近似值。`Orient2d(a, b, c)` 返回负/零/正，其中"零"意味着三点**确实共线**，而不是"误差在阈值内"。因此"点在三角形内"、"线段是否相交"、"凸包方向"这类判定完全不需要 epsilon。

容差的出现范围被压缩到仅剩两处：

1. **近似相等判定** —— 顶点能否合并、两点是否视为重合。
2. **构造结果的有效性判定** —— 交点是否落在线段内、两个平面是否视为共面。

其余判定一律走精确符号。

### 4.2 容差模型

**拒绝全局 `EPS` 常量。** 单一全局阈值在尺度相差数个数量级的输入上必然出错。

容差是**显式参数**，默认值来自一个结构体，可逐调用覆盖：

```cpp
namespace DragonGeo::Core {

struct Tolerance {
    double abs = 1e-12;   // 绝对项
    double rel = 1e-9;    // 相对项

    // 有效容差 = abs + rel * |magnitude|
    [[nodiscard]] constexpr double resolve(double magnitude) const noexcept;
};

} // namespace DragonGeo::Core
```

**代价**：函数签名变长。
**收益**：调用者始终知道自己在何种精度下工作，且能针对具体问题调参；不存在"库作者替你选了一个对你不合适的阈值"的情况。

### 4.3 Scalar 策略

代数类型模板化，但对使用者隐藏模板：

```cpp
template <typename Scalar> struct Vector3T;
using Vector3 = Vector3T<double>;   // 绝大多数使用者只会看到这一层
```

算法层只实例化 `double`（通过显式实例化控制编译时间与二进制体积）。需要 `float` 或扩展精度的场景，后续按需增加实例。

### 4.4 Point / Vector / UnitVector 分离

同底层存储，不同类型语义。**不变量由类型系统承载。**

```cpp
// Point 与 Vector 分离（下面以 3D 为例；2D 的 Point2 / Vector2 逐条同构）
Point3 operator+(Point3 p, Vector3 v);    // -> Point3
Point3 operator+(Vector3 v, Point3 p);    // -> Point3，与上一条对称：平移不关心书写顺序
Point3 operator-(Point3 p, Vector3 v);    // -> Point3
Vector3 operator-(Point3 a, Point3 b);   // -> Vector3
Point3 operator*(Transform3 t, Point3 p); // -> Point3：点吃平移；Transform3 * Vector3 不吃
// 两个点相加在编译期即不存在

// 单位向量独立类型，存储等价于 Vector3，零额外开销
struct UnitVector3;

UnitVector3 * double            -> Vector3     // 缩放后不再保证单位长度
UnitVector3 + UnitVector3       -> Vector3     // 和一般不是单位向量
UnitVector3 - UnitVector3       -> Vector3
-UnitVector3                    -> UnitVector3 // 取反仍是单位向量
dot(UnitVector3, UnitVector3)   -> double      // 即余弦，范围 [-1, 1]
cross(UnitVector3, UnitVector3) -> Vector3     // 平行时退化为零向量，不保证长度
```

运算的返回类型如实反映该运算**是否保持不变量** —— 这是该设计的核心价值，而非单纯的限制。

**构造**是唯一需要处理退化的地方，也是 `Tolerance` 的第一个真实用例：

```cpp
std::optional<UnitVector3> Vector3::Normalized(Tolerance tol = {}) const;  // 零向量或退化向量返回 nullopt
UnitVector3 UnitVector3::FromNormalizedUnchecked(Vector3 v);               // 前置条件：|v| > 0，违反则行为未定义
```

`FromNormalizedUnchecked` 的命名即是警告：它跳过容差检查，**违反前置条件是未定义行为**。仅在调用者已通过其他途径确知非零时使用。

**类型系统带来的实际收益**：`Plane` 的法线、`Ray` 的方向从此不必依赖文档约定：

```cpp
struct Plane { UnitVector3 normal; double offset; };
// 法线未归一化则平面方程的常数项没有"距离"的物理意义 —— 该约束现在由类型保证
```

**文档义务**：`UnitVector3` 的不变量是**弱不变量** —— 浮点归一化后 `|v|` 不完全等于 1，它保证的是"已归一化过一次"。此限制必须在类型文档中写明。

**边界**：此"带不变量类型"模式**止步于 `UnitVector2` / `UnitVector3`**。不引入 `Rotation`、`Normal`、`Symmetric` 等类型。理由：模式一旦确立，后续新增此类类型是纯增量且不破坏 API 的；现在为想象中的需求付费不符合 YAGNI。

### 4.5 谓词实现

**内建 C++ 实现**，不引入外部依赖：

- **过滤（filtered predicates）** —— 先以浮点计算行列式并给出严格误差界。若符号在误差界之外，直接返回（常规输入命中率 > 99%）。
- **自适应精确算术** —— 仅当落在不确定区间时，展开为 error-free transformation 序列做精确判定。对 `Orient2d` 约需 4 项展开，`Orient3d` 约 8 项，`Incircle` / `Insphere` 更多。

接口：

```cpp
namespace DragonGeo::Predicates {

[[nodiscard]] int Orient2d(Point2 a, Point2 b, Point2 c) noexcept;  // -1 / 0 / +1
[[nodiscard]] int Orient3d(Point3 a, Point3 b, Point3 c, Point3 d) noexcept;
[[nodiscard]] int Incircle(Point2 a, Point2 b, Point2 c, Point2 d) noexcept;
[[nodiscard]] int Insphere(Point3 a, Point3 b, Point3 c, Point3 d, Point3 e) noexcept;

} // namespace DragonGeo::Predicates
```

返回三值符号，且**必须能够精确返回零** —— 这是浮点近似永远无法可靠给出、而精确算术可以给出的信息。

自建而非移植第三方实现，以保持许可证干净、代码风格一致、构建零依赖。这是全库唯一允许"复杂"的模块，其余代码保持平实可读。

### 4.6 变换

`Transform2` / `Transform3` 表示仿射变换（3×3 / 4×4，列向量约定 `M * v`）。`Quaternion` 提供旋转。法线变换使用逆转置矩阵，按需计算、不缓存 —— 除非基准测试证明它是热点。

---

## 5. 能力清单

### 5.0 Linear — 数值与仿射基础

**纯代数**：`Vector2/3/4`、`UnitVector2/3`、`Matrix2/3/4`、`Quaternion`、`Transform2/3`

**仿射与区间**：`Point2/3`、`Interval`、`Box2/3`、`OrientedBox2/3`、`Coordinate2/3`

**Point / Box / Coordinate 属于 `Linear` 而非 `Prim`**，这是对早期划分的一次修正。理由有三：

1. `Point` 只依赖 `Core`，与 `Vector` 同样处于依赖图的叶子位置 —— 它不含"射线""平面"这类几何语义，把它放 `Prim` 的那条界线是薄的。
2. 放进 `Linear` 后，`Transform` 可以**直接提供变换点的能力**（`transform_point(Point) -> Point`）。原先因 `Point` 在 `Prim` 而被迫采用的 `apply(t, Vector)` 分工 —— 把"位置"塞进 `Vector` —— 随之消失。那不是优雅的取舍，是划错层之后的补偿动作。
3. `Box`/`OrientedBox`/`Coordinate` 只依赖 `Point` 与 `UnitVector`，跟随 `Point` 一同落在 `Linear` 不会增加层数；`Coordinate` 与 `Transform` 成为同层兄弟（一个是参考系的几何表示，一个是其矩阵表示），这是它们应有的关系。

`Prim` 因此收敛为**有真实几何语义的原语**，那条线更干净。

### 5.1 Prim — 几何原语

**2D**：`Segment2` `Ray2` `Line2` `Circle` `Arc2` `Ellipse2` `Triangle2` `Rectangle` `Polyline2`（简单：仅直线段）`Polyline2`（复杂：直线段 / 圆弧段 / 椭圆弧段任意组合）
**3D**：`Segment3` `Ray3` `Line3` `Plane` `Triangle3` `Sphere` `Cylinder` `Capsule` `Disk` `Frustum`

标记：★ 已有规划，☆ 后续阶段新增。

**曲线的若干已定裁决**（详见 §5.6）：

- **圆弧与椭圆是不同的类型**；圆与圆弧**共用一个类型**（`Circle` 表示整圆，`Arc2` 表示其上的弧段）—— 圆是圆弧的特例，不是椭圆的特例。
- 复杂多段线的每一段可为直线段、圆弧段、椭圆弧段三者之一，需要**变体段类型**承载。
- `NURBS` 在 `Prim` 阶段**只做骨架**，求值/导数/分割留作独立任务。

全部为 POD 风格值类型，`constexpr` 可构造，成员为命名字段（便于调试）而非数组。

### 5.2 Query — 求交 / 距离 / 投影 / 包含

**避免 N² 爆炸是此层的核心设计约束。** N 个原语两两组合会产生 N² 个函数，硬编码全覆盖将导致 API 面积失控、维护成本不可承受。因此采用**双层策略**：

**第一层：解析解** —— 高频组合编写专门实现，精度与速度最优：

| 目标 | Ray | Segment | Line |
|---|---|---|---|
| Plane | ★ | ★ | ★ |
| Triangle | ★ | ★ | ★ |
| AxisAlignedBox | ★ | ★ | ★ |
| OrientedBox | ★ | ★ | — |
| Sphere | ★ | ★ | — |
| Cylinder / Capsule / Disk | ★ | ☆ | — |

注：`—` 表示该组合既不在 ★ 也不在 ☆ 范围内，属明确不做；长尾组合由第二层兜底。

**第二层：通用凸体算法（GJK / EPA）** —— 一次性覆盖所有凸体之间的距离与接触判定：`AxisAlignedBox`、`OrientedBox`、球、胶囊、凸包、`Box`、`Frustum`。这是避免 N² 的关键，也是长尾组合的默认兜底路径。

**其余查询**：

- **距离** ★：点到线/段/射线/平面/三角形/AxisAlignedBox/球；段-段；三角形-三角形
- **投影** ★：点到线/段/射线/平面/三角形/AxisAlignedBox/球
- **包含** ★：点在 AxisAlignedBox/OrientedBox/球/三角形/凸包/Frustum/圆柱/胶囊内
- **2D** ★：点在多边形内、线段相交、直线相交、圆-圆、圆-线段、矩形-矩形

### 5.3 Polygon — 2D 计算几何

**★ 核心**：凸包（monotone chain）· 面积/质心/周长/方向判定 · 点包含（winding number，走精确谓词）· 自交检测 · 耳切三角剖分 · **布尔运算（并/交/差/异或）**

**☆ 进阶**：多边形偏移（Clipper 风格）· Delaunay 三角剖分 · 约束 Delaunay（CDT）· Sutherland-Hodgman 裁剪 · Douglas-Peucker 简化 · 凸分解

布尔运算是整个 2D 层最大的单项工程（约 2–3k 行），也是精确谓词唯一真正无可替代的地方。

### 5.4 Mesh — 3D 网格与图形学

**★ 核心**：索引三角网格 + 邻接查询 · 面/顶点法线 · 面积/体积/包围盒 · **BoundingVolumeHierarchy（BVH）**（构建 + 射线求交 + 最近邻）

**☆ 进阶**：半边结构（拓扑编辑用）· QEM 网格简化 · Laplacian/Taubin 平滑 · 顶点焊接与网格修复 · 平面切片 · 网格布尔

**两项明确的高风险项**（须在能力矩阵中标注）：

- **半边结构**内存开销显著（每条边两个半边指针），且与索引面表是两套并存的数据结构。设计决策：**索引面表为主，半边结构后补**。
- **网格布尔**难度接近 CAD 布尔（须处理自交、共面、退化），仅精度要求较低。列为 ☆ 中的高风险项，**不作为默认承诺**。

### 5.5 Solid — CAD 精确实体（骨架）

仅接口、数据结构与文档，不实现核心算法：

- **拓扑**：`Vertex` / `Edge` / `Face` / `Shell` / `Solid` —— B-rep 的 face-edge-vertex 图，含遍历与拓扑有效性校验
- **曲线**：`Line` / `Circle` / `Ellipse` / `BSplineCurve`
- **曲面**：`Plane` / `Cylinder` / `Sphere` / `Cone` / `Torus` / `BSplineSurface`
- **操作接口**：曲面求交（SSI）、布尔、倒角、抽壳、偏移 —— 全部为已定义签名 + 明确的未实现行为

### 5.6 曲线与多段线（登记，待 `Prim` 阶段实现）

本节记录一组已确认的需求，**实现排期在后续的 `Prim` 阶段**，本轮不做。此处登记的目的是让分层归属与类型关系在动工前就定下来，避免届时重新讨论。

**要实现的类型**：

| 类型 | 说明 |
|---|---|
| `Rectangle` | 轴对齐矩形（2D），可由两个角点或一个角点加尺寸构造 |
| `Circle` / `Arc2` | **圆与圆弧共用一套类型**：`Circle` 表示整圆，`Arc2` 表示其上的弧段（带起止角） |
| `Ellipse2` / `EllipseArc2` | **椭圆与椭圆弧是独立于圆/圆弧的类型**，不与 `Circle` 合并 |
| `Polyline2`（简单） | 仅由直线段构成的多段线 |
| `Polyline2`（复杂） | 每段可为**直线段、圆弧段、椭圆弧段**三者之一 —— 需要一个变体段类型承载 |
| `NurbsCurve2` / `NurbsCurve3` | **先做骨架**：控制点、节点向量、次数、权重的数据结构与接口；求值、导数、分割留作独立任务 |

**已定裁决**（未来实现时不得推翻，若需推翻须先改本节）：

1. **圆弧与椭圆是不同的类型。** 圆弧是圆上的弧段，椭圆弧是椭圆上的弧段，二者不共享类型。
2. **圆与圆弧共用一个类型。** 圆是圆弧的退化情形（整圈），而不是椭圆的特例 —— 这条与第 1 条并不矛盾：圆与椭圆本就不同族，圆与它的弧同族。
3. **复杂多段线用变体段类型**，而非三个并列的容器；段的种类是封闭集合（三种），适合用 `std::variant` 或等价机制表达。
4. **NURBS 先骨架后实现**，骨架须满足 §6 的三条骨架约定（签名完整、结构可用、未实现行为统一抛 `std::logic_error`）。

**尚未决定、实现前须确认的**：

- 复杂多段线的段类型是复用 `Prim` 的 `Segment2`/`Arc2`/`EllipseArc2`，还是包一层带参数的段结构（后者能携带"从第几段到第几段"的定位信息）。
- `Polyline2` 是否要求连续（后段起点等于前段终点），还是允许离散段序列。
- 椭圆弧的参数化方式（起始角 + 扫掠角 / 两个参数点 / 四段贝塞尔近似）。

**不在本轮范围**：任何曲线类型的实现、曲线求交、曲线离散化、以及依赖它们的 SVG 序列化。

---

## 6. 骨架统一约定

一个合法的骨架单元必须同时满足三条，缺一不可：

1. **签名完整** —— 含前置条件、复杂度、返回值语义的文档注释。使用者能读到该功能将来做什么、如何调用、失败时如何表现。
2. **结构可用** —— 数据结构与拓扑可构造、可遍历、可校验。以 `Solid` 的 B-rep 为例：即使布尔运算未实现，也应能手工构造一个实体、遍历其面-边-顶点、检查拓扑有效性。
3. **未实现行为有统一定义** —— 不得静默返回错误结果，不得是未定义行为。

### 6.1 未实现的运行时行为

全库统一：**抛出 `std::logic_error`**，消息包含函数名与未实现原因。

```cpp
namespace DragonGeo::Detail {
[[noreturn]] void throw_not_implemented(const char* function, const char* reason);
}
```

**选型理由**：明确、可捕获、测试友好、不会静默出错。C++ 异常默认开启，满足绝大多数消费者；`no-exceptions` 项目可通过 CMake 选项切换为 `abort`（该选项不改变任何签名，故为纯增量能力，在确有需求时再添加）。

### 6.2 能力矩阵

以 `docs/CAPABILITIES.md` 与测试的形式维护，**不提供编译期的能力查询 API**（避免为想象中的需求付设计成本）。骨架相关的测试用例须与矩阵保持一致，矩阵过时即测试失败。

---

## 7. 工程化

### 7.1 语言与构建

- **C++20 必需。** 使用 concepts 约束模板、`std::span` 表达接口、`<=>` 简化类型比较。
- **CMake ≥ 3.20** + `CMakePresets.json` 提供开箱即用配置。
- **零运行时依赖。**
- **三种接入方式**：`find_package` / `add_subdirectory` / `FetchContent`。

### 7.2 测试策略

几何库的质量命脉，分四层：

1. **单元测试** —— 每个 API 的基本行为。
2. **退化用例回归集** —— 共线点、重合点、零面积三角形、自交多边形、切点。几何库绝大多数 bug 集中于此。**规则：每个修掉的 bug 必须转成一条永久用例，永不删除。**
3. **性质测试（property-based）** —— 随机输入验证不变量，在无参考答案时亦能抓住真实错误：
   - 布尔运算：`area(A∪B) + area(A∩B) == area(A) + area(B)`
   - 三角剖分：面积守恒
   - 凸包：结果确实凸，且包含全部输入点
   - 谓词：`Orient2d(a, b, c) == -Orient2d(a, c, b)`

   浮点等式一律以容差比较，且容差须随输入坐标尺度缩放 —— 使用固定绝对阈值会让性质测试在大坐标下假失败、在小坐标下放行真错误。
4. **对拍**（可选，作为验证手段而非依赖）—— 与 CGAL / Clipper2 / Boost.Geometry 交叉比对。

补充：**libFuzzer** 覆盖布尔运算与解析入口；**Google Benchmark** 建立性能基线，防止后续"优化"变成退化。

### 7.3 CI

GitHub Actions 矩阵：{MSVC, GCC, Clang, AppleClang} × {Debug, Release}，挂载 ASan / UBSan、覆盖率、clang-tidy。

### 7.4 文档

- `README.md` —— 快速开始 + 特性矩阵 + 构建方法
- `docs/` —— 设计文档、能力矩阵
- Doxygen API 参考
- `examples/` —— 可独立编译运行的示例

对发布库而言，**示例的重要性不亚于文档**：它是使用者实际复制粘贴的东西。

### 7.5 版本与兼容

- **semver 2.0**。
- 核心 namespace 保证源码兼容；破坏性变更走大版本。
- 骨架 API 在实现落地前标记 `unstable` —— 签名可能调整。
- `experimental` namespace 明确不保证任何兼容性。

### 7.6 许可证

默认 **MIT**（C++ 生态中最易被采纳）。若需要专利授权条款，改用 Apache-2.0。

---

## 8. 实施路线图

分阶段交付。**每个阶段独立可用**，不以"全部完成"为前提。

本文件描述的是**全库架构**，因此会跨越全部阶段。实施计划按阶段分别生成 —— 每个阶段的计划在该阶段启动前另行动笔，以便把前序阶段的经验纳入考量。

> **路线图已按实际执行情况重排。** 原先把 `Core`/`Linear`/`Predicates`/`Prim`/`Query` 与三套骨架并列为"阶段 1"，实测下来那个划分过粗：单是 `Core` + `Linear` 就产出了 53 个 commit、3340 行与 527 条断言。现按依赖顺序细分为下列阶段，**每个阶段独立可交付**。

| 阶段 | 内容 | 状态 |
|---|---|---|
| **1. 数值地基** | `Core`（常量、数值工具、容差模型）+ `Linear` 的纯代数部分（`Vector` / `UnitVector` / `Matrix` / `Quaternion` / `Transform`） | ✅ 已完成 |
| **2. API 重构与基础类型** | `Linear` 的 API 成员函数化；`Vector`/`Point` 的下标访问与数组导出；新增 `Point2/3`、`Interval`、`Box2/3`、`OrientedBox2/3`、`Coordinate2/3` | ← 本阶段 |
| **3. 精确谓词** | `Predicates`：`Orient2d` / `Orient3d` / `Incircle` / `Insphere`（过滤 + 自适应精确算术） | |
| **4. 几何原语与查询** | `Prim`（含 §5.6 的曲线，NURBS 先骨架）+ `Query` 的解析解与 GJK/EPA | |
| **5. 2D 计算几何** | `Polygon` 全部 ★ 项；通过性质测试与退化回归集 | |
| **6. 3D 网格与图形学** | `Mesh` 全部 ★ 项；BVH 性能达标 | |
| **7. CAD 精确实体** | `Solid` 的 SSI、布尔、倒角、抽壳。**高风险**：须先评估"自研 vs 集成现成内核" | |

序列化（JSON 与 SVG）**尚未排期**：JSON 已明确暂不实现；SVG 依赖 2D 图形对象，须待阶段 4 的曲线与多段线落地后才有意义。

---

## 9. 关键决策记录

| # | 决策 | 理由 |
|---|---|---|
| 1 | 内核采用"double 默认 + 过滤/自适应精确谓词" | 在真正鲁棒与真正易用之间取平衡；鲁棒性的代价只在谓词层付出 |
| 2 | 拒绝全局 `EPS` 常量，容差显式传参 | 单一全局阈值在跨尺度输入上必然失效 |
| 3 | `Point` / `Vector` / `UnitVector` 类型分离 | 以零运行时成本消灭一整类真实 bug；让不变量由类型承载 |
| 4 | 带不变量类型止步于 `UnitVector2/3` | 模式已确立，后续新增为纯增量；现在扩张是 YAGNI |
| 5 | 严格单向分层，`Polygon`/`Mesh`/`Solid` 平级 | 保证任意一层可独立理解与测试；跨域功能不污染既有层 |
| 6 | `Query` 为无状态自由函数层，不引入虚分派 | 三大能力域共享同一底座，且无虚表开销 |
| 7 | 双层 Query 策略：解析解 + GJK/EPA 兜底 | 避免 N² 组合导致的 API 面积爆炸 |
| 8 | 内建谓词实现，零第三方运行时依赖 | 许可证干净、风格一致、接入无障碍 |
| 9 | 接口先行，☆ 与骨架的签名在早期定型 | API 面可在投入实现成本之前被审查和纠正 |
| 10 | 未实现行为统一抛 `std::logic_error` | 明确、可测试、不静默出错 |
| 11 | C++20 必需 | 换取更干净的接口与更好的编译期诊断；接受排除旧工具链的代价 |
| 12 | 核心 header-only，算法层编译 | 类型零开销，同时避免谓词与 BVH 拖垮使用者的编译时间 |
| 13 | 类型名 PascalCase 且优先完整拼写 | 缩写规则的判断成本高于多打几个字符的成本；完整名称在自动补全下几乎无额外负担 |
| 14 | 模块 namespace 与目录用大驼峰 | 与文件名、类名同一套规则；同名类型放在模块命名空间内，如 `DragonGeo::Polygon::Polygon` |
| 15 | `Point` / `Box` / `OrientedBox` / `Coordinate` 归 `Linear`，不归 `Prim` | `Point` 只依赖 `Core`，与 `Vector` 同处依赖图叶子位置；放 `Linear` 后 `Transform` 可直接提供变换点的能力，消除原先 `apply(t, Vector)` 把位置塞进向量的补偿性分工 |
| 16 | `Circle` 与 `Arc2` 共用类型，`Ellipse`/`EllipseArc` 独立成族 | 圆是圆弧的退化（整圈），不是椭圆的特例；两组曲线的参数化与求交都不同，合并会带来虚假的统一 |
| 17 | 自由函数式的具名运算（`dot`/`cross`/`norm`/`determinant`…）一律改为成员函数 | 便于 IDE 自动补全与发现；运算符仍为自由函数（二元运算的对称性要求），这一分界在阶段 2 明确 |

---

## 10. 风险与待确认

### 10.1 已识别风险

| 风险 | 影响 | 缓解 |
|---|---|---|
| 精确谓词的验证成本被低估 | 谓词错误会污染所有上层，且极难定位 | 独立验证：与多精度库（如 MPFR）穷举对拍，覆盖过滤边界 |
| `Polygon` 布尔运算是 2–3k 行的高复杂度工程 | 可能显著拖长阶段 2 | 独立成阶段；以性质测试为验收标准而非时间 |
| 网格布尔难度接近 CAD 布尔 | 若被当作默认承诺会伤害可信度 | 已在能力矩阵中标注为高风险，不作为默认承诺 |
| "接口先行"可能定型了错误的接口 | 后期修改破坏兼容性 | 骨架标记 `unstable`；阶段 1 与阶段 2 之间是接口的调整窗口 |
| 全库 API 面积过大 | 文档与维护负担 | 以能力矩阵显式标注未实现项；按阶段评审 |

### 10.2 待确认

**已定**：项目名与 namespace 为 `DragonGeo`；许可证 MIT（若需专利授权条款则改 Apache-2.0）。

**序列化栈尚未排期**：JSON 已明确暂不实现；SVG 依赖 2D 图形对象，须待阶段 4 落地。届时需要重新决定 JSON 的实现方式 —— spec 的"零运行时依赖"约束意味着 C++ 没有标准 JSON 可用，自建一个最小实现（约 300–500 行，含解析）与引入第三方库是两个方向不同的选择，会影响序列化层的整体设计。
