# DragonGeo Linear 层增强设计

日期：2026-10-01
状态：**已确认**（2026-10-01）。§1–§7 为合同；实现已落在 `include/DragonGeo/Linear/` 与对应测试。

阶段 4 已暂停。本文只谈 `DragonGeo::Linear` 的增量能力，不引入 Prim / Query 类型。总览见 `2026-09-29-dragongeo-design.md` §5.0。阶段 4 合同里已写、但尚未实现的 `Box2/3` 最近点与距离，在本设计中落地，阶段 4 恢复实现时直接调用即可。

---

## 1. 范围

**本设计要做：**

1. `Box2` / `Box3`：点集包围、`ClosestPoint`、`Distance`、`DistanceSquared`。
2. `Transform2` / `Transform3`：绕任意点的旋转工厂、`TransformNormal`（仅三维）。
3. `Quaternion`：从旋转矩阵提取（含容差）。
4. `Vector2` / `Vector3` / `Vector4`：`Lerp`。
5. `Point2` / `Point3`：`Lerp`。
6. `UnitVector2` / `UnitVector3`：夹角；二维另有带符号夹角。

**明确不做：**

- 不改 `Box` / `Interval` / `OrientedBox` 已有 `Contains`、`Intersects`、`Merged`、`Intersection` 的精确比较语义（仍不用 `Tolerance`）。
- 不给 `OrientedBox` 加最近点或距离（本阶段；若以后需要另开设计）。
- 不在 Linear 里加 Prim 语义（线段、平面、射线等）。
- 不加全局 epsilon；容差仍走 `Core::Tolerance` 显式参数。
- 不实现 `TransformNormal` 的二维版（二维法线是标量，Prim 的 `Plane` 用 `Flipped` 即可）。

---

## 2. 需要确认的裁决

1. **空盒没有最近点。** `ClosestPoint`、`Distance`、`DistanceSquared` 在 `IsEmpty()` 为真时一律返回 `std::nullopt`。非空盒对任意有限点都有唯一最近点（逐轴夹紧）。
2. **距离非负。** `Distance` 是 `DistanceSquared` 的开方；`DistanceSquared` 供精确比较，与 `Point::DistanceSquared` 命名一致。
3. **点集包围盒。** `FromPoints` 接收 `std::span<const Point>`。空 span 返回规范空盒 `Empty()`。任一点含非有限分量时返回规范空盒，与 `FromCorners` 对 NaN 的处理一致。不 dedupe 重复点。
4. **绕点旋转用复合，不扩矩阵 API。** `Transform2::RotationAbout(point, angle)` 等价于 `Translation(point - origin) * Rotation(angle) * Translation(origin - point)` 的工厂封装（实现时写成一次矩阵乘或等价展开）。三维 `RotationAbout(point, axis, angle)` 同理。
5. **法线变换。** `Transform3::TransformNormal` 只接受 `UnitVector3`。线性部分为 `L` 时，方向向量 `n` 按 `(L^{-1})^T n` 变换，再归一化。`L` 在容差下奇异时返回 `std::nullopt`。不要求调用方传入逆转置；在函数内按需求 `Matrix3` 的逆（与 `Inverse` 同一容差）。
6. **四元数提取。** `Quaternion::FromRotationMatrix(matrix, tolerance)` 只处理 3×3 旋转（正交、行列式 +1）。否则 `std::nullopt`。不要求从带缩放的 4×4 仿射矩阵提取；那属于 `Transform3` 分解，不在本设计。
7. **夹角。** `UnitVector2/3::AngleBetween(other)` 返回 `[0, π]` 的弧度，用 `acos` 前把点积夹到 `[-1, 1]`。`UnitVector2::SignedAngle(other)` 返回 `(-π, π]`，用 `atan2(cross, dot)`，符号与现有二维叉积约定一致（CCW 为正）。不对零向量定义；单位向量类型已排除零长。
8. **插值。** `Vector` 与 `Point` 的 `Lerp(other, t)` 要求 `t` 为有限数；`NaN` 或无穷时结果分量按 IEEE 规则传播，不抛异常。`t` 不在 `[0, 1]` 仍允许（ extrapolation ），不 clamp。
9. **`constexpr` 边界。** 逐轴夹紧、`Lerp`、绕点旋转的**纯代数部分**在能 `constexpr` 处保持 `constexpr`。`Distance`、`AngleBetween`、`FromRotationMatrix`、`TransformNormal` 涉及 `sqrt` / `acos` / 矩阵求逆，不标 `constexpr`。
10. **模板与别名。** 新方法写在 `*T<Scalar>` 上，并提供 `double` / `float` 别名。不新增仅 `double` 的自由函数层。

---

## 3. Box2 / Box3

### 3.1 FromPoints

```cpp
[[nodiscard]] static Box2T FromPoints(std::span<const Point2T<Scalar>> points) noexcept;
```

三维为 `Point3T`。算法：初值为 `Empty()`，对每个有限点用 `Merged(Box::FromCorners(p, p))` 等价逻辑扩展 Min/Max；实现可手写单次遍历，语义须与「逐点合并退化盒」一致。

### 3.2 ClosestPoint

```cpp
[[nodiscard]] constexpr std::optional<Point2T<Scalar>> ClosestPoint(Point2T<Scalar> point) const noexcept;
```

对每个轴：`c = clamp(point.i, Min.i, Max.i)`，比较用与 `Contains` 相同的精确顺序（`Min <= Max` 已保证非空盒）。非有限输入点：仍返回 clamp 结果；若 clamp 产生非有限，返回 `std::nullopt`（与「给不出有限最近点」一致）。

### 3.3 DistanceSquared / Distance

```cpp
[[nodiscard]] constexpr std::optional<Scalar> DistanceSquared(Point2T<Scalar> point) const noexcept;
[[nodiscard]] std::optional<Scalar> Distance(Point2T<Scalar> point) const noexcept;
```

先 `ClosestPoint`；空则空。否则 `(point - closest).LengthSquared()` / `Length()`。点在盒内时距离为 0。

---

## 4. Transform

### 4.1 绕任意点旋转

```cpp
// Transform2T
[[nodiscard]] static Transform2T RotationAbout(Point2T<Scalar> origin, Scalar angleRadians) noexcept;

// Transform3T
[[nodiscard]] static Transform3T RotationAbout(
    Point3T<Scalar> origin, UnitVector3T<Scalar> axis, Scalar angleRadians) noexcept;
```

`origin` 非有限时，仍构造变换，行为与现有 `Translation` + `Rotation` 复合一致（可能得到非有限矩阵；不特判）。文档注释说明：Prim 曲线协议里的 `Rotated(center, …)` 应调用这些工厂，而不是只绕世界原点转。

### 4.2 TransformNormal

```cpp
[[nodiscard]] std::optional<UnitVector3T<Scalar>> TransformNormal(
    UnitVector3T<Scalar> normal, Core::Tolerance tolerance = {}) const noexcept;
```

步骤：取 `Matrix` 左上角 3×3 为 `L`；`L.Inverse(tolerance)` 失败则空；`n' = inverse(L)^T * normal.AsVector()`；`n'.Normalized(tolerance)` 得到单位法向。反射（det −1）仍用同一公式，调用方负责语义是否仍叫「法线」。

---

## 5. Quaternion

```cpp
[[nodiscard]] static std::optional<QuaternionT<Scalar>> FromRotationMatrix(
    MatrixT<Scalar, 3> rotation, Core::Tolerance tolerance = {}) noexcept;
```

校验：`R^T R ≈ I`，`det(R) ≈ +1`（容差同 `Matrix::Inverse`）。提取算法用 Shepperd 或等价稳定分支，避免 trace 接近 −1 时的数值问题。不要求从 `Transform3` 的 4×4 一次提取旋转+平移。

可选便捷（与上同一次交付）：`Transform3T::RotationQuaternion()` 返回线性部分对应的四元数，仅当 3×3 在容差下为旋转时成功；否则 `std::nullopt`。若你认为与 `FromRotationMatrix` 重复，实现方案里二选一；本设计倾向**两个都提供**（一个静态、一个成员，测试各覆盖一次）。

---

## 6. Vector 与 Point

### 6.1 Lerp

```cpp
// Vector2T / Vector3T / Vector4T
[[nodiscard]] constexpr VectorNT Lerp(VectorNT other, Scalar t) const noexcept;

// Point2T / Point3T
[[nodiscard]] constexpr PointNT Lerp(PointNT other, Scalar t) const noexcept;
```

公式：`(*this) * (1 - t) + other * t`（点用向量差：`Point + (other - *this) * t`）。

### 6.2 夹角

```cpp
// UnitVector2T / UnitVector3T
[[nodiscard]] Scalar AngleBetween(UnitVectorNT other) const noexcept;

// 仅 UnitVector2T
[[nodiscard]] Scalar SignedAngle(UnitVector2T other) const noexcept;
```

---

## 7. 建议实现顺序

确认后按此顺序写实现计划，每步自带测试，步末可单独合并。

1. **Box 查询与 FromPoints** —— 阶段 4 恢复时 Query 立刻受益。
2. **Vector / Point Lerp** —— 小、独立，给后续 Prim 插值示例用。
3. **UnitVector 夹角** —— 二维带符号角锁定与 `Orient2d` 一致性的用例。
4. **RotationAbout** —— 2D 与 3D。
5. **TransformNormal + Quaternion FromRotationMatrix**（及可选 `RotationQuaternion`）。

---

## 8. 测试

沿用 `.cursor/rules/tests-with-changes.mdc`：每个新公开方法至少一条**直接调用**它的用例。

**Box：** 空盒三者皆空；点在内部距离 0；点在某一轴外只动该轴；角点/边/面最近点；`FromPoints` 空 span、`FromPoints` 含 NaN、`FromPoints` 单点退化盒；与 `Merged` 批量合并一致。

**Transform：** 绕非原点旋转后固定点不动；`TransformNormal` 在均匀缩放下与 `operator*(Vector)` 方向一致；非均匀缩放 + 逆变换 round-trip；奇异线性部分返回空。

**Quaternion：** 已知旋转矩阵 round-trip；反射矩阵（det −1）提取失败。

**Lerp / 角：** `t=0/1`；`SignedAngle` 与叉积符号；平行与反平行（`AngleBetween` 为 0 或 π）。

---

## 9. 与总览、阶段 4 的关系

- 总览 §4.6 法线逆转置：在本设计 §4.2 落地。
- 阶段 4 合同 §4 空盒与距离：在本设计 §3 落地；阶段 4 文档无需改语义，恢复实现时引用本文即可。
- 不在此设计中的 Linear 想法（例如 `OrientedBox` 最近点、`Transform` 分解 TRS）先不排期；若你后续提出，另开一节或新文档。

---

## 10. 确认之后

本文已确认。可选补写 `docs/superpowers/plans/2026-10-01-dragongeo-linear-enhancement.md` 作事后分任务记录；编码与测试已完成，行为以 §1–§8 与本文件状态行为准。
