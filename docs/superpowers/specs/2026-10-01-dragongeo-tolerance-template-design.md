# DragonGeo Core：ToleranceT 模板化

日期：2026-10-01  
状态：**已确认并实现**。谓词层仍只用 `Tolerance`（`double`）。

---

## 1. 动机

Linear 类型已是 `*T<Scalar>`（`double` / `float`），但容差长期固定为 `double` 的 `Tolerance`，float 路径在实现里常 `static_cast<double>` 再比较。模板化后：**同一标量域内**做容差判定，float 默认阈值与 float 精度匹配。

---

## 2. 类型与别名

```cpp
template <std::floating_point Scalar>
struct ToleranceT {
    Scalar Abs = DefaultToleranceAbs<Scalar>();
    Scalar Rel = DefaultToleranceRel<Scalar>();
    // Resolve / Equal / IsZero，参数与返回值均为 Scalar
};

using Tolerance  = ToleranceT<double>;
using Tolerancef = ToleranceT<float>;
```

**默认常数**

| Scalar | Abs | Rel | 说明 |
|--------|-----|-----|------|
| `double` | `1e-12` | `1e-9` | 与改前 `struct Tolerance` 一致 |
| `float` | `1e-6f` | `1e-5f` | 约 float epsilon 量级，供 `Vector3f` 等默认 `{}` |

不提供 `ToleranceT<double>` → `ToleranceT<float>` 的隐式转换；混精度时调用方显式构造对应类型的容差。

---

## 3. 签名变更（`Core::Tolerance` → `Core::ToleranceT<Scalar>`）

凡带 `Scalar` 模板且接受容差的 **Linear** API：

- `Vector2T` / `Vector3T`：`Normalized`
- `MatrixT<Scalar, N>`：`Inverse`
- `QuaternionT`：`Normalized`、`FromRotationMatrix`
- `Transform2T` / `Transform3T`：`Inverse`、`TransformNormal`、`RotationQuaternion`
- `Coordinate2T` / `Coordinate3T`：`FromAxes`、`FromXAxis` / `FromZAxis`、`FromTransform`
- `OrientedBox2T` / `OrientedBox3T`：`Contains`

**不改**

- `Box` / `Interval`：仍不用容差。
- **Predicates**（`Orient2d` 等）：仅 `double`，无容差参数或将来若加仍用 `Tolerance`。

---

## 4. 与总览的关系

总览 §4.2 中的 `struct Tolerance { double … }` 视为 **`Tolerance` = `ToleranceT<double>`** 的叙述；float 别名与默认常数以本文为准。
