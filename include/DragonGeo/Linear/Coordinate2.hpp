#pragma once

#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Point2.hpp>
#include <DragonGeo/Linear/Transform2.hpp>
#include <DragonGeo/Linear/UnitVector2.hpp>
#include <DragonGeo/Linear/Vector2.hpp>

namespace DragonGeo::Linear {

/// 二维坐标系（标架）：一个原点加一组**正交单位**的右手轴。
///
/// 语义与 `Coordinate3T` 完全一致（强不变量类型：私有构造 + 校验过的工厂，
/// 非正交的标架会让经它表达的变换静默地拉伸几何），差异仅在维度，以及二维
/// 必然不同的两处：
///
///   - `FromAxes` 只有两根轴，校验两两正交、长度为 1，以及标量叉积**为正**
///     （二维没有第三根轴可比，右手 ⟺ 逆时针 ⟺ `x × y > 0`）；
///   - 没有 `FromZAxis`，代之以 `FromXAxis`（主轴是 x，y 由 x 逆时针转 90°
///     补全，见该工厂）。两者形态相同：`(origin, axis, Tolerance = {})` →
///     `std::optional<Coordinate2T>`。
///
/// **两个工厂都不产出裸值** —— 公开接口里没有一条能绕过校验的路径。
///
/// 长度校验在二维尤其不能省：定向检查只看叉积的**符号**，均匀缩放不会改变它，
/// 于是「`{2,0}` 配 `{0,3}`」这种拉伸标架能顺利通过正交与定向两项。
template <typename Scalar>
class Coordinate2T {
public:
    using ScalarType = Scalar;

    /// 标准标架：原点在 (0,0)，两轴取标准基。
    [[nodiscard]] static constexpr Coordinate2T Identity() noexcept {
        return Coordinate2T{
            Point2T<Scalar>{},
            UnitVector2T<Scalar>::FromNormalizedUnchecked(
                Vector2T<Scalar>{Scalar{1}, Scalar{0}}),
            UnitVector2T<Scalar>::FromNormalizedUnchecked(
                Vector2T<Scalar>{Scalar{0}, Scalar{1}})};
    }

    /// 由原点与两根轴构造，校验**正交、单位长、逆时针**。
    ///
    /// 判据：`x·y ≈ 0`、`|x| ≈ |y| ≈ 1`、`x × y > 0`。前两项判「是不是一组正交
    /// 单位轴」，第三项判定向。三者互不遮蔽 —— 每一格都有只由其中一条决定的
    /// 测试（长度那两条尤其：叉积是纯符号判断，拉伸/压缩在它眼里完全一样）。
    ///
    /// 任一项不满足返回 `std::nullopt`。容差参数贯穿长度与正交检查；
    /// 定向是符号判断，不需要容差（正交单位轴对上 `|x × y| = 1`）。
    [[nodiscard]] static constexpr std::optional<Coordinate2T> FromAxes(
        Point2T<Scalar> origin, UnitVector2T<Scalar> x, UnitVector2T<Scalar> y,
        Core::Tolerance tolerance = {}) noexcept {
        const bool unitLengths =
            tolerance.Equal(x.Dot(x), 1.0) && tolerance.Equal(y.Dot(y), 1.0);
        const bool orthogonal = tolerance.IsZero(x.Dot(y));
        const bool counterClockwise = x.Cross(y) > Scalar{0};

        if (!unitLengths || !orthogonal || !counterClockwise) {
            return std::nullopt;
        }
        return Coordinate2T{origin, x, y};
    }

    /// 由 x 轴补全 y 轴：`y` 就是 `x` 逆时针转 90°，即 `(-x.Y, x.X)`。
    ///
    /// 与三维的 `FromZAxis` **结构一致**：补全出 y 之后交给 `FromAxes` 校验
    /// 并返回 `optional`，而不是直接返回裸值。
    ///
    /// 这里同样不能省掉校验 —— 补全公式只用得到 `x` 的**方向**，对长度一无所知：
    /// `UnitVector2T` 的不变量是弱不变量，而 `FromNormalizedUnchecked` 是公开的，
    /// 于是 `FromNormalizedUnchecked({2,0})` 会补出一组长度都是 2 的轴，
    /// 得到一个把几何拉伸 2 倍的「坐标系」。那是本类型存在的全部意义所在，
    /// 不能因为「正常调用者不会这么传」就放过（三维那边同样拒绝非单位的 z）。
    /// 长度校验在 `FromAxes` 里，这里不做第二次。
    ///
    /// **与三维的一处规格不对称（刻意保留，不要为了「对称」改任何一边）**：
    /// 本工厂的补全**不做归一化**（`y = (-x.Y, x.X)` 只是旋转），而三维的
    /// `FromZAxis` 走的是 `reference.Cross(z).Normalized(tolerance)` ——
    /// 那里的 `IsZero(length)` 会在退化容差下把中间叉积的长度（`z = (1,1,1)/√3`
    /// 时是 0.8165）当成零吞掉。于是**在退化容差（`{0.009, 0.99}` 与 `{1, 1}` 量级）
    /// 下，三维比二维更严**：同一档容差里 3D 会多返回一些 `nullopt`。方向是安全的
    /// （只会多拒绝，不会放进非法标架），默认容差与任何正常容差下两者**零分歧**。
    /// 3D 那边更严是它的实现路径使然，不是缺陷。
    [[nodiscard]] static constexpr std::optional<Coordinate2T> FromXAxis(
        Point2T<Scalar> origin, UnitVector2T<Scalar> x,
        Core::Tolerance tolerance = {}) noexcept {
        const UnitVector2T<Scalar> y =
            UnitVector2T<Scalar>::FromNormalizedUnchecked(Vector2T<Scalar>{-x.Y(), x.X()});
        return FromAxes(origin, x, y, tolerance);
    }

    /// 由刚体变换（旋转 + 平移）构造标架。
    ///
    /// 原点取变换作用于局部原点的结果；两根轴取线性部分的**两列**。
    ///
    /// 与三维的 `FromTransform` 同一条理由：`Transform2T::scaling` 就在工厂里，
    /// 不校验的话这里就是一条公开的、能构造出非正交坐标系的路径。判据是
    /// 「线性部分本身是正交矩阵」，不是「把两列各自归一化之后再判它们正交」——
    /// 后者会让 `Scaling(2.0)` 静默通过（归一化之后就是标准基）。二维的死亡
    /// 证明比三维更直接：`diag(2,2)` 的叉积是 +4，**符号仍是正的**，只有
    /// 长度检查拦得住它。
    ///
    /// 反射（正交但 det = −1，例如 `Scaling({1,-1})`）同样被拒绝：`FromAxes`
    /// 要求叉积为正，同一个不变量不能有两个说法。
    [[nodiscard]] static constexpr std::optional<Coordinate2T> FromTransform(
        const Transform2T<Scalar>& transform, Core::Tolerance tolerance = {}) noexcept {
        const MatrixT<Scalar, 3>& m = transform.Matrix;

        // 线性部分的两列，**原样**取出（不归一化，理由同上）。
        const UnitVector2T<Scalar> x = UnitVector2T<Scalar>::FromNormalizedUnchecked(
            Vector2T<Scalar>{m.Data[0][0], m.Data[1][0]});
        const UnitVector2T<Scalar> y = UnitVector2T<Scalar>::FromNormalizedUnchecked(
            Vector2T<Scalar>{m.Data[0][1], m.Data[1][1]});

        return FromAxes(transform.TransformPoint(Point2T<Scalar>{}), x, y, tolerance);
    }

    /// 原点。
    [[nodiscard]] constexpr Point2T<Scalar> Origin() const noexcept { return m_origin; }

    [[nodiscard]] constexpr UnitVector2T<Scalar> XAxis() const noexcept { return m_x; }
    [[nodiscard]] constexpr UnitVector2T<Scalar> YAxis() const noexcept { return m_y; }

    /// 把局部坐标的**点**送到父坐标系：`origin + x·lx + y·ly`。
    [[nodiscard]] constexpr Point2T<Scalar> ToParent(Point2T<Scalar> local) const noexcept {
        return Point2T<Scalar>{m_origin.X + m_x.X() * local.X + m_y.X() * local.Y,
                               m_origin.Y + m_x.Y() * local.X + m_y.Y() * local.Y};
    }

    /// 把父坐标系的**点**送回局部：先减去原点，再逐轴投影。
    [[nodiscard]] constexpr Point2T<Scalar> ToLocal(Point2T<Scalar> parent) const noexcept {
        const Vector2T<Scalar> offset = parent - m_origin;
        return Point2T<Scalar>{m_x.AsVector().Dot(offset), m_y.AsVector().Dot(offset)};
    }

    /// 把局部坐标的**方向**送到父坐标系：只施加线性部分（不平移）。
    [[nodiscard]] constexpr Vector2T<Scalar> ToParent(Vector2T<Scalar> local) const noexcept {
        return m_x.AsVector() * local.X + m_y.AsVector() * local.Y;
    }

    /// 把父坐标系的**方向**送回局部：逐轴投影，不平移。
    [[nodiscard]] constexpr Vector2T<Scalar> ToLocal(Vector2T<Scalar> parent) const noexcept {
        return Vector2T<Scalar>{m_x.AsVector().Dot(parent), m_y.AsVector().Dot(parent)};
    }

private:
    /// 唯一构造函数，私有：不变量由三个工厂负责。
    explicit constexpr Coordinate2T(Point2T<Scalar> origin, UnitVector2T<Scalar> x,
                                    UnitVector2T<Scalar> y) noexcept
        : m_origin(origin), m_x(x), m_y(y) {}

    Point2T<Scalar> m_origin;
    UnitVector2T<Scalar> m_x;
    UnitVector2T<Scalar> m_y;
};

using Coordinate2 = Coordinate2T<double>;
using Coordinate2f = Coordinate2T<float>;

// ---- 运算符 ----

/// 三个成员都相等才算相等（原点 + 两根轴）。自由函数，`!=` 由 C++20 自动生成。
/// 与三维一致：逐分量精确比较，不带容差。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(const Coordinate2T<Scalar>& a,
                                        const Coordinate2T<Scalar>& b) noexcept {
    return a.Origin() == b.Origin() && a.XAxis() == b.XAxis() && a.YAxis() == b.YAxis();
}

} // namespace DragonGeo::Linear
