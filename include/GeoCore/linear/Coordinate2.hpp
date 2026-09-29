#pragma once

#include <optional>

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Point2.hpp>
#include <GeoCore/linear/Transform2.hpp>
#include <GeoCore/linear/UnitVector2.hpp>
#include <GeoCore/linear/Vector2.hpp>

namespace GeoCore::linear {

/// 二维坐标系（标架）：一个原点加一组**正交单位**的右手轴。
///
/// 语义与 `Coordinate3T` 完全一致（强不变量类型：私有构造 + 校验过的工厂，
/// 非正交的标架会让经它表达的变换静默地拉伸几何），差异仅在维度，以及二维
/// 必然不同的两处：
///
///   - `from_axes` 只有两根轴，校验两两正交、长度为 1，以及标量叉积**为正**
///     （二维没有第三根轴可比，右手 ⟺ 逆时针 ⟺ `x × y > 0`）；
///   - 没有 `from_z_axis`，代之以 `from_x_axis`（主轴是 x，y 由 x 逆时针转 90°
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
    using scalar_type = Scalar;

    /// 标准标架：原点在 (0,0)，两轴取标准基。
    [[nodiscard]] static constexpr Coordinate2T identity() noexcept {
        return Coordinate2T{
            Point2T<Scalar>{},
            UnitVector2T<Scalar>::from_normalized_unchecked(
                Vector2T<Scalar>{Scalar{1}, Scalar{0}}),
            UnitVector2T<Scalar>::from_normalized_unchecked(
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
    [[nodiscard]] static constexpr std::optional<Coordinate2T> from_axes(
        Point2T<Scalar> origin, UnitVector2T<Scalar> x, UnitVector2T<Scalar> y,
        core::Tolerance tolerance = {}) noexcept {
        const bool unit_lengths =
            tolerance.equal(x.dot(x), 1.0) && tolerance.equal(y.dot(y), 1.0);
        const bool orthogonal = tolerance.is_zero(x.dot(y));
        const bool counter_clockwise = x.cross(y) > Scalar{0};

        if (!unit_lengths || !orthogonal || !counter_clockwise) {
            return std::nullopt;
        }
        return Coordinate2T{origin, x, y};
    }

    /// 由 x 轴补全 y 轴：`y` 就是 `x` 逆时针转 90°，即 `(-x.y, x.x)`。
    ///
    /// 与三维的 `from_z_axis` **结构一致**：补全出 y 之后交给 `from_axes` 校验
    /// 并返回 `optional`，而不是直接返回裸值。
    ///
    /// 这里同样不能省掉校验 —— 补全公式只用得到 `x` 的**方向**，对长度一无所知：
    /// `UnitVector2T` 的不变量是弱不变量，而 `from_normalized_unchecked` 是公开的，
    /// 于是 `from_normalized_unchecked({2,0})` 会补出一组长度都是 2 的轴，
    /// 得到一个把几何拉伸 2 倍的「坐标系」。那是本类型存在的全部意义所在，
    /// 不能因为「正常调用者不会这么传」就放过（三维那边同样拒绝非单位的 z）。
    /// 长度校验在 `from_axes` 里，这里不做第二次。
    [[nodiscard]] static constexpr std::optional<Coordinate2T> from_x_axis(
        Point2T<Scalar> origin, UnitVector2T<Scalar> x,
        core::Tolerance tolerance = {}) noexcept {
        const UnitVector2T<Scalar> y =
            UnitVector2T<Scalar>::from_normalized_unchecked(Vector2T<Scalar>{-x.y(), x.x()});
        return from_axes(origin, x, y, tolerance);
    }

    /// 由刚体变换（旋转 + 平移）构造标架。
    ///
    /// 原点取变换作用于局部原点的结果；两根轴取线性部分的**两列**。
    ///
    /// 与三维的 `from_transform` 同一条理由：`Transform2T::scaling` 就在工厂里，
    /// 不校验的话这里就是一条公开的、能构造出非正交坐标系的路径。判据是
    /// 「线性部分本身是正交矩阵」，不是「把两列各自归一化之后再判它们正交」——
    /// 后者会让 `scaling(2.0)` 静默通过（归一化之后就是标准基）。二维的死亡
    /// 证明比三维更直接：`diag(2,2)` 的叉积是 +4，**符号仍是正的**，只有
    /// 长度检查拦得住它。
    ///
    /// 反射（正交但 det = −1，例如 `scaling({1,-1})`）同样被拒绝：`from_axes`
    /// 要求叉积为正，同一个不变量不能有两个说法。
    [[nodiscard]] static constexpr std::optional<Coordinate2T> from_transform(
        const Transform2T<Scalar>& transform, core::Tolerance tolerance = {}) noexcept {
        const MatrixT<Scalar, 3>& m = transform.matrix;

        // 线性部分的两列，**原样**取出（不归一化，理由同上）。
        const UnitVector2T<Scalar> x = UnitVector2T<Scalar>::from_normalized_unchecked(
            Vector2T<Scalar>{m.data[0][0], m.data[1][0]});
        const UnitVector2T<Scalar> y = UnitVector2T<Scalar>::from_normalized_unchecked(
            Vector2T<Scalar>{m.data[0][1], m.data[1][1]});

        const Vector2T<Scalar> moved = transform.apply(Vector2T<Scalar>{});
        return from_axes(Point2T<Scalar>{moved.x, moved.y}, x, y, tolerance);
    }

    /// 原点。
    [[nodiscard]] constexpr Point2T<Scalar> origin() const noexcept { return origin_; }

    [[nodiscard]] constexpr UnitVector2T<Scalar> x_axis() const noexcept { return x_; }
    [[nodiscard]] constexpr UnitVector2T<Scalar> y_axis() const noexcept { return y_; }

    /// 把局部坐标的**点**送到父坐标系：`origin + x·lx + y·ly`。
    [[nodiscard]] constexpr Point2T<Scalar> to_parent(Point2T<Scalar> local) const noexcept {
        return Point2T<Scalar>{origin_.x + x_.x() * local.x + y_.x() * local.y,
                               origin_.y + x_.y() * local.x + y_.y() * local.y};
    }

    /// 把父坐标系的**点**送回局部：先减去原点，再逐轴投影。
    [[nodiscard]] constexpr Point2T<Scalar> to_local(Point2T<Scalar> parent) const noexcept {
        const Vector2T<Scalar> offset = parent - origin_;
        return Point2T<Scalar>{x_.as_vector().dot(offset), y_.as_vector().dot(offset)};
    }

    /// 把局部坐标的**方向**送到父坐标系：只施加线性部分（不平移）。
    [[nodiscard]] constexpr Vector2T<Scalar> to_parent(Vector2T<Scalar> local) const noexcept {
        return x_.as_vector() * local.x + y_.as_vector() * local.y;
    }

    /// 把父坐标系的**方向**送回局部：逐轴投影，不平移。
    [[nodiscard]] constexpr Vector2T<Scalar> to_local(Vector2T<Scalar> parent) const noexcept {
        return Vector2T<Scalar>{x_.as_vector().dot(parent), y_.as_vector().dot(parent)};
    }

private:
    /// 唯一构造函数，私有：不变量由三个工厂负责。
    explicit constexpr Coordinate2T(Point2T<Scalar> origin, UnitVector2T<Scalar> x,
                                    UnitVector2T<Scalar> y) noexcept
        : origin_(origin), x_(x), y_(y) {}

    Point2T<Scalar> origin_;
    UnitVector2T<Scalar> x_;
    UnitVector2T<Scalar> y_;
};

using Coordinate2 = Coordinate2T<double>;
using Coordinate2f = Coordinate2T<float>;

// ---- 运算符 ----

/// 三个成员都相等才算相等（原点 + 两根轴）。自由函数，`!=` 由 C++20 自动生成。
/// 与三维一致：逐分量精确比较，不带容差。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(const Coordinate2T<Scalar>& a,
                                        const Coordinate2T<Scalar>& b) noexcept {
    return a.origin() == b.origin() && a.x_axis() == b.x_axis() && a.y_axis() == b.y_axis();
}

} // namespace GeoCore::linear
