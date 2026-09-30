#pragma once

#include <optional>

#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Point3.hpp>
#include <GeoCore/linear/Transform3.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 三维坐标系（标架）：一个原点加一组**正交单位**的右手轴。
///
/// 这是本阶段唯一的**强不变量**类型。正交性是坐标系的定义性质，不是可以事后
/// 修正的细节 —— 非正交的「坐标系」会让经它表达的变换静默地拉伸几何，因此
/// 构造函数私有，任何公开接口都无法产出违反不变量的值：
///
///   - `from_axes` 校验三轴长度为 1、两两正交、且构成右手系；
///   - `from_z_axis` 由一根轴补全另外两根，补全出来的轴按构造必然正交；
///   - `from_transform` **必须**校验线性部分本身是正交矩阵 —— 理由见该工厂。
///
/// 与 `UnitVector3T` 的对比：那边的不变量是**弱**的（|v| 在舍入误差内等于 1，
/// 而且 `from_normalized_unchecked` 公开可用，违反前置条件只是「用户自己的
/// 事」）；这边是**强**的 —— 一个错误的标架不会立即出错，只会让下游几何被
/// 悄悄拉伸，所以本类型连那种逃生舱都不提供。
///
/// 轴的**长度**校验看似多余（形参类型已经是 `UnitVector3T`），其实不然：
/// `UnitVector3T` 的不变量是弱不变量，而 `from_normalized_unchecked` 是公开的。
/// 不校验长度就等于留了一条公开的、能构造出「把几何拉伸 2 倍」的坐标系的路径，
/// 实测 `{2,0,0}` 与 `{0,0.5,0}` 这样一伸一缩的组合能同时通过两两点积与定向
/// 检查（叉积长度被补了回来）。
///
/// **右手性是接口的一部分**（`x × y` 与 `z` 同向）。`from_axes` 拒绝左手系，
/// 因此 `from_transform` 也必须拒绝反射矩阵：那是正交但 det = −1 的线性部分，
/// 放行它等于给同一个不变量留两种说法。
///
/// 所有容差都由调用者显式传入，默认值来自 `core::Tolerance`，函数体内不硬编码
/// 阈值。默认容差按 double 定标；float 实例化请显式传入与该精度相称的容差。
///
/// 注意本类型只承诺**线性**部分是正交的：`Transform3T` 的第 4 行（`(0,0,0,1)`）
/// 是那边的契约，本类型既不读取也不校验。
template <typename Scalar>
class Coordinate3T {
public:
    using scalar_type = Scalar;

    /// 标准标架：原点在 (0,0,0)，三轴取标准基。
    [[nodiscard]] static constexpr Coordinate3T identity() noexcept {
        return Coordinate3T{
            Point3T<Scalar>{},
            UnitVector3T<Scalar>::from_normalized_unchecked(
                Vector3T<Scalar>{Scalar{1}, Scalar{0}, Scalar{0}}),
            UnitVector3T<Scalar>::from_normalized_unchecked(
                Vector3T<Scalar>{Scalar{0}, Scalar{1}, Scalar{0}}),
            UnitVector3T<Scalar>::from_normalized_unchecked(
                Vector3T<Scalar>{Scalar{0}, Scalar{0}, Scalar{1}})};
    }

    /// 由原点与三根轴构造，校验**正交、单位长、右手**。
    ///
    /// 校验的是 `AᵀA ≈ I`（六个互异的点积：三个对角元为 1、三个非对角元为 0）
    /// 加上定向 `(x × y) · z ≈ +1`。判据是**轴本身**，不是「先把每根轴归一化
    /// 再判它们两两正交」—— 后者有一个隐蔽的洞：`diag(2,3,4)` 与 `diag(2,2,2)`
    /// 归一化之后都变成标准基，看起来完全正交，于是最该拒绝的非均匀缩放会
    /// 静默通过（`from_transform` 的那条测试就是它的死亡证明）。
    ///
    /// 定向用标量形式 `(x × y) · z ≈ +1`，而不是逐分量比较 `x × y` 与 `z`：
    /// 对单位向量两者等价，但逐分量比较会让三条点积**失去独立见证** ——
    /// 那些「本应为零」的分量走的是 `equal(0, v)`，参考量级塌到 `|v|`，
    /// 于是容差退化成 abs，正好与点积检查重合。**实测**：把定向换成逐分量
    /// 比较之后，「正交检查只留 `x·y`」这个变异体在全部 296 条断言下存活；
    /// 用标量形式时它被杀死 —— 由 `x·z` 与 `y·z` **两条**点积各自的证据
    /// （`x·y` 那一格恰好是它自己保留的，杀不了它）。
    ///
    /// 任一项不满足返回 `std::nullopt`。容差参数贯穿全部检查（测试里有一格：
    /// 同一份输入、默认容差拒绝、显式放松后接受）。
    [[nodiscard]] static constexpr std::optional<Coordinate3T> from_axes(
        Point3T<Scalar> origin,
        UnitVector3T<Scalar> x,
        UnitVector3T<Scalar> y,
        UnitVector3T<Scalar> z,
        core::Tolerance tolerance = {}) noexcept {
        const bool unit_lengths = tolerance.equal(x.dot(x), 1.0)
                               && tolerance.equal(y.dot(y), 1.0)
                               && tolerance.equal(z.dot(z), 1.0);

        const bool orthogonal = tolerance.is_zero(x.dot(y))
                             && tolerance.is_zero(x.dot(z))
                             && tolerance.is_zero(y.dot(z));

        // `x × y` 与 `z` 同向。三根轴此时已各自是单位向量（上面那一行），
        // 于是「同向」等价于 `(x × y) · z ≈ +1`；左手系给出 −1。
        const bool right_handed =
            tolerance.equal(x.cross(y).dot(z.as_vector()), 1.0);

        if (!unit_lengths || !orthogonal || !right_handed) {
            return std::nullopt;
        }
        return Coordinate3T{origin, x, y, z};
    }

    /// 由 z 轴补全一组正交的 x / y。
    ///
    /// 参考向量取「z 的绝对值最小的那个分量方向」—— 该方向与 z 的夹角必然
    /// 不小于 54.7°（三个分量平方和为 1，最小者必不超过 1/√3），因此叉积不会
    /// 退化成零向量。若固定用 (0,0,1) 作参考，z 接近 z 轴时就会失败：z 恰好是
    /// (0,0,1) 时叉积为零向量，`normalized` 返回 `nullopt` —— 本文件里那条
    /// 往返用例（z 取世界 z 轴）就是这一格的死亡证明。
    ///
    /// 与 `from_axes` 一样返回 `optional`（补全出来的轴理论上不可能退化，
    /// 但那要押在参考向量的选择上；交给同一个校验出口更诚实）。
    [[nodiscard]] static std::optional<Coordinate3T> from_z_axis(
        Point3T<Scalar> origin, UnitVector3T<Scalar> z,
        core::Tolerance tolerance = {}) noexcept;

    /// 由刚体变换（旋转 + 平移）构造标架。
    ///
    /// 原点取变换作用于局部原点的结果；三根轴取线性部分的**三列**。
    ///
    /// **返回 `optional` 而不是裸值**：计划原先写的是「线性部分须为正交
    /// （由 `Transform` 的工厂保证）」，这句话是假的 —— `Transform3T::scaling`
    /// 就在工厂里，`translation * rotation * scaling` 这样的复合更是把非正交
    /// 直接喂进来。若不校验，这里就是一条**公开的、能构造出非正交坐标系的
    /// 路径**，而本类型存在的全部意义就是堵死它。
    ///
    /// 判据同样是「线性部分本身是正交矩阵」，不是「把三列各自归一化之后再判
    /// 它们两两正交」：后者会让 `diag(2,3,4)` 与 `diag(2,2,2)` 静默通过
    /// （归一化之后都是标准基），而它们正是最该被拒绝的非均匀缩放。
    /// 实现上把三列原样交给 `from_axes`，由它完成 `AᵀA ≈ I` 与定向的校验 ——
    /// 于是「反射」（正交但 det = −1）也在同一个出口被拒绝。
    [[nodiscard]] static constexpr std::optional<Coordinate3T> from_transform(
        const Transform3T<Scalar>& transform, core::Tolerance tolerance = {}) noexcept {
        const MatrixT<Scalar, 4>& m = transform.matrix;

        // 线性部分的三列。**原样**取出，不做归一化 —— 归一化会把判据偷换成
        // 「方向正交」，那是本类型要拒绝的那种标架。
        const UnitVector3T<Scalar> x = UnitVector3T<Scalar>::from_normalized_unchecked(
            Vector3T<Scalar>{m.data[0][0], m.data[1][0], m.data[2][0]});
        const UnitVector3T<Scalar> y = UnitVector3T<Scalar>::from_normalized_unchecked(
            Vector3T<Scalar>{m.data[0][1], m.data[1][1], m.data[2][1]});
        const UnitVector3T<Scalar> z = UnitVector3T<Scalar>::from_normalized_unchecked(
            Vector3T<Scalar>{m.data[0][2], m.data[1][2], m.data[2][2]});

        return from_axes(transform.transform_point(Point3T<Scalar>{}), x, y, z, tolerance);
    }

    /// 原点。
    [[nodiscard]] constexpr Point3T<Scalar> origin() const noexcept { return origin_; }

    [[nodiscard]] constexpr UnitVector3T<Scalar> x_axis() const noexcept { return x_; }
    [[nodiscard]] constexpr UnitVector3T<Scalar> y_axis() const noexcept { return y_; }
    [[nodiscard]] constexpr UnitVector3T<Scalar> z_axis() const noexcept { return z_; }

    /// 把局部坐标的**点**送到父坐标系：`origin + x·lx + y·ly + z·lz`。
    [[nodiscard]] constexpr Point3T<Scalar> to_parent(Point3T<Scalar> local) const noexcept {
        return Point3T<Scalar>{
            origin_.x + x_.x() * local.x + y_.x() * local.y + z_.x() * local.z,
            origin_.y + x_.y() * local.x + y_.y() * local.y + z_.y() * local.z,
            origin_.z + x_.z() * local.x + y_.z() * local.y + z_.z() * local.z};
    }

    /// 把父坐标系的**点**送回局部：先减去原点，再逐轴投影。
    [[nodiscard]] constexpr Point3T<Scalar> to_local(Point3T<Scalar> parent) const noexcept {
        const Vector3T<Scalar> offset = parent - origin_;
        return Point3T<Scalar>{x_.as_vector().dot(offset), y_.as_vector().dot(offset),
                               z_.as_vector().dot(offset)};
    }

    /// 把局部坐标的**方向**送到父坐标系：只施加线性部分。
    ///
    /// 与 `to_parent(Point3T)` 的差别是平移不生效 —— 方向不是位置。这两个重载
    /// 的期望值在测试里刻意取得不同（同一个 (1,2,3) 得到 (8,21,33) 与 (-2,1,3)），
    /// 把原点漏进方向重载的实现无处可藏。
    [[nodiscard]] constexpr Vector3T<Scalar> to_parent(Vector3T<Scalar> local) const noexcept {
        return x_.as_vector() * local.x + y_.as_vector() * local.y + z_.as_vector() * local.z;
    }

    /// 把父坐标系的**方向**送回局部：逐轴投影，不平移。
    [[nodiscard]] constexpr Vector3T<Scalar> to_local(Vector3T<Scalar> parent) const noexcept {
        return Vector3T<Scalar>{x_.as_vector().dot(parent), y_.as_vector().dot(parent),
                                z_.as_vector().dot(parent)};
    }

private:
    /// 唯一构造函数，私有：不变量由三个工厂负责，外部没有别的入口。
    /// （没有默认构造 —— 三个工厂才是全部的公开构造路径。）
    explicit constexpr Coordinate3T(Point3T<Scalar> origin, UnitVector3T<Scalar> x,
                                    UnitVector3T<Scalar> y,
                                    UnitVector3T<Scalar> z) noexcept
        : origin_(origin), x_(x), y_(y), z_(z) {}

    Point3T<Scalar> origin_;
    UnitVector3T<Scalar> x_;
    UnitVector3T<Scalar> y_;
    UnitVector3T<Scalar> z_;
};

using Coordinate3 = Coordinate3T<double>;
using Coordinate3f = Coordinate3T<float>;

template <typename Scalar>
std::optional<Coordinate3T<Scalar>> Coordinate3T<Scalar>::from_z_axis(
    Point3T<Scalar> origin, UnitVector3T<Scalar> z, core::Tolerance tolerance) noexcept {
    const Scalar ax = core::absolute_value(z.x());
    const Scalar ay = core::absolute_value(z.y());
    const Scalar az = core::absolute_value(z.z());

    Vector3T<Scalar> reference{};
    if (ax <= ay && ax <= az) {
        reference = Vector3T<Scalar>{Scalar{1}, Scalar{0}, Scalar{0}};
    } else if (ay <= az) {
        reference = Vector3T<Scalar>{Scalar{0}, Scalar{1}, Scalar{0}};
    } else {
        reference = Vector3T<Scalar>{Scalar{0}, Scalar{0}, Scalar{1}};
    }

    const auto x = reference.cross(z.as_vector()).normalized(tolerance);
    if (!x.has_value()) {
        return std::nullopt;
    }
    // y = z × x：与 x 正交、与 z 正交，且 x × y = z（右手系）。
    const auto y = z.cross(*x).normalized(tolerance);
    if (!y.has_value()) {
        return std::nullopt;
    }
    return Coordinate3T<Scalar>::from_axes(origin, *x, *y, z, tolerance);
}

// ---- 运算符 ----

/// 四个成员都相等才算相等（原点 + 三根轴）。与 `UnitVector` / `Point` / `Box`
/// 的约定一致：自由函数，`!=` 由 C++20 自动生成，不手写。
///
/// 逐分量精确比较，不带容差 —— 两个「几乎相同」的标架不相等，这一点是刻意的：
/// 容差比较会让 `==` 的含义随上下文变化。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(const Coordinate3T<Scalar>& a,
                                        const Coordinate3T<Scalar>& b) noexcept {
    return a.origin() == b.origin() && a.x_axis() == b.x_axis()
        && a.y_axis() == b.y_axis() && a.z_axis() == b.z_axis();
}

} // namespace GeoCore::linear
