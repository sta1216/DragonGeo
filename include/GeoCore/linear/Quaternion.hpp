#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <GeoCore/core/Constants.hpp>
#include <GeoCore/core/Numeric.hpp>
#include <GeoCore/core/Tolerance.hpp>
#include <GeoCore/linear/Matrix.hpp>
#include <GeoCore/linear/UnitVector3.hpp>
#include <GeoCore/linear/Vector3.hpp>

namespace GeoCore::linear {

/// 四元数，表示三维旋转。
///
/// 成员顺序为标量部分在前（w, x, y, z）。默认构造得到**单位四元数**
/// 而非全零 —— 本类型表示旋转，默认值必须表示恒等旋转，否则默认构造
/// 的对象会静默地成为一个不可用的零旋转。
template <typename Scalar>
struct QuaternionT {
    using scalar_type = Scalar;

    Scalar w{1};
    Scalar x{};
    Scalar y{};
    Scalar z{};

    [[nodiscard]] constexpr bool operator==(const QuaternionT&) const noexcept = default;
};

using Quaternion = QuaternionT<double>;
using Quaternionf = QuaternionT<float>;

/// 单位四元数，即恒等旋转。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> identity_quaternion() noexcept {
    return QuaternionT<Scalar>{};
}

/// 共轭。对单位四元数而言即为逆旋转。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> conjugate(QuaternionT<Scalar> q) noexcept {
    return QuaternionT<Scalar>{q.w, -q.x, -q.y, -q.z};
}

/// 四元数内积。
template <typename Scalar>
[[nodiscard]] constexpr Scalar dot(QuaternionT<Scalar> a, QuaternionT<Scalar> b) noexcept {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/// 模长。
template <typename Scalar>
[[nodiscard]] Scalar norm(QuaternionT<Scalar> q) noexcept {
    // 先按最大分量缩放，避免中间量上溢或下溢。
    const Scalar scale = core::max_abs_of(q.w, q.x, q.y, q.z);

    if (scale == Scalar{0}) {
        return Scalar{0};
    }
    if (!core::is_finite(scale)) {
        // 与 Vector3T::length 保持一致，规则同为：任一无穷分量 ⇒ ±inf；
        // 否则含 NaN ⇒ NaN。两条路径的语义必须一致，否则调用方在模长的
        // 两个来源上会得到互相矛盾的结果。
        return scale;
    }
    const Scalar sw = q.w / scale;
    const Scalar sx = q.x / scale;
    const Scalar sy = q.y / scale;
    const Scalar sz = q.z / scale;
    return scale * std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
}

/// 归一化。模长在给定容差下可视为零、或本身不是有限值时返回 std::nullopt。
template <typename Scalar>
[[nodiscard]] std::optional<QuaternionT<Scalar>> normalize(
    QuaternionT<Scalar> q, core::Tolerance tolerance = {}) noexcept {
    const Scalar magnitude = norm(q);
    // 与 UnitVector::normalize / Matrix::inverse 同一条原则：绝不交出一个
    // has_value() 为真、内容却是 NaN 的结果 —— 调用者无从察觉，而 NaN 会
    // 污染后续全部计算。
    if (!core::is_finite(static_cast<double>(magnitude))
        || tolerance.is_zero(static_cast<double>(magnitude))) {
        return std::nullopt;
    }
    const Scalar inverse_magnitude = Scalar{1} / magnitude;
    const QuaternionT<Scalar> result{
        q.w * inverse_magnitude,
        q.x * inverse_magnitude,
        q.y * inverse_magnitude,
        q.z * inverse_magnitude,
    };
    // 模长有限并不保证倒数有限：|q| 是次正规数时（例如 5e-324）1/|q| 就是
    // inf，分量乘上去即得 inf。与 Matrix::inverse 还原尺度后的检查是同一件
    // 事 —— 绝不交出 has_value() 为真、内容却是 inf 的结果。
    if (!core::is_finite(static_cast<double>(result.w))
        || !core::is_finite(static_cast<double>(result.x))
        || !core::is_finite(static_cast<double>(result.y))
        || !core::is_finite(static_cast<double>(result.z))) {
        return std::nullopt;
    }
    return result;
}

/// 四元数乘法，对应旋转的复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> a, QuaternionT<Scalar> b) noexcept {
    return QuaternionT<Scalar>{
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> q, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return QuaternionT<Scalar>{q.w * scale, q.x * scale, q.y * scale, q.z * scale};
}

/// 标量 × 四元数。与 operator*(quaternion, factor) 对称 —— 缺了它
/// `2.0 * q` 会编译失败，而 Vector 与 UnitVector 都提供两种写法。
template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    Factor factor, QuaternionT<Scalar> q) noexcept {
    return q * factor;
}

/// 逐分量取负。q 与 -q 表示同一个旋转，故取负不影响旋转语义。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator-(QuaternionT<Scalar> q) noexcept {
    return QuaternionT<Scalar>{-q.w, -q.x, -q.y, -q.z};
}

/// 由单位轴与弧度角构造旋转。
///
/// 轴参数刻意接受 UnitVector3T 而非 Vector3T：非单位轴会让结果不再是
/// 单位四元数，这个前置条件由类型系统表达比写在文档里更可靠。
template <typename Scalar>
[[nodiscard]] QuaternionT<Scalar> from_axis_angle(
    UnitVector3T<Scalar> axis, Scalar angle_radians) noexcept {
    const Scalar half_angle = angle_radians / Scalar{2};
    const Scalar sine = std::sin(half_angle);
    return QuaternionT<Scalar>{
        std::cos(half_angle),
        axis.x() * sine,
        axis.y() * sine,
        axis.z() * sine,
    };
}

/// 用四元数旋转向量。要求旋转是单位四元数。
template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> rotate(
    QuaternionT<Scalar> q, Vector3T<Scalar> v) noexcept {
    // v' = v + 2 * q_vec × (q_vec × v + w * v)
    // 比 q * (0,v) * conj(q) 少了两次四元数乘法，且不必构造纯四元数。
    const Vector3T<Scalar> q_vector{q.x, q.y, q.z};
    const Vector3T<Scalar> t = q_vector.cross(v) + v * q.w;
    return v + q_vector.cross(t) * Scalar{2};
}

/// 转换为等价的旋转矩阵。
template <typename Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, 3> to_matrix(QuaternionT<Scalar> q) noexcept {
    const Scalar xx = q.x * q.x;
    const Scalar yy = q.y * q.y;
    const Scalar zz = q.z * q.z;
    const Scalar xy = q.x * q.y;
    const Scalar xz = q.x * q.z;
    const Scalar yz = q.y * q.z;
    const Scalar wx = q.w * q.x;
    const Scalar wy = q.w * q.y;
    const Scalar wz = q.w * q.z;

    MatrixT<Scalar, 3> result{};
    result.data[0][0] = Scalar{1} - Scalar{2} * (yy + zz);
    result.data[0][1] = Scalar{2} * (xy - wz);
    result.data[0][2] = Scalar{2} * (xz + wy);
    result.data[1][0] = Scalar{2} * (xy + wz);
    result.data[1][1] = Scalar{1} - Scalar{2} * (xx + zz);
    result.data[1][2] = Scalar{2} * (yz - wx);
    result.data[2][0] = Scalar{2} * (xz - wy);
    result.data[2][1] = Scalar{2} * (yz + wx);
    result.data[2][2] = Scalar{1} - Scalar{2} * (xx + yy);
    return result;
}

} // namespace GeoCore::linear
