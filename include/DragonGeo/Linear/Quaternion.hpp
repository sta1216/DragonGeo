#pragma once

#include <cmath>
#include <concepts>
#include <optional>

#include <DragonGeo/Core/Constants.hpp>
#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Matrix.hpp>
#include <DragonGeo/Linear/UnitVector3.hpp>
#include <DragonGeo/Linear/Vector3.hpp>

namespace DragonGeo::Linear {

/// 四元数，表示三维旋转。
///
/// 成员顺序为标量部分在前（W, X, Y, Z）。默认构造得到**单位四元数**
/// 而非全零 —— 本类型表示旋转，默认值必须表示恒等旋转，否则默认构造
/// 的对象会静默地成为一个不可用的零旋转。
template <typename Scalar>
struct QuaternionT {
    using ScalarType = Scalar;

    Scalar W{1};
    Scalar X{};
    Scalar Y{};
    Scalar Z{};

    [[nodiscard]] constexpr bool operator==(const QuaternionT&) const noexcept = default;

    /// 单位四元数，即恒等旋转。
    [[nodiscard]] static constexpr QuaternionT Identity() noexcept {
        return QuaternionT<Scalar>{};
    }

    /// 共轭。对单位四元数而言即为逆旋转。
    [[nodiscard]] constexpr QuaternionT Conjugate() const noexcept {
        return QuaternionT<Scalar>{W, -X, -Y, -Z};
    }

    /// 四元数内积。
    [[nodiscard]] constexpr Scalar Dot(QuaternionT other) const noexcept {
        return W * other.W + X * other.X + Y * other.Y + Z * other.Z;
    }

    /// 模长。
    [[nodiscard]] Scalar Norm() const noexcept {
        // 先按最大分量缩放，避免中间量上溢或下溢。
        const Scalar scale = Core::MaxAbsOf(W, X, Y, Z);

        if (scale == Scalar{0}) {
            return Scalar{0};
        }
        if (!Core::IsFinite(scale)) {
            // 与 Vector3T::Length 保持一致，规则同为：任一无穷分量 ⇒ ±inf；
            // 否则含 NaN ⇒ NaN。两条路径的语义必须一致，否则调用方在模长的
            // 两个来源上会得到互相矛盾的结果。
            return scale;
        }
        const Scalar sw = W / scale;
        const Scalar sx = X / scale;
        const Scalar sy = Y / scale;
        const Scalar sz = Z / scale;
        return scale * std::sqrt(sw * sw + sx * sx + sy * sy + sz * sz);
    }

    /// 归一化。模长在给定容差下可视为零、或本身不是有限值时返回 std::nullopt。
    [[nodiscard]] std::optional<QuaternionT> Normalized(
        Core::Tolerance tolerance = {}) const noexcept {
        const Scalar magnitude = Norm();
        // 与 UnitVector3T::normalized / MatrixT::inverse 同一条原则：绝不交出一个
        // has_value() 为真、内容却是 NaN 的结果 —— 调用者无从察觉，而 NaN 会
        // 污染后续全部计算。
        if (!Core::IsFinite(static_cast<double>(magnitude))
            || tolerance.IsZero(static_cast<double>(magnitude))) {
            return std::nullopt;
        }
        const Scalar inverseMagnitude = Scalar{1} / magnitude;
        const QuaternionT<Scalar> result{
            W * inverseMagnitude,
            X * inverseMagnitude,
            Y * inverseMagnitude,
            Z * inverseMagnitude,
        };
        // 模长有限并不保证倒数有限：|q| 是次正规数时（例如 5e-324）1/|q| 就是
        // inf，分量乘上去即得 inf。与 MatrixT::inverse 还原尺度后的检查是同一件
        // 事 —— 绝不交出 has_value() 为真、内容却是 inf 的结果。
        if (!Core::IsFinite(static_cast<double>(result.W))
            || !Core::IsFinite(static_cast<double>(result.X))
            || !Core::IsFinite(static_cast<double>(result.Y))
            || !Core::IsFinite(static_cast<double>(result.Z))) {
            return std::nullopt;
        }
        return result;
    }

    /// 由单位轴与弧度角构造旋转。
    ///
    /// 轴参数刻意接受 UnitVector3T 而非 Vector3T：非单位轴会让结果不再是
    /// 单位四元数，这个前置条件由类型系统表达比写在文档里更可靠。
    [[nodiscard]] static QuaternionT FromAxisAngle(
        UnitVector3T<Scalar> axis, Scalar angleRadians) noexcept {
        const Scalar halfAngle = angleRadians / Scalar{2};
        const Scalar sine = std::sin(halfAngle);
        return QuaternionT<Scalar>{
            std::cos(halfAngle),
            axis.X() * sine,
            axis.Y() * sine,
            axis.Z() * sine,
        };
    }

    /// 用四元数旋转向量。要求旋转是单位四元数。
    [[nodiscard]] constexpr Vector3T<Scalar> Rotate(Vector3T<Scalar> v) const noexcept {
        // 旋转公式：v' = v + 2 * qVec × (qVec × v + W * v)
        // 比 q * (0,v) * conj(q) 少了两次四元数乘法，且不必构造纯四元数。
        const Vector3T<Scalar> qVector{X, Y, Z};
        const Vector3T<Scalar> t = qVector.Cross(v) + v * W;
        return v + qVector.Cross(t) * Scalar{2};
    }

    /// 转换为等价的旋转矩阵。
    [[nodiscard]] constexpr MatrixT<Scalar, 3> ToMatrix() const noexcept {
        const Scalar xx = X * X;
        const Scalar yy = Y * Y;
        const Scalar zz = Z * Z;
        const Scalar xy = X * Y;
        const Scalar xz = X * Z;
        const Scalar yz = Y * Z;
        const Scalar wx = W * X;
        const Scalar wy = W * Y;
        const Scalar wz = W * Z;

        MatrixT<Scalar, 3> result{};
        result.Data[0][0] = Scalar{1} - Scalar{2} * (yy + zz);
        result.Data[0][1] = Scalar{2} * (xy - wz);
        result.Data[0][2] = Scalar{2} * (xz + wy);
        result.Data[1][0] = Scalar{2} * (xy + wz);
        result.Data[1][1] = Scalar{1} - Scalar{2} * (xx + zz);
        result.Data[1][2] = Scalar{2} * (yz - wx);
        result.Data[2][0] = Scalar{2} * (xz - wy);
        result.Data[2][1] = Scalar{2} * (yz + wx);
        result.Data[2][2] = Scalar{1} - Scalar{2} * (xx + yy);
        return result;
    }
};

using Quaternion = QuaternionT<double>;
using Quaternionf = QuaternionT<float>;

/// 四元数乘法，对应旋转的复合。`a * b` 表示先施加 b 再施加 a。
template <typename Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> a, QuaternionT<Scalar> b) noexcept {
    return QuaternionT<Scalar>{
        a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z,
        a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
        a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
        a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
    };
}

template <typename Scalar, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr QuaternionT<Scalar> operator*(
    QuaternionT<Scalar> q, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    return QuaternionT<Scalar>{q.W * scale, q.X * scale, q.Y * scale, q.Z * scale};
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
    return QuaternionT<Scalar>{-q.W, -q.X, -q.Y, -q.Z};
}

} // namespace DragonGeo::Linear
