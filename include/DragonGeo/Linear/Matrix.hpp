#pragma once

#include <concepts>
#include <optional>

#include <DragonGeo/Core/Numeric.hpp>
#include <DragonGeo/Core/Tolerance.hpp>
#include <DragonGeo/Linear/Vector2.hpp>
#include <DragonGeo/Linear/Vector3.hpp>
#include <DragonGeo/Linear/Vector4.hpp>

namespace DragonGeo::Linear {

/// N×N 方阵，行主序存储。
///
/// 用二维数组而非命名字段：4×4 有 16 个元素，逐一命名反而降低可读性。
/// 通过 operator()(row, column) 访问以保持「先行后列」的一致读法。
template <typename Scalar, int N>
struct MatrixT {
    using ScalarType = Scalar;
    static constexpr int DIMENSION = N;

    Scalar Data[N][N]{};

    [[nodiscard]] constexpr Scalar& operator()(int row, int column) noexcept {
        return Data[row][column];
    }

    [[nodiscard]] constexpr Scalar operator()(int row, int column) const noexcept {
        return Data[row][column];
    }

    /// 单位阵。
    [[nodiscard]] static constexpr MatrixT Identity() noexcept {
        MatrixT<Scalar, N> result{};
        for (int i = 0; i < N; ++i) {
            result.Data[i][i] = Scalar{1};
        }
        return result;
    }

    /// 转置：交换行与列。
    ///
    /// 命名为过去分词以区别于原地操作，与 Normalized() 一致。
    [[nodiscard]] constexpr MatrixT Transposed() const noexcept {
        MatrixT<Scalar, N> result{};
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                result.Data[i][j] = Data[j][i];
            }
        }
        return result;
    }

    /// 2×2 / 3×3 / 4×4 的行列式。
    ///
    /// 用 if constexpr 写成单一实现，而不是为每个尺寸提供一个重载：单一
    /// 实现能借 static_assert 给出「只支持 2/3/4」这样明确的编译期诊断，
    /// 也免去维护三份几乎相同的签名。
    ///
    /// 展开为闭式解（4×4 用第一行余子式展开）而非通用 LU 分解：固定尺寸的
    /// 展开式没有循环开销，编译器也能完全内联。
    [[nodiscard]] constexpr Scalar Determinant() const noexcept {
        static_assert(N == 2 || N == 3 || N == 4,
                      "determinant is implemented for 2x2, 3x3 and 4x4 matrices only");

        if constexpr (N == 2) {
            return Data[0][0] * Data[1][1] - Data[0][1] * Data[1][0];
        } else if constexpr (N == 3) {
            return Data[0][0] * (Data[1][1] * Data[2][2] - Data[1][2] * Data[2][1])
                 - Data[0][1] * (Data[1][0] * Data[2][2] - Data[1][2] * Data[2][0])
                 + Data[0][2] * (Data[1][0] * Data[2][1] - Data[1][1] * Data[2][0]);
        } else {
            const Scalar sub00 = Data[1][1] * (Data[2][2] * Data[3][3] - Data[2][3] * Data[3][2])
                                - Data[1][2] * (Data[2][1] * Data[3][3] - Data[2][3] * Data[3][1])
                                + Data[1][3] * (Data[2][1] * Data[3][2] - Data[2][2] * Data[3][1]);
            const Scalar sub01 = Data[1][0] * (Data[2][2] * Data[3][3] - Data[2][3] * Data[3][2])
                                - Data[1][2] * (Data[2][0] * Data[3][3] - Data[2][3] * Data[3][0])
                                + Data[1][3] * (Data[2][0] * Data[3][2] - Data[2][2] * Data[3][0]);
            const Scalar sub02 = Data[1][0] * (Data[2][1] * Data[3][3] - Data[2][3] * Data[3][1])
                                - Data[1][1] * (Data[2][0] * Data[3][3] - Data[2][3] * Data[3][0])
                                + Data[1][3] * (Data[2][0] * Data[3][1] - Data[2][1] * Data[3][0]);
            const Scalar sub03 = Data[1][0] * (Data[2][1] * Data[3][2] - Data[2][2] * Data[3][1])
                                - Data[1][1] * (Data[2][0] * Data[3][2] - Data[2][2] * Data[3][0])
                                + Data[1][2] * (Data[2][0] * Data[3][1] - Data[2][1] * Data[3][0]);

            return Data[0][0] * sub00 - Data[0][1] * sub01
                 + Data[0][2] * sub02 - Data[0][3] * sub03;
        }
    }

    /// 逆矩阵。行列式在给定容差下可视为零（即奇异）时返回 std::nullopt ——
    /// 参与比较的是双侧平衡后的行列式，理由见函数体首段注释。
    ///
    /// 用 optional 而非抛出异常：奇异矩阵是数学事实，不是程序错误，调用者
    /// 有责任处理这个分支。返回 std::nullopt 也让调用者不可能拿到一个含
    /// inf / NaN 的矩阵。
    ///
    /// 与 determinant 一样用 if constexpr 写成单一实现。
    [[nodiscard]] constexpr std::optional<MatrixT> Inverse(
        Core::Tolerance tolerance = {}) const noexcept {
        static_assert(N == 2 || N == 3 || N == 4,
                      "inverse is implemented for 2x2, 3x3 and 4x4 matrices only");

        // 行列式是 N 阶量，直接与长度容差比较是量纲错误：s = 1e-4 时 det = 1e-12
        // 会被默认容差的绝对项判为奇异，而 s = 1e150 时 det 会先溢出成 ±inf。
        //
        // 双侧平衡：先逐行、再逐列，各除以本行 / 本列的最大绝对元素。
        //
        // 记 R = diag(rowScale)、C = diag(columnScale)，则
        //     B = R⁻¹·A        （第 i 行除以 rowScale[i]）
        //     N = B·C⁻¹        （B 的第 j 列除以 columnScale[j]）
        // 于是
        //     A = R·N·C        且        A⁻¹ = C⁻¹·N⁻¹·R⁻¹
        // 即还原时元素 (i, j) 要除以 columnScale[i] 与 rowScale[j] —— **行号取
        // 列尺度、列号取行尺度**。两个对角因子一左一右，下标极易写反，而对称矩阵
        // 上的测试看不出这个错误。
        //
        // 为什么必须两侧都做：只做逐行平衡时，Transform3T::Translation(t) 的第 0 行 (1,0,0,t)
        // 变成 (1/t,0,0,1)，行列式恰为 1/t，t ≥ 1e12 就被默认容差的绝对项（1e-12）
        // 判成奇异 —— 可它的真逆是精确平移 -t，既存在又精确可表示。逐列平衡把第 0
        // 列重新放大回 1，行列式回到 1，与 t 无关。这正是本函数该有的语义：可逆性
        // 由条件数（即平衡后行列式与 0 的距离）决定，而不是由矩阵的整体尺度决定 ——
        // 否则 Transform3T::Scaling(1e150)（条件数 1e150）可逆，而 Transform3T::Translation(1e12)
        // （条件数 1e12）反倒「奇异」，自相矛盾。
        //
        // 「按整个矩阵的最大元素归一化」同样不成立 —— Transform3T::scaling 返回的是齐次矩阵，
        // 其第 4 行恒为 (0,0,0,1)，最大元素永远是 1，于是小尺度缩放的 det 依旧是
        // 1e-12。逐行 / 逐列平衡不假设各行、各列元素同量级，正合此处；平衡后的
        // 行列式落在由 N 决定的小常数之内，既不会溢出，也不会因量纲而与一阶容差
        // 不可比。
        //
        // 折叠用滚动比较而非 MaxAbsOf：这里与 Length() 不同，非有限分量不需要
        // 在折叠里显式保留 —— 它要么被跳过（与 rowMax 的比较恒为 false），要么
        // 让 rowMax 变成 ±inf，两种情形都会在 balanced 里留下 NaN，交由下面的
        // 行列式守卫处置。
        Scalar rowScale[N];
        for (int i = 0; i < N; ++i) {
            Scalar rowMax = Scalar{0};
            for (int j = 0; j < N; ++j) {
                const Scalar magnitude = Core::AbsoluteValue(Data[i][j]);
                if (magnitude > rowMax) {
                    rowMax = magnitude;
                }
            }
            if (rowMax == Scalar{0}) {
                return std::nullopt;   // 整行为零 ⇒ 必然奇异
            }
            rowScale[i] = rowMax;
        }

        // 列平衡作用在**已完成行平衡的** B 上，而不是原始矩阵：两级平衡必须依次
        // 施加，后一级的量尺要在前一级的结果上量。若在原始元素上取列尺度，行与行
        // 的量级差会重新混进列里 —— 此时 N 的列最大元素是 1/rI（而非 1），行、列
        // 都没有归一化到 1，行列式「落在由 N 决定的小常数之内」这条保证随之失效。
        MatrixT<Scalar, N> balanced{};
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                balanced.Data[i][j] = Data[i][j] / rowScale[i];
            }
        }

        Scalar columnScale[N];
        for (int j = 0; j < N; ++j) {
            Scalar columnMax = Scalar{0};
            for (int i = 0; i < N; ++i) {
                const Scalar magnitude = Core::AbsoluteValue(balanced.Data[i][j]);
                if (magnitude > columnMax) {
                    columnMax = magnitude;
                }
            }
            // 行尺度非零，故 B 的整列为零当且仅当 A 的该列整列为零 ⇒ 必然奇异。
            if (columnMax == Scalar{0}) {
                return std::nullopt;
            }
            columnScale[j] = columnMax;
        }

        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                balanced.Data[i][j] /= columnScale[j];
            }
        }

        const Scalar det = balanced.Determinant();
        // 行列式非有限时同样返回 nullopt。若只依赖容差判断，含 NaN 或 ±inf 的
        // 矩阵会得到一个 has_value() 为真、内容全是 NaN 的逆矩阵 —— 调用者无从
        // 察觉，而 NaN 会污染后续全部计算。这与 Vector3T::normalized 的裁定是同一条原则。
        if (!Core::IsFinite(static_cast<double>(det))
            || tolerance.IsZero(static_cast<double>(det))) {
            return std::nullopt;
        }
        const Scalar invDet = Scalar{1} / det;

        MatrixT<Scalar, N> result{};
        if constexpr (N == 2) {
            result.Data[0][0] =  balanced.Data[1][1] * invDet;
            result.Data[0][1] = -balanced.Data[0][1] * invDet;
            result.Data[1][0] = -balanced.Data[1][0] * invDet;
            result.Data[1][1] =  balanced.Data[0][0] * invDet;
        } else if constexpr (N == 3) {
            result.Data[0][0] = (balanced.Data[1][1] * balanced.Data[2][2] - balanced.Data[1][2] * balanced.Data[2][1]) * invDet;
            result.Data[0][1] = (balanced.Data[0][2] * balanced.Data[2][1] - balanced.Data[0][1] * balanced.Data[2][2]) * invDet;
            result.Data[0][2] = (balanced.Data[0][1] * balanced.Data[1][2] - balanced.Data[0][2] * balanced.Data[1][1]) * invDet;
            result.Data[1][0] = (balanced.Data[1][2] * balanced.Data[2][0] - balanced.Data[1][0] * balanced.Data[2][2]) * invDet;
            result.Data[1][1] = (balanced.Data[0][0] * balanced.Data[2][2] - balanced.Data[0][2] * balanced.Data[2][0]) * invDet;
            result.Data[1][2] = (balanced.Data[0][2] * balanced.Data[1][0] - balanced.Data[0][0] * balanced.Data[1][2]) * invDet;
            result.Data[2][0] = (balanced.Data[1][0] * balanced.Data[2][1] - balanced.Data[1][1] * balanced.Data[2][0]) * invDet;
            result.Data[2][1] = (balanced.Data[0][1] * balanced.Data[2][0] - balanced.Data[0][0] * balanced.Data[2][1]) * invDet;
            result.Data[2][2] = (balanced.Data[0][0] * balanced.Data[1][1] - balanced.Data[0][1] * balanced.Data[1][0]) * invDet;
        } else {
            // 4×4 按伴随矩阵求逆：result(i, j) = cofactor(j, i) / det。
            // 先用 2×2 子式算出每个 3×3 余子式，再按符号填入转置位置。
            const auto minor3 = [&balanced](int skipRow, int skipColumn) noexcept -> Scalar {
                Scalar block[3][3];
                int r = 0;
                for (int i = 0; i < 4; ++i) {
                    if (i == skipRow) { continue; }
                    int c = 0;
                    for (int j = 0; j < 4; ++j) {
                        if (j == skipColumn) { continue; }
                        block[r][c] = balanced.Data[i][j];
                        ++c;
                    }
                    ++r;
                }
                return block[0][0] * (block[1][1] * block[2][2] - block[1][2] * block[2][1])
                     - block[0][1] * (block[1][0] * block[2][2] - block[1][2] * block[2][0])
                     + block[0][2] * (block[1][0] * block[2][1] - block[1][1] * block[2][0]);
            };

            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    const Scalar cofactor = minor3(j, i);
                    const Scalar sign = ((i + j) % 2 == 0) ? Scalar{1} : Scalar{-1};
                    result.Data[i][j] = sign * cofactor * invDet;
                }
            }
        }

        // A = R·N·C ⇒ A^-1 = C^-1·N^-1·R^-1，即元素 (i, j) 除以 columnScale[i] 与
        // rowScale[j]：**行号配列尺度、列号配行尺度**，与直觉相反。result 里装的
        // 是 N^-1，两个尺度一左一右夹着它；写成 rowScale[i] / columnScale[j] 会
        // 让非对角元出错，而对称矩阵上的测试未必看得出来。
        //
        // 两次除法分开写，而不是先乘出 1/(columnScale[i] · rowScale[j])：尺度量级
        // 相差悬殊时，这个中间乘积是唯一会脱离 double 范围的量（一侧可以极小、
        // 另一侧可以极大），先算出它会在结果本身完全可表示时就提前上溢或下溢成
        // 0 / ±inf。分两步除，是否溢出只取决于最终元素，那才是真实逆矩阵的固有性质。
        //
        // 还原尺度后元素可能重新溢出（真实逆确实可能巨大），必须再检查一次，否则
        // 又会交出一个 has_value() 为真、内容却是 inf 的矩阵 —— 那正是先前裁定
        // 禁止的形态。
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                result.Data[i][j] /= columnScale[i];
                result.Data[i][j] /= rowScale[j];
                if (!Core::IsFinite(static_cast<double>(result.Data[i][j]))) {
                    return std::nullopt;
                }
            }
        }
        return result;
    }
};

using Matrix2 = MatrixT<double, 2>;
using Matrix2f = MatrixT<float, 2>;
using Matrix3 = MatrixT<double, 3>;
using Matrix3f = MatrixT<float, 3>;
using Matrix4 = MatrixT<double, 4>;
using Matrix4f = MatrixT<float, 4>;

// ---- 运算符 ----

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator+(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.Data[i][j] = a.Data[i][j] + b.Data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator-(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.Data[i][j] = a.Data[i][j] - b.Data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < N; ++k) {
            const Scalar factor = a.Data[i][k];
            for (int j = 0; j < N; ++j) {
                result.Data[i][j] += factor * b.Data[k][j];
            }
        }
    }
    return result;
}

template <typename Scalar, int N, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    const MatrixT<Scalar, N>& m, Factor factor) noexcept {
    const auto scale = static_cast<Scalar>(factor);
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.Data[i][j] = m.Data[i][j] * scale;
        }
    }
    return result;
}

/// 标量 × 矩阵。与 operator*(matrix, factor) 对称 —— 缺了它 `2.0 * m`
/// 会编译失败，而 Vector 与 UnitVector 都提供两种写法。
template <typename Scalar, int N, typename Factor>
    requires std::convertible_to<Factor, Scalar>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator*(
    Factor factor, const MatrixT<Scalar, N>& m) noexcept {
    return m * factor;
}

/// 逐元素取负。
template <typename Scalar, int N>
[[nodiscard]] constexpr MatrixT<Scalar, N> operator-(const MatrixT<Scalar, N>& m) noexcept {
    MatrixT<Scalar, N> result{};
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            result.Data[i][j] = -m.Data[i][j];
        }
    }
    return result;
}

template <typename Scalar, int N>
[[nodiscard]] constexpr bool operator==(
    const MatrixT<Scalar, N>& a, const MatrixT<Scalar, N>& b) noexcept {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (a.Data[i][j] != b.Data[i][j]) {
                return false;
            }
        }
    }
    return true;
}

// ---- 矩阵 × 向量 ----
//
// 三个维度各写一个重载，而不是写一个泛型版本：泛型版本需要把向量
// 也参数化，会立刻把调用点拖进模板推导的泥潭，得不偿失。

template <typename Scalar>
[[nodiscard]] constexpr Vector2T<Scalar> operator*(
    const MatrixT<Scalar, 2>& m, Vector2T<Scalar> v) noexcept {
    return Vector2T<Scalar>{
        m.Data[0][0] * v.X + m.Data[0][1] * v.Y,
        m.Data[1][0] * v.X + m.Data[1][1] * v.Y,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector3T<Scalar> operator*(
    const MatrixT<Scalar, 3>& m, Vector3T<Scalar> v) noexcept {
    return Vector3T<Scalar>{
        m.Data[0][0] * v.X + m.Data[0][1] * v.Y + m.Data[0][2] * v.Z,
        m.Data[1][0] * v.X + m.Data[1][1] * v.Y + m.Data[1][2] * v.Z,
        m.Data[2][0] * v.X + m.Data[2][1] * v.Y + m.Data[2][2] * v.Z,
    };
}

template <typename Scalar>
[[nodiscard]] constexpr Vector4T<Scalar> operator*(
    const MatrixT<Scalar, 4>& m, Vector4T<Scalar> v) noexcept {
    return Vector4T<Scalar>{
        m.Data[0][0] * v.X + m.Data[0][1] * v.Y + m.Data[0][2] * v.Z + m.Data[0][3] * v.W,
        m.Data[1][0] * v.X + m.Data[1][1] * v.Y + m.Data[1][2] * v.Z + m.Data[1][3] * v.W,
        m.Data[2][0] * v.X + m.Data[2][1] * v.Y + m.Data[2][2] * v.Z + m.Data[2][3] * v.W,
        m.Data[3][0] * v.X + m.Data[3][1] * v.Y + m.Data[3][2] * v.Z + m.Data[3][3] * v.W,
    };
}

} // namespace DragonGeo::Linear
