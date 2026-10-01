#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include <DragonGeo/Detail/CurveParameter.hpp>
#include <DragonGeo/Linear/Box3.hpp>
#include <DragonGeo/Linear/Interval.hpp>
#include <DragonGeo/Prim/Segment3.hpp>

namespace DragonGeo::Prim {

/// 三维折线：至少一段，且相邻段首尾精确相接。
///
/// 段序列是私有的，只能经 `FromSegments` 构造，因此本类型不是聚合。
/// 相接判定使用点的 `operator==`，不容差。
/// 参数域是 `[0, SegmentCount()]`，每一段占长度 1。
/// 本步只提供 `Domain`、`PointAt`、`IsValid`、`Box`、`Clone`。
/// 其余曲线协议方法尚未声明。
template <typename Scalar>
class Polyline3T {
public:
    using ScalarType = Scalar;

    /// 空序列，或任一相邻段不满足 `segments[i].B == segments[i + 1].A` 时返回空。
    [[nodiscard]] static std::optional<Polyline3T> FromSegments(
        std::span<const Segment3T<Scalar>> segments) {
        if (segments.empty()) {
            return std::nullopt;
        }
        for (std::size_t index = 0; index + 1 < segments.size(); ++index) {
            if (!(segments[index].B == segments[index + 1].A)) {
                return std::nullopt;
            }
        }
        return std::optional<Polyline3T>(Polyline3T{
            std::vector<Segment3T<Scalar>>(segments.begin(), segments.end())});
    }

    [[nodiscard]] constexpr std::size_t SegmentCount() const noexcept {
        return m_segments.size();
    }

    /// `index` 必须小于 `SegmentCount()`。越界是未定义行为。
    [[nodiscard]] constexpr Segment3T<Scalar> Segment(std::size_t index) const {
        return m_segments[index];
    }

    [[nodiscard]] constexpr std::size_t PointCount() const noexcept {
        return SegmentCount() + 1;
    }

    /// `index == 0` 是首段的 `A`，`index == SegmentCount()` 是末段的 `B`。
    /// 越界是未定义行为。
    [[nodiscard]] constexpr Linear::Point3T<Scalar> Point(std::size_t index) const {
        if (index == m_segments.size()) {
            return m_segments.back().B;
        }
        return m_segments[index].A;
    }

    /// 存储的每一段都有效时为真。
    [[nodiscard]] constexpr bool IsValid() const noexcept {
        for (const Segment3T<Scalar>& segment : m_segments) {
            if (!segment.IsValid()) {
                return false;
            }
        }
        return true;
    }

    /// 参数域 `[0, SegmentCount()]`。每一段占长度 1。
    [[nodiscard]] constexpr Linear::IntervalT<Scalar> Domain() const noexcept {
        return {Scalar{0}, static_cast<Scalar>(m_segments.size())};
    }

    /// `t` 不在域内或非有限时为空。
    /// 整数参数落在顶点上；域的右端点是末段的 `B`。
    [[nodiscard]] constexpr std::optional<Linear::Point3T<Scalar>> PointAt(Scalar t) const noexcept {
        if (!Detail::IsAcceptedParameter(t, Domain())) {
            return std::nullopt;
        }
        const Scalar end = static_cast<Scalar>(m_segments.size());
        if (t == end) {
            return m_segments.back().B;
        }
        const auto index = static_cast<std::size_t>(t);
        const Scalar local = t - static_cast<Scalar>(index);
        return m_segments[index].PointAt(local);
    }

    /// 各段包围盒的并。
    [[nodiscard]] constexpr Linear::Box3T<Scalar> Box() const noexcept {
        Linear::Box3T<Scalar> bounds = Linear::Box3T<Scalar>::Empty();
        for (const Segment3T<Scalar>& segment : m_segments) {
            bounds = bounds.Merged(segment.Box());
        }
        return bounds;
    }

    /// 按值复制整条折线。复制段序列可能分配内存。
    [[nodiscard]] Polyline3T Clone() const {
        return *this;
    }

private:
    explicit Polyline3T(std::vector<Segment3T<Scalar>> segments)
        : m_segments(std::move(segments)) {}

    std::vector<Segment3T<Scalar>> m_segments;
};

/// 逐段比较存储的每一段。按 const 引用传递，避免复制整段序列。
/// `operator!=` 由 C++20 自动生成，不手写。
template <typename Scalar>
[[nodiscard]] constexpr bool operator==(
    const Polyline3T<Scalar>& a, const Polyline3T<Scalar>& b) noexcept {
    if (a.SegmentCount() != b.SegmentCount()) {
        return false;
    }
    for (std::size_t index = 0; index < a.SegmentCount(); ++index) {
        if (!(a.Segment(index) == b.Segment(index))) {
            return false;
        }
    }
    return true;
}

using Polyline3 = Polyline3T<double>;
using Polyline3f = Polyline3T<float>;

} // namespace DragonGeo::Prim
