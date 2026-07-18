#pragma once

#include "render2d/math/vec2.hpp"

#include <algorithm>

namespace render2d::math {

struct Aabb {
    Vec2 min {};
    Vec2 max {};
};

[[nodiscard]] constexpr bool overlaps(const Aabb& first, const Aabb& second) noexcept {
    return first.min.x <= second.max.x && first.max.x >= second.min.x &&
           first.min.y <= second.max.y && first.max.y >= second.min.y;
}

[[nodiscard]] constexpr bool contains(const Aabb& bounds, const Vec2 point) noexcept {
    return point.x >= bounds.min.x && point.x <= bounds.max.x &&
           point.y >= bounds.min.y && point.y <= bounds.max.y;
}

[[nodiscard]] constexpr Vec2 clamp(const Vec2 point, const Aabb& bounds) noexcept {
    return {
        .x = std::clamp(point.x, bounds.min.x, bounds.max.x),
        .y = std::clamp(point.y, bounds.min.y, bounds.max.y),
    };
}

[[nodiscard]] inline bool isValid(const Aabb& bounds) noexcept {
    return isFinite(bounds.min) && isFinite(bounds.max) &&
           bounds.min.x <= bounds.max.x && bounds.min.y <= bounds.max.y;
}

} // namespace render2d::math
