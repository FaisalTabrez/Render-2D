#pragma once

#include <cmath>

namespace render2d::math {

struct Vec2 {
    float x {0.0F};
    float y {0.0F};

    [[nodiscard]] constexpr Vec2 operator+() const noexcept { return *this; }
    [[nodiscard]] constexpr Vec2 operator-() const noexcept { return {-x, -y}; }

    constexpr Vec2& operator+=(const Vec2 other) noexcept {
        x += other.x;
        y += other.y;
        return *this;
    }

    constexpr Vec2& operator-=(const Vec2 other) noexcept {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    constexpr Vec2& operator*=(const float scalar) noexcept {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr Vec2& operator/=(const float scalar) noexcept {
        x /= scalar;
        y /= scalar;
        return *this;
    }
};

[[nodiscard]] constexpr Vec2 operator+(Vec2 left, const Vec2 right) noexcept {
    return left += right;
}

[[nodiscard]] constexpr Vec2 operator-(Vec2 left, const Vec2 right) noexcept {
    return left -= right;
}

[[nodiscard]] constexpr Vec2 operator*(Vec2 value, const float scalar) noexcept {
    return value *= scalar;
}

[[nodiscard]] constexpr Vec2 operator*(const float scalar, Vec2 value) noexcept {
    return value *= scalar;
}

[[nodiscard]] constexpr Vec2 operator/(Vec2 value, const float scalar) noexcept {
    return value /= scalar;
}

[[nodiscard]] constexpr float dot(const Vec2 left, const Vec2 right) noexcept {
    return (left.x * right.x) + (left.y * right.y);
}

[[nodiscard]] constexpr float lengthSquared(const Vec2 value) noexcept {
    return dot(value, value);
}

[[nodiscard]] constexpr float cross(const Vec2 first, const Vec2 second) noexcept {
    return first.x * second.y - first.y * second.x;
}

[[nodiscard]] constexpr Vec2 cross(const float scalar, const Vec2 value) noexcept {
    return {-scalar * value.y, scalar * value.x};
}

[[nodiscard]] inline Vec2 rotate(const Vec2 value, const float angle) noexcept {
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    return {cosine * value.x - sine * value.y, sine * value.x + cosine * value.y};
}

[[nodiscard]] inline float length(const Vec2 value) noexcept {
    return std::sqrt(lengthSquared(value));
}

[[nodiscard]] inline Vec2 normalized(const Vec2 value) noexcept {
    constexpr float epsilon = 1.0e-6F;
    const float valueLength = length(value);
    return valueLength > epsilon ? value / valueLength : Vec2{1.0F, 0.0F};
}

[[nodiscard]] inline bool isFinite(const Vec2 value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace render2d::math
