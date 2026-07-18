#pragma once

#include "render2d/math/vec2.hpp"
#include "render2d/render/color.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace render2d::render {

class Camera2D {
public:
    Camera2D(
        const std::uint32_t viewportWidth,
        const std::uint32_t viewportHeight,
        const float pixelsPerMetre = 100.0F)
        : viewportWidth_(viewportWidth), viewportHeight_(viewportHeight),
          pixelsPerMetre_(pixelsPerMetre) {
        validate();
    }

    [[nodiscard]] std::uint32_t viewportWidth() const noexcept { return viewportWidth_; }
    [[nodiscard]] std::uint32_t viewportHeight() const noexcept { return viewportHeight_; }
    [[nodiscard]] float pixelsPerMetre() const noexcept { return pixelsPerMetre_; }
    [[nodiscard]] math::Vec2 position() const noexcept { return position_; }
    [[nodiscard]] float rotation() const noexcept { return rotation_; }
    [[nodiscard]] Color clearColor() const noexcept { return clearColor_; }

    void setViewport(const std::uint32_t width, const std::uint32_t height) {
        viewportWidth_ = width;
        viewportHeight_ = height;
        validate();
    }

    void setPixelsPerMetre(const float pixelsPerMetre) {
        pixelsPerMetre_ = pixelsPerMetre;
        validate();
    }

    void setPosition(const math::Vec2 position) {
        if (!math::isFinite(position)) {
            throw std::invalid_argument("Camera2D position must be finite");
        }
        position_ = position;
    }

    void setRotation(const float rotationRadians) {
        if (!std::isfinite(rotationRadians)) {
            throw std::invalid_argument("Camera2D rotation must be finite");
        }
        rotation_ = rotationRadians;
    }

    void setClearColor(const Color clearColor) noexcept { clearColor_ = clearColor; }

    [[nodiscard]] math::Vec2 worldToScreen(const math::Vec2 world) const noexcept {
        const math::Vec2 relative = world - position_;
        const float cosine = std::cos(rotation_);
        const float sine = std::sin(rotation_);
        const math::Vec2 cameraSpace {
            .x = cosine * relative.x + sine * relative.y,
            .y = -sine * relative.x + cosine * relative.y,
        };
        return {
            .x = static_cast<float>(viewportWidth_) * 0.5F + cameraSpace.x * pixelsPerMetre_,
            .y = static_cast<float>(viewportHeight_) * 0.5F - cameraSpace.y * pixelsPerMetre_,
        };
    }

    [[nodiscard]] math::Vec2 screenToWorld(const math::Vec2 screen) const noexcept {
        const math::Vec2 cameraSpace {
            .x = (screen.x - static_cast<float>(viewportWidth_) * 0.5F) / pixelsPerMetre_,
            .y = -(screen.y - static_cast<float>(viewportHeight_) * 0.5F) / pixelsPerMetre_,
        };
        const float cosine = std::cos(rotation_);
        const float sine = std::sin(rotation_);
        return {
            .x = position_.x + cosine * cameraSpace.x - sine * cameraSpace.y,
            .y = position_.y + sine * cameraSpace.x + cosine * cameraSpace.y,
        };
    }

private:
    void validate() const {
        if (viewportWidth_ == 0U || viewportHeight_ == 0U || !std::isfinite(pixelsPerMetre_) ||
            pixelsPerMetre_ <= 0.0F) {
            throw std::invalid_argument("Camera2D viewport and scale must be valid");
        }
    }

    std::uint32_t viewportWidth_ {1};
    std::uint32_t viewportHeight_ {1};
    float pixelsPerMetre_ {100.0F};
    math::Vec2 position_ {};
    float rotation_ {0.0F};
    Color clearColor_ {Color::rgb(20, 24, 31)};
};

} // namespace render2d::render
