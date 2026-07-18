#pragma once

#include "render2d/math/vec2.hpp"
#include "render2d/render/texture.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>

namespace render2d::render {

struct SpriteRegion {
    const Texture* texture {nullptr};
    math::Vec2 uvMin {0.0F, 0.0F};
    math::Vec2 uvMax {1.0F, 1.0F};
};

struct PixelRect {
    std::uint32_t x {0};
    std::uint32_t y {0};
    std::uint32_t width {0};
    std::uint32_t height {0};
};

class SpriteAtlas {
public:
    explicit SpriteAtlas(const Texture& texture) : texture_(&texture) {}

    [[nodiscard]] const SpriteRegion& add(std::string name, const PixelRect pixels) {
        if (name.empty() || pixels.width == 0U || pixels.height == 0U ||
            pixels.x + pixels.width > texture_->image().width() ||
            pixels.y + pixels.height > texture_->image().height()) {
            throw std::invalid_argument("Sprite region must be non-empty and contained by its texture");
        }

        const float textureWidth = static_cast<float>(texture_->image().width());
        const float textureHeight = static_cast<float>(texture_->image().height());
        const auto [iterator, inserted] = regions_.insert_or_assign(std::move(name), SpriteRegion {
            .texture = texture_,
            .uvMin = {static_cast<float>(pixels.x) / textureWidth,
                      static_cast<float>(pixels.y) / textureHeight},
            .uvMax = {static_cast<float>(pixels.x + pixels.width) / textureWidth,
                      static_cast<float>(pixels.y + pixels.height) / textureHeight},
        });
        static_cast<void>(inserted);
        return iterator->second;
    }

    [[nodiscard]] const SpriteRegion* find(const std::string_view name) const noexcept {
        const auto iterator = regions_.find(std::string {name});
        return iterator != regions_.end() ? &iterator->second : nullptr;
    }

private:
    const Texture* texture_ {nullptr};
    std::unordered_map<std::string, SpriteRegion> regions_;
};

} // namespace render2d::render
