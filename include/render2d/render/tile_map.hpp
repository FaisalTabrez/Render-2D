#pragma once

#include "render2d/render/draw_list.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace render2d::render {

class TileMap {
public:
    TileMap(const std::uint32_t width, const std::uint32_t height, const math::Vec2 tileSize)
        : width_(width), height_(height), tileSize_(tileSize), tiles_(static_cast<std::size_t>(width) * height) {
        if (width == 0U || height == 0U || tileSize.x <= 0.0F || tileSize.y <= 0.0F) {
            throw std::invalid_argument("TileMap dimensions and tile size must be positive");
        }
    }

    void set(const std::uint32_t x, const std::uint32_t y, const SpriteRegion* region) {
        tiles_.at(index(x, y)) = region;
    }

    [[nodiscard]] const SpriteRegion* at(const std::uint32_t x, const std::uint32_t y) const {
        return tiles_.at(index(x, y));
    }

    void appendTo(
        DrawList& drawList,
        const math::Vec2 origin,
        const std::int32_t layer = 0,
        const std::uint64_t firstSortKey = 0U) const {
        const math::Vec2 halfTile = tileSize_ * 0.5F;
        std::uint64_t sortKey = firstSortKey;
        for (std::uint32_t y = 0; y < height_; ++y) {
            for (std::uint32_t x = 0; x < width_; ++x) {
                const SpriteRegion* const region = tiles_[index(x, y)];
                if (region != nullptr) {
                    drawList.addSprite(*region, {
                        .x = origin.x + (static_cast<float>(x) + 0.5F) * tileSize_.x,
                        .y = origin.y - (static_cast<float>(y) + 0.5F) * tileSize_.y,
                    }, halfTile, Color {}, layer, sortKey);
                }
                ++sortKey;
            }
        }
    }

private:
    [[nodiscard]] std::size_t index(const std::uint32_t x, const std::uint32_t y) const {
        if (x >= width_ || y >= height_) {
            throw std::out_of_range("Tile coordinates are outside the map");
        }
        return static_cast<std::size_t>(y) * width_ + x;
    }

    std::uint32_t width_ {0};
    std::uint32_t height_ {0};
    math::Vec2 tileSize_ {};
    std::vector<const SpriteRegion*> tiles_;
};

} // namespace render2d::render
