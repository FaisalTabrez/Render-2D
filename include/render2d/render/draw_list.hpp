#pragma once

#include "render2d/math/vec2.hpp"
#include "render2d/render/color.hpp"

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

namespace render2d::render {

enum class PrimitiveType {
    Circle,
    Rectangle,
    Line,
};

struct DrawCommand {
    PrimitiveType primitive {PrimitiveType::Circle};
    std::int32_t layer {0};
    std::uint64_t sortKey {0};
    math::Vec2 center {};
    math::Vec2 extent {};
    math::Vec2 end {};
    float radius {0.0F};
    float thickness {1.0F};
    Color color {};
};

class DrawList {
public:
    void clear() noexcept { commands_.clear(); }

    void addCircle(
        const math::Vec2 center,
        const float radius,
        const Color color,
        const std::int32_t layer = 0,
        const std::uint64_t sortKey = 0U) {
        commands_.push_back({
            .primitive = PrimitiveType::Circle,
            .layer = layer,
            .sortKey = sortKey,
            .center = center,
            .radius = radius,
            .color = color,
        });
    }

    void addRectangle(
        const math::Vec2 center,
        const math::Vec2 halfExtents,
        const Color color,
        const std::int32_t layer = 0,
        const std::uint64_t sortKey = 0U) {
        commands_.push_back({
            .primitive = PrimitiveType::Rectangle,
            .layer = layer,
            .sortKey = sortKey,
            .center = center,
            .extent = halfExtents,
            .color = color,
        });
    }

    void addLine(
        const math::Vec2 from,
        const math::Vec2 to,
        const float thickness,
        const Color color,
        const std::int32_t layer = 0,
        const std::uint64_t sortKey = 0U) {
        commands_.push_back({
            .primitive = PrimitiveType::Line,
            .layer = layer,
            .sortKey = sortKey,
            .center = from,
            .end = to,
            .thickness = thickness,
            .color = color,
        });
    }

    void sort() {
        std::stable_sort(commands_.begin(), commands_.end(), [](const DrawCommand& first,
                                                               const DrawCommand& second) {
            return first.layer == second.layer ? first.sortKey < second.sortKey
                                                : first.layer < second.layer;
        });
    }

    [[nodiscard]] std::span<const DrawCommand> commands() const noexcept { return commands_; }

private:
    std::vector<DrawCommand> commands_;
};

} // namespace render2d::render
