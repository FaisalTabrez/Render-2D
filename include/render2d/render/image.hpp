#pragma once

#include "render2d/render/color.hpp"

#include <cstdint>
#include <filesystem>
#include <span>
#include <vector>

namespace render2d::render {

class Image {
public:
    Image(std::uint32_t width, std::uint32_t height, Color fill = {});

    [[nodiscard]] std::uint32_t width() const noexcept;
    [[nodiscard]] std::uint32_t height() const noexcept;
    [[nodiscard]] const Color& pixel(std::uint32_t x, std::uint32_t y) const;
    [[nodiscard]] std::span<const Color> pixels() const noexcept;

    void clear(Color color) noexcept;
    void setPixel(std::uint32_t x, std::uint32_t y, Color color) noexcept;
    void blendPixel(std::uint32_t x, std::uint32_t y, Color color) noexcept;
    void writePpm(const std::filesystem::path& path) const;

private:
    std::uint32_t width_ {0};
    std::uint32_t height_ {0};
    std::vector<Color> pixels_;
};

} // namespace render2d::render
