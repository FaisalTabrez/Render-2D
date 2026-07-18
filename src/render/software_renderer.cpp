#include "render2d/render/software_renderer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace render2d::render {
namespace {

[[nodiscard]] int clampCoordinate(const int value, const int maximum) noexcept {
    return std::clamp(value, 0, maximum);
}

[[nodiscard]] std::string readPpmToken(std::istream& input) {
    char character = '\0';
    while (input.get(character)) {
        if (std::isspace(static_cast<unsigned char>(character))) {
            continue;
        }
        if (character == '#') {
            input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        break;
    }
    if (!input) {
        throw std::runtime_error("PPM file ended before its header was complete");
    }

    std::string token(1, character);
    while (input.get(character) && !std::isspace(static_cast<unsigned char>(character))) {
        token.push_back(character);
    }
    return token;
}

void drawCircle(
    Image& target, const Camera2D& camera, const DrawCommand& command) {
    const math::Vec2 screenCenter = camera.worldToScreen(command.center);
    const float screenRadius = command.radius * camera.pixelsPerMetre();
    if (screenRadius <= 0.0F) {
        return;
    }

    const int minX = clampCoordinate(
        static_cast<int>(std::floor(screenCenter.x - screenRadius)),
        static_cast<int>(target.width()) - 1);
    const int maxX = clampCoordinate(
        static_cast<int>(std::ceil(screenCenter.x + screenRadius)),
        static_cast<int>(target.width()) - 1);
    const int minY = clampCoordinate(
        static_cast<int>(std::floor(screenCenter.y - screenRadius)),
        static_cast<int>(target.height()) - 1);
    const int maxY = clampCoordinate(
        static_cast<int>(std::ceil(screenCenter.y + screenRadius)),
        static_cast<int>(target.height()) - 1);
    const float radiusSquared = screenRadius * screenRadius;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const float horizontal = static_cast<float>(x) + 0.5F - screenCenter.x;
            const float vertical = static_cast<float>(y) + 0.5F - screenCenter.y;
            if (horizontal * horizontal + vertical * vertical <= radiusSquared) {
                target.blendPixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), command.color);
            }
        }
    }
}

void drawRectangle(
    Image& target, const Camera2D& camera, const DrawCommand& command) {
    const math::Vec2 bottomLeft = camera.worldToScreen(command.center - command.extent);
    const math::Vec2 topRight = camera.worldToScreen(command.center + command.extent);
    const int minX = clampCoordinate(
        static_cast<int>(std::floor(std::min(bottomLeft.x, topRight.x))),
        static_cast<int>(target.width()) - 1);
    const int maxX = clampCoordinate(
        static_cast<int>(std::ceil(std::max(bottomLeft.x, topRight.x))),
        static_cast<int>(target.width()) - 1);
    const int minY = clampCoordinate(
        static_cast<int>(std::floor(std::min(bottomLeft.y, topRight.y))),
        static_cast<int>(target.height()) - 1);
    const int maxY = clampCoordinate(
        static_cast<int>(std::ceil(std::max(bottomLeft.y, topRight.y))),
        static_cast<int>(target.height()) - 1);

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const math::Vec2 world = camera.screenToWorld({
                .x = static_cast<float>(x) + 0.5F,
                .y = static_cast<float>(y) + 0.5F,
            });
            const math::Vec2 offset = world - command.center;
            if (std::abs(offset.x) <= command.extent.x && std::abs(offset.y) <= command.extent.y) {
                target.blendPixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), command.color);
            }
        }
    }
}

void drawLine(
    Image& target, const Camera2D& camera, const DrawCommand& command) {
    const math::Vec2 start = camera.worldToScreen(command.center);
    const math::Vec2 end = camera.worldToScreen(command.end);
    const float horizontal = end.x - start.x;
    const float vertical = end.y - start.y;
    const int samples = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(horizontal), std::abs(vertical)))));
    const float radius = std::max(command.thickness * camera.pixelsPerMetre() * 0.5F, 0.5F);
    const float radiusSquared = radius * radius;

    for (int sample = 0; sample <= samples; ++sample) {
        const float fraction = static_cast<float>(sample) / static_cast<float>(samples);
        const float centerX = start.x + horizontal * fraction;
        const float centerY = start.y + vertical * fraction;
        const int minX = clampCoordinate(static_cast<int>(std::floor(centerX - radius)), static_cast<int>(target.width()) - 1);
        const int maxX = clampCoordinate(static_cast<int>(std::ceil(centerX + radius)), static_cast<int>(target.width()) - 1);
        const int minY = clampCoordinate(static_cast<int>(std::floor(centerY - radius)), static_cast<int>(target.height()) - 1);
        const int maxY = clampCoordinate(static_cast<int>(std::ceil(centerY + radius)), static_cast<int>(target.height()) - 1);
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const float dx = static_cast<float>(x) + 0.5F - centerX;
                const float dy = static_cast<float>(y) + 0.5F - centerY;
                if (dx * dx + dy * dy <= radiusSquared) {
                    target.blendPixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y), command.color);
                }
            }
        }
    }
}

[[nodiscard]] Color tint(const Color source, const Color modifier) noexcept {
    const auto multiply = [](const std::uint8_t first, const std::uint8_t second) {
        return static_cast<std::uint8_t>(
            (static_cast<std::uint32_t>(first) * static_cast<std::uint32_t>(second)) / 255U);
    };
    return {
        .red = multiply(source.red, modifier.red),
        .green = multiply(source.green, modifier.green),
        .blue = multiply(source.blue, modifier.blue),
        .alpha = multiply(source.alpha, modifier.alpha),
    };
}

void drawSprite(
    Image& target, const Camera2D& camera, const DrawCommand& command) {
    if (command.texture == nullptr || command.extent.x <= 0.0F || command.extent.y <= 0.0F) {
        return;
    }

    const math::Vec2 bottomLeft = camera.worldToScreen(command.center - command.extent);
    const math::Vec2 topRight = camera.worldToScreen(command.center + command.extent);
    const int minX = clampCoordinate(
        static_cast<int>(std::floor(std::min(bottomLeft.x, topRight.x))),
        static_cast<int>(target.width()) - 1);
    const int maxX = clampCoordinate(
        static_cast<int>(std::ceil(std::max(bottomLeft.x, topRight.x))),
        static_cast<int>(target.width()) - 1);
    const int minY = clampCoordinate(
        static_cast<int>(std::floor(std::min(bottomLeft.y, topRight.y))),
        static_cast<int>(target.height()) - 1);
    const int maxY = clampCoordinate(
        static_cast<int>(std::ceil(std::max(bottomLeft.y, topRight.y))),
        static_cast<int>(target.height()) - 1);

    const Image& texture = command.texture->image();
    const math::Vec2 uvRange = command.uvMax - command.uvMin;
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const math::Vec2 world = camera.screenToWorld({
                .x = static_cast<float>(x) + 0.5F,
                .y = static_cast<float>(y) + 0.5F,
            });
            const math::Vec2 offset = world - command.center;
            if (std::abs(offset.x) > command.extent.x || std::abs(offset.y) > command.extent.y) {
                continue;
            }

            const float localU = (offset.x + command.extent.x) / (2.0F * command.extent.x);
            const float localV = 1.0F - (offset.y + command.extent.y) / (2.0F * command.extent.y);
            const float u = command.uvMin.x + localU * uvRange.x;
            const float v = command.uvMin.y + localV * uvRange.y;
            const std::uint32_t textureX = std::min(
                static_cast<std::uint32_t>(std::clamp(u, 0.0F, 0.999999F) * texture.width()),
                texture.width() - 1U);
            const std::uint32_t textureY = std::min(
                static_cast<std::uint32_t>(std::clamp(v, 0.0F, 0.999999F) * texture.height()),
                texture.height() - 1U);
            target.blendPixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                              tint(texture.pixel(textureX, textureY), command.color));
        }
    }
}

} // namespace

Image::Image(const std::uint32_t width, const std::uint32_t height, const Color fill)
    : width_(width), height_(height), pixels_(static_cast<std::size_t>(width) * height, fill) {
    if (width == 0U || height == 0U) {
        throw std::invalid_argument("Image dimensions must be non-zero");
    }
}

Image Image::readPpm(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("Unable to open PPM input file");
    }
    if (readPpmToken(input) != "P6") {
        throw std::runtime_error("Only binary P6 PPM images are supported");
    }

    const std::uint32_t width = static_cast<std::uint32_t>(std::stoul(readPpmToken(input)));
    const std::uint32_t height = static_cast<std::uint32_t>(std::stoul(readPpmToken(input)));
    const unsigned int maximumChannel = std::stoul(readPpmToken(input));
    if (width == 0U || height == 0U || maximumChannel != 255U ||
        static_cast<std::size_t>(width) > std::numeric_limits<std::size_t>::max() /
            static_cast<std::size_t>(height) / 3U) {
        throw std::runtime_error("PPM dimensions or channel depth are invalid");
    }

    std::vector<std::uint8_t> source(static_cast<std::size_t>(width) * height * 3U);
    input.read(reinterpret_cast<char*>(source.data()), static_cast<std::streamsize>(source.size()));
    if (input.gcount() != static_cast<std::streamsize>(source.size())) {
        throw std::runtime_error("PPM image data is truncated");
    }

    Image image {width, height};
    for (std::uint32_t y = 0; y < height; ++y) {
        for (std::uint32_t x = 0; x < width; ++x) {
            const std::size_t offset = (static_cast<std::size_t>(y) * width + x) * 3U;
            image.setPixel(x, y, {
                .red = source[offset],
                .green = source[offset + 1U],
                .blue = source[offset + 2U],
                .alpha = 255U,
            });
        }
    }
    return image;
}

std::uint32_t Image::width() const noexcept {
    return width_;
}

std::uint32_t Image::height() const noexcept {
    return height_;
}

const Color& Image::pixel(const std::uint32_t x, const std::uint32_t y) const {
    if (x >= width_ || y >= height_) {
        throw std::out_of_range("Image pixel coordinate is outside the image");
    }
    return pixels_[static_cast<std::size_t>(y) * width_ + x];
}

std::span<const Color> Image::pixels() const noexcept {
    return pixels_;
}

void Image::clear(const Color color) noexcept {
    std::fill(pixels_.begin(), pixels_.end(), color);
}

void Image::setPixel(const std::uint32_t x, const std::uint32_t y, const Color color) noexcept {
    if (x < width_ && y < height_) {
        pixels_[static_cast<std::size_t>(y) * width_ + x] = color;
    }
}

void Image::blendPixel(const std::uint32_t x, const std::uint32_t y, const Color color) noexcept {
    if (x < width_ && y < height_) {
        Color& destination = pixels_[static_cast<std::size_t>(y) * width_ + x];
        destination = alphaBlend(color, destination);
    }
}

void Image::writePpm(const std::filesystem::path& path) const {
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("Unable to open PPM output file");
    }
    output << "P6\n" << width_ << ' ' << height_ << "\n255\n";
    for (const Color pixel : pixels_) {
        output.put(static_cast<char>(pixel.red));
        output.put(static_cast<char>(pixel.green));
        output.put(static_cast<char>(pixel.blue));
    }
    if (!output) {
        throw std::runtime_error("Unable to write PPM output file");
    }
}

void SoftwareRenderer::render(
    const DrawList& drawList, const Camera2D& camera, Image& target) const {
    if (target.width() != camera.viewportWidth() || target.height() != camera.viewportHeight()) {
        throw std::invalid_argument("Image dimensions must match the camera viewport");
    }

    target.clear(camera.clearColor());
    std::vector<DrawCommand> sortedCommands(drawList.commands().begin(), drawList.commands().end());
    std::stable_sort(sortedCommands.begin(), sortedCommands.end(), [](const DrawCommand& first,
                                                                   const DrawCommand& second) {
        return first.layer == second.layer ? first.sortKey < second.sortKey
                                            : first.layer < second.layer;
    });

    for (const DrawCommand& command : sortedCommands) {
        switch (command.primitive) {
        case PrimitiveType::Circle:
            drawCircle(target, camera, command);
            break;
        case PrimitiveType::Rectangle:
            drawRectangle(target, camera, command);
            break;
        case PrimitiveType::Line:
            drawLine(target, camera, command);
            break;
        case PrimitiveType::Sprite:
            drawSprite(target, camera, command);
            break;
        }
    }
}

} // namespace render2d::render
