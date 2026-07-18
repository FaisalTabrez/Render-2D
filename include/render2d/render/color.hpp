#pragma once

#include <algorithm>
#include <cstdint>

namespace render2d::render {

struct Color {
    std::uint8_t red {255};
    std::uint8_t green {255};
    std::uint8_t blue {255};
    std::uint8_t alpha {255};

    [[nodiscard]] static constexpr Color rgb(
        const std::uint8_t redValue,
        const std::uint8_t greenValue,
        const std::uint8_t blueValue) noexcept {
        return {.red = redValue, .green = greenValue, .blue = blueValue, .alpha = 255};
    }

    constexpr bool operator==(const Color&) const noexcept = default;
};

[[nodiscard]] inline Color alphaBlend(const Color source, const Color destination) noexcept {
    const std::uint32_t sourceAlpha = source.alpha;
    const std::uint32_t inverseSourceAlpha = 255U - sourceAlpha;
    const std::uint32_t outputAlpha = sourceAlpha + (destination.alpha * inverseSourceAlpha) / 255U;
    if (outputAlpha == 0U) {
        return {};
    }

    const auto blendChannel = [sourceAlpha, inverseSourceAlpha, outputAlpha,
                               destinationAlpha = destination.alpha](
                                  const std::uint8_t sourceChannel,
                                  const std::uint8_t destinationChannel) {
        return static_cast<std::uint8_t>(std::clamp(
            (static_cast<std::uint32_t>(sourceChannel) * sourceAlpha +
             (static_cast<std::uint32_t>(destinationChannel) * destinationAlpha *
              inverseSourceAlpha) /
                 255U) /
                outputAlpha,
            0U,
            255U));
    };

    return {
        .red = blendChannel(source.red, destination.red),
        .green = blendChannel(source.green, destination.green),
        .blue = blendChannel(source.blue, destination.blue),
        .alpha = static_cast<std::uint8_t>(outputAlpha),
    };
}

} // namespace render2d::render
