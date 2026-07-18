#pragma once

#include "render2d/render/image.hpp"

#include <filesystem>
#include <memory>
#include <vector>

namespace render2d::render {

class Texture {
public:
    explicit Texture(Image image) : image_(std::move(image)) {}

    [[nodiscard]] static Texture loadPpm(const std::filesystem::path& path) {
        return Texture {Image::readPpm(path)};
    }

    [[nodiscard]] const Image& image() const noexcept { return image_; }

private:
    Image image_;
};

class TextureLibrary {
public:
    [[nodiscard]] Texture& add(Image image) {
        textures_.push_back(std::make_unique<Texture>(std::move(image)));
        return *textures_.back();
    }

    [[nodiscard]] Texture& loadPpm(const std::filesystem::path& path) {
        textures_.push_back(std::make_unique<Texture>(Texture::loadPpm(path)));
        return *textures_.back();
    }

private:
    std::vector<std::unique_ptr<Texture>> textures_;
};

} // namespace render2d::render
