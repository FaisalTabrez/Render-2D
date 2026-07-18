#pragma once

#include "render2d/render/camera2d.hpp"
#include "render2d/render/draw_list.hpp"

#include <memory>

namespace render2d::render {

using GlProcAddress = void (*)();
using GlProcAddressResolver = GlProcAddress (*)(const char* name);

class OpenGlRenderer {
public:
    explicit OpenGlRenderer(GlProcAddressResolver resolveProcAddress);
    ~OpenGlRenderer();

    OpenGlRenderer(const OpenGlRenderer&) = delete;
    OpenGlRenderer& operator=(const OpenGlRenderer&) = delete;

    void render(const DrawList& drawList, const Camera2D& camera);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace render2d::render
