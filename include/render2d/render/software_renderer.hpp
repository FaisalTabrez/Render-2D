#pragma once

#include "render2d/render/camera2d.hpp"
#include "render2d/render/draw_list.hpp"
#include "render2d/render/image.hpp"

namespace render2d::render {

class SoftwareRenderer {
public:
    void render(const DrawList& drawList, const Camera2D& camera, Image& target) const;
};

} // namespace render2d::render
