#pragma once

#include "render2d/physics/world.hpp"
#include "render2d/render/draw_list.hpp"

namespace render2d::debug {

struct PhysicsDebugSettings {
    bool drawAabbs {true};
    bool drawContacts {true};
    bool drawJoints {true};
    float lineThickness {0.03F};
    int layer {100};
};

class PhysicsDebugRenderer {
public:
    void append(
        const physics::World& world,
        render::DrawList& drawList,
        const PhysicsDebugSettings& settings = {}) const;
};

} // namespace render2d::debug
