#include "render2d/physics/world.hpp"

#include <iomanip>
#include <iostream>

using namespace render2d::physics;

int main() {
    World world {{.gravity = {0.0F, -9.81F}}};

    const BodyId floor = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, -2.0F},
    });
    static_cast<void>(world.createCircleFixture(floor, {.radius = 1.0F}));

    const BodyId ball = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 3.0F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createCircleFixture(ball, {.radius = 0.5F, .restitution = 0.65F}));

    constexpr float step = 1.0F / 120.0F;
    for (int iteration = 0; iteration < 360; ++iteration) {
        world.step(step);
    }

    const BodyState* const state = world.body(ball);
    if (state == nullptr) {
        return 1;
    }
    std::cout << std::fixed << std::setprecision(3)
              << "ball position: (" << state->position.x << ", " << state->position.y << ")\n"
              << "ball velocity: (" << state->linearVelocity.x << ", " << state->linearVelocity.y << ")\n";
}
