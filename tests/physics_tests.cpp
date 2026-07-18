#include "render2d/physics/world.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace render2d::physics;

namespace {

void testGravity() {
    World world {{.gravity = {0.0F, -10.0F}}};
    const BodyId body = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 0.0F},
        .mass = 2.0F,
    });

    world.step(0.1F);
    const BodyState* const state = world.body(body);
    assert(state != nullptr);
    assert(std::abs(state->linearVelocity.y + 1.0F) < 0.0001F);
    assert(std::abs(state->position.y + 0.1F) < 0.0001F);
}

void testCircleContactLifecycle() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId anchor = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, 0.0F},
    });
    static_cast<void>(world.createCircleFixture(anchor, {.radius = 1.0F}));

    const BodyId mover = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 1.5F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createCircleFixture(mover, {.radius = 1.0F}));

    world.step(1.0F / 120.0F);
    assert(world.contactEvents().size() == 1U);
    assert(world.contactEvents().front().type == ContactEventType::Begin);

    world.step(1.0F / 120.0F);
    assert(world.contactEvents().size() == 1U);
    assert(world.contactEvents().front().type == ContactEventType::Stay);

    assert(world.setLinearVelocity(mover, {0.0F, 120.0F}));
    world.step(0.1F);
    assert(world.contactEvents().size() == 1U);
    assert(world.contactEvents().front().type == ContactEventType::End);
}

void testSensorDoesNotCorrectPosition() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId sensor = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, 0.0F},
    });
    static_cast<void>(world.createCircleFixture(sensor, {.radius = 1.0F, .sensor = true}));

    const BodyId mover = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 0.5F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createCircleFixture(mover, {.radius = 1.0F}));

    world.step(1.0F / 120.0F);
    const BodyState* const state = world.body(mover);
    assert(state != nullptr);
    assert(std::abs(state->position.y - 0.5F) < 0.0001F);
    assert(world.contactEvents().front().sensor);
}

} // namespace

int main() {
    testGravity();
    testCircleContactLifecycle();
    testSensorDoesNotCorrectPosition();
    std::cout << "All physics tests passed.\n";
}
