#include "render2d/physics/world.hpp"
#include "render2d/render/software_renderer.hpp"

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

void testBoxContactAndBroadPhaseStats() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId wall = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, 0.0F},
    });
    static_cast<void>(world.createBoxFixture(wall, {.halfExtents = {1.0F, 1.0F}}));

    const BodyId crate = world.createBody({
        .type = BodyType::Dynamic,
        .position = {1.5F, 0.0F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createBoxFixture(crate, {.halfExtents = {1.0F, 1.0F}}));

    world.step(1.0F / 120.0F);
    const BodyState* const state = world.body(crate);
    assert(state != nullptr);
    assert(state->position.x > 1.8F);
    assert(world.contactEvents().size() == 1U);
    assert(world.contactEvents().front().type == ContactEventType::Begin);
    assert(world.stats().broadPhaseCandidatePairs == 1U);
    assert(world.stats().narrowPhaseTests == 1U);
    assert(world.stats().activeContacts == 1U);
}

void testCircleBoxCollision() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId box = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, 0.0F},
    });
    static_cast<void>(world.createBoxFixture(box, {.halfExtents = {1.0F, 1.0F}}));

    const BodyId circle = world.createBody({
        .type = BodyType::Dynamic,
        .position = {-1.25F, 0.0F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createCircleFixture(circle, {.radius = 0.5F}));

    world.step(1.0F / 120.0F);
    const BodyState* const state = world.body(circle);
    assert(state != nullptr);
    assert(state->position.x < -1.4F);
    assert(world.contactEvents().size() == 1U);
}

void testFilterAndAabbQuery() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId box = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, 0.0F},
    });
    const FixtureId boxFixture = world.createBoxFixture(box, {
        .halfExtents = {1.0F, 1.0F},
        .filter = {.categoryBits = 0x0000'0002U},
    });

    const BodyId circle = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 0.0F},
        .mass = 1.0F,
    });
    const FixtureId circleFixture = world.createCircleFixture(circle, {
        .radius = 0.5F,
        .filter = {.categoryBits = 0x0000'0001U, .maskBits = 0x0000'0001U},
    });

    const auto nearby = world.queryAabb({.min = {-2.0F, -2.0F}, .max = {2.0F, 2.0F}});
    assert(nearby.size() == 2U);
    assert((nearby[0] == boxFixture && nearby[1] == circleFixture) ||
           (nearby[0] == circleFixture && nearby[1] == boxFixture));

    world.step(1.0F / 120.0F);
    assert(world.contactEvents().empty());
    assert(world.stats().broadPhaseCandidatePairs == 1U);
    assert(world.stats().narrowPhaseTests == 0U);
}

void testCameraRoundTripAndSoftwareRenderer() {
    render2d::render::Camera2D camera {64U, 64U, 16.0F};
    camera.setPosition({1.0F, -2.0F});
    camera.setRotation(0.35F);
    camera.setClearColor(render2d::render::Color::rgb(1, 2, 3));

    const render2d::math::Vec2 original {2.5F, -0.5F};
    const render2d::math::Vec2 recovered = camera.screenToWorld(camera.worldToScreen(original));
    assert(std::abs(recovered.x - original.x) < 0.0001F);
    assert(std::abs(recovered.y - original.y) < 0.0001F);

    camera.setPosition({0.0F, 0.0F});
    camera.setRotation(0.0F);
    render2d::render::DrawList drawList;
    drawList.addCircle({0.0F, 0.0F}, 1.0F, render2d::render::Color::rgb(220, 30, 40), 0);
    drawList.addCircle({0.0F, 0.0F}, 0.5F, render2d::render::Color::rgb(30, 80, 240), 1);
    drawList.addRectangle({2.0F, 0.0F}, {0.5F, 0.5F}, render2d::render::Color::rgb(20, 200, 90), 0);

    render2d::render::Image image {64U, 64U};
    render2d::render::SoftwareRenderer renderer;
    renderer.render(drawList, camera, image);

    assert(image.pixel(32U, 32U) == render2d::render::Color::rgb(30, 80, 240));
    assert(image.pixel(0U, 0U) == render2d::render::Color::rgb(1, 2, 3));
    assert(image.pixel(64U - 1U, 32U) == render2d::render::Color::rgb(20, 200, 90));
}

} // namespace

int main() {
    testGravity();
    testCircleContactLifecycle();
    testSensorDoesNotCorrectPosition();
    testBoxContactAndBroadPhaseStats();
    testCircleBoxCollision();
    testFilterAndAabbQuery();
    testCameraRoundTripAndSoftwareRenderer();
    std::cout << "All physics tests passed.\n";
}
