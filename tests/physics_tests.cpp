#include "render2d/physics/world.hpp"
#include "render2d/render/software_renderer.hpp"
#include "render2d/render/tile_map.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
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

void testAngularMotion() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId body = world.createBody({
        .type = BodyType::Dynamic,
        .momentOfInertia = 2.0F,
    });
    assert(world.applyTorque(body, 4.0F));
    world.step(0.5F);

    const BodyState* const state = world.body(body);
    assert(state != nullptr);
    assert(std::abs(state->angularVelocity - 1.0F) < 0.0001F);
    assert(std::abs(state->angle - 0.5F) < 0.0001F);

    assert(world.applyForceAtPoint(body, {0.0F, 2.0F}, {1.0F, 0.0F}));
    world.step(0.5F);
    assert(state->angularVelocity > 1.4F);
}

void testSleepAndWake() {
    World world {{
        .gravity = {0.0F, 0.0F},
        .sleepDelay = 0.1F,
    }};
    const BodyId body = world.createBody({
        .type = BodyType::Dynamic,
        .mass = 1.0F,
    });
    world.step(0.11F);
    const BodyState* const state = world.body(body);
    assert(state != nullptr);
    assert(state->asleep);
    assert(world.stats().sleepingBodies == 1U);

    assert(world.applyForce(body, {2.0F, 0.0F}));
    assert(!state->asleep);
    world.step(0.1F);
    assert(state->position.x > 0.0F);
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

void testOrientedBoxContacts() {
    constexpr float halfPi = 1.57079632679F;
    World separatedWorld {{.gravity = {0.0F, 0.0F}}};
    const BodyId thinBox = separatedWorld.createBody({
        .type = BodyType::Static,
        .angle = halfPi,
    });
    static_cast<void>(separatedWorld.createBoxFixture(thinBox, {.halfExtents = {0.4F, 0.1F}}));
    const BodyId separatedCircle = separatedWorld.createBody({
        .type = BodyType::Dynamic,
        .position = {0.45F, 0.0F},
        .mass = 1.0F,
    });
    static_cast<void>(separatedWorld.createCircleFixture(separatedCircle, {.radius = 0.2F}));
    separatedWorld.step(1.0F / 120.0F);
    assert(separatedWorld.contactEvents().empty());

    World overlapWorld {{.gravity = {0.0F, 0.0F}}};
    const BodyId firstBox = overlapWorld.createBody({
        .type = BodyType::Static,
        .angle = 0.78539816339F,
    });
    static_cast<void>(overlapWorld.createBoxFixture(firstBox, {.halfExtents = {1.0F, 0.2F}}));
    const BodyId secondBox = overlapWorld.createBody({
        .type = BodyType::Dynamic,
        .position = {0.45F, 0.45F},
        .angle = -0.78539816339F,
        .mass = 1.0F,
    });
    static_cast<void>(overlapWorld.createBoxFixture(secondBox, {.halfExtents = {0.35F, 0.35F}}));
    overlapWorld.step(1.0F / 120.0F);
    assert(overlapWorld.contactEvents().size() == 1U);
    assert(overlapWorld.contactEvents().front().type == ContactEventType::Begin);
}

void testConvexPolygonContacts() {
    World world {{.gravity = {0.0F, 0.0F}}};
    const BodyId triangle = world.createBody({.type = BodyType::Static});
    static_cast<void>(world.createPolygonFixture(triangle, {
        .vertices = {{-1.0F, -1.0F}, {1.0F, -1.0F}, {0.0F, 1.0F}},
    }));
    const BodyId circle = world.createBody({
        .type = BodyType::Dynamic,
        .position = {0.0F, 0.85F},
        .mass = 1.0F,
    });
    static_cast<void>(world.createCircleFixture(circle, {.radius = 0.3F}));
    world.step(1.0F / 120.0F);
    assert(world.contactEvents().size() == 1U);

    World polygonWorld {{.gravity = {0.0F, 0.0F}}};
    const BodyId first = polygonWorld.createBody({.type = BodyType::Static});
    static_cast<void>(polygonWorld.createPolygonFixture(first, {
        .vertices = {{-1.0F, -0.5F}, {1.0F, -0.5F}, {1.0F, 0.5F}, {-1.0F, 0.5F}},
    }));
    const BodyId second = polygonWorld.createBody({
        .type = BodyType::Dynamic,
        .position = {0.75F, 0.0F},
        .mass = 1.0F,
    });
    static_cast<void>(polygonWorld.createPolygonFixture(second, {
        .vertices = {{-0.5F, -0.5F}, {0.5F, -0.5F}, {0.0F, 0.6F}},
    }));
    polygonWorld.step(1.0F / 120.0F);
    assert(polygonWorld.contactEvents().size() == 1U);

    bool rejectedConcavePolygon = false;
    try {
        static_cast<void>(polygonWorld.createPolygonFixture(second, {
            .vertices = {{-1.0F, -1.0F}, {1.0F, -1.0F}, {0.0F, 0.0F}, {1.0F, 1.0F}, {-1.0F, 1.0F}},
        }));
    } catch (const std::invalid_argument&) {
        rejectedConcavePolygon = true;
    }
    assert(rejectedConcavePolygon);
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

void testSweepAndPruneSkipsSparsePairs() {
    World world {{.gravity = {0.0F, 0.0F}}};
    for (int index = 0; index < 16; ++index) {
        const BodyId body = world.createBody({
            .type = BodyType::Dynamic,
            .position = {static_cast<float>(index) * 3.0F, 0.0F},
            .mass = 1.0F,
        });
        static_cast<void>(world.createCircleFixture(body, {.radius = 0.25F}));
    }
    world.step(1.0F / 120.0F);
    assert(world.stats().broadPhasePairTests == 0U);
    assert(world.stats().broadPhaseCandidatePairs == 0U);
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

void testTexturesSpritesAtlasesAndTileMaps() {
    using namespace render2d::render;

    Image source {2U, 2U};
    source.setPixel(0U, 0U, Color::rgb(240, 30, 20));
    source.setPixel(1U, 0U, Color::rgb(30, 230, 40));
    source.setPixel(0U, 1U, Color::rgb(35, 70, 245));
    source.setPixel(1U, 1U, Color::rgb(245, 210, 30));

    const std::filesystem::path texturePath = "render2d_test_texture.ppm";
    source.writePpm(texturePath);
    const Image loaded = Image::readPpm(texturePath);
    std::filesystem::remove(texturePath);
    assert(loaded.pixel(0U, 0U) == Color::rgb(240, 30, 20));
    assert(loaded.pixel(1U, 1U) == Color::rgb(245, 210, 30));

    Texture texture {loaded};
    SpriteAtlas atlas {texture};
    const SpriteRegion& whole = atlas.add("whole", {.x = 0U, .y = 0U, .width = 2U, .height = 2U});
    const SpriteRegion& topRight = atlas.add("top-right", {.x = 1U, .y = 0U, .width = 1U, .height = 1U});
    assert(atlas.find("top-right") == &topRight);

    Camera2D camera {64U, 64U, 16.0F};
    camera.setClearColor(Color::rgb(1, 2, 3));
    DrawList drawList;
    drawList.addSprite(whole, {0.0F, 0.0F}, {1.0F, 1.0F});
    Image rendered {64U, 64U};
    SoftwareRenderer renderer;
    renderer.render(drawList, camera, rendered);
    assert(rendered.pixel(20U, 20U) == Color::rgb(240, 30, 20));
    assert(rendered.pixel(44U, 20U) == Color::rgb(30, 230, 40));
    assert(rendered.pixel(20U, 44U) == Color::rgb(35, 70, 245));
    assert(rendered.pixel(44U, 44U) == Color::rgb(245, 210, 30));

    TileMap tileMap {2U, 1U, {1.0F, 1.0F}};
    tileMap.set(0U, 0U, &whole);
    tileMap.set(1U, 0U, &topRight);
    drawList.clear();
    tileMap.appendTo(drawList, {-1.0F, 0.5F}, 3, 9U);
    assert(drawList.commands().size() == 2U);
    assert(drawList.commands()[0].primitive == PrimitiveType::Sprite);
    assert(drawList.commands()[0].sortKey == 9U);
    assert(drawList.commands()[1].sortKey == 10U);
}

} // namespace

int main() {
    testGravity();
    testAngularMotion();
    testSleepAndWake();
    testCircleContactLifecycle();
    testSensorDoesNotCorrectPosition();
    testBoxContactAndBroadPhaseStats();
    testCircleBoxCollision();
    testOrientedBoxContacts();
    testConvexPolygonContacts();
    testFilterAndAabbQuery();
    testSweepAndPruneSkipsSparsePairs();
    testCameraRoundTripAndSoftwareRenderer();
    testTexturesSpritesAtlasesAndTileMaps();
    std::cout << "All physics tests passed.\n";
}
