#include "render2d/physics/world.hpp"
#include "render2d/render/software_renderer.hpp"

#include <filesystem>
#include <iomanip>
#include <iostream>

using namespace render2d::physics;

int main() {
    World world {{.gravity = {0.0F, -9.81F}}};

    const BodyId floor = world.createBody({
        .type = BodyType::Static,
        .position = {0.0F, -2.0F},
    });
    static_cast<void>(world.createBoxFixture(floor, {.halfExtents = {4.0F, 0.25F}}));

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

    render2d::render::Camera2D camera {640, 360, 80.0F};
    camera.setPosition({0.0F, 0.0F});
    render2d::render::DrawList drawList;
    drawList.addRectangle(
        {0.0F, -2.0F}, {4.0F, 0.25F}, render2d::render::Color::rgb(92, 108, 132), 0);
    drawList.addCircle(state->position, 0.5F, render2d::render::Color::rgb(255, 176, 0), 1);
    drawList.addLine({-4.0F, 0.0F}, {4.0F, 0.0F}, 0.01F,
                     render2d::render::Color::rgb(68, 78, 94), -1);

    render2d::render::Image image {640, 360};
    render2d::render::SoftwareRenderer renderer;
    renderer.render(drawList, camera, image);
    std::filesystem::create_directories("out");
    const std::filesystem::path outputPath = "out/headless_demo.ppm";
    image.writePpm(outputPath);

    std::cout << std::fixed << std::setprecision(3)
              << "ball position: (" << state->position.x << ", " << state->position.y << ")\n"
              << "ball velocity: (" << state->linearVelocity.x << ", " << state->linearVelocity.y << ")\n"
              << "rendered image: " << outputPath.string() << '\n';
}
