#include "render2d/physics/world.hpp"
#include "render2d/render/software_renderer.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <vector>

namespace {

constexpr std::uint32_t imageWidth = 960U;
constexpr std::uint32_t imageHeight = 540U;
constexpr float fixedStep = 1.0F / 120.0F;

void drawWorld(
    const render2d::physics::World& world,
    const std::vector<render2d::physics::BodyId>& balls,
    render2d::render::SoftwareRenderer& renderer,
    render2d::render::Image& image,
    const render2d::render::Camera2D& camera) {
    render2d::render::DrawList drawList;
    drawList.addRectangle(
        {0.0F, -3.0F}, {7.0F, 0.3F}, render2d::render::Color::rgb(84, 97, 116), 0);
    drawList.addRectangle(
        {-6.5F, 0.0F}, {0.25F, 4.0F}, render2d::render::Color::rgb(84, 97, 116), 0);
    drawList.addRectangle(
        {6.5F, 0.0F}, {0.25F, 4.0F}, render2d::render::Color::rgb(84, 97, 116), 0);
    drawList.addLine({-6.0F, 0.0F}, {6.0F, 0.0F}, 0.012F,
                     render2d::render::Color::rgb(57, 67, 82), -1);

    constexpr std::array palette {
        render2d::render::Color::rgb(255, 176, 0),
        render2d::render::Color::rgb(68, 190, 173),
        render2d::render::Color::rgb(235, 100, 125),
        render2d::render::Color::rgb(111, 151, 255),
        render2d::render::Color::rgb(190, 130, 255),
    };
    for (std::size_t index = 0; index < balls.size(); ++index) {
        const render2d::physics::BodyState* const state = world.body(balls[index]);
        if (state != nullptr) {
            drawList.addCircle(state->position, 0.32F, palette[index % palette.size()], 1,
                               static_cast<std::uint64_t>(index));
        }
    }
    renderer.render(drawList, camera, image);
}

} // namespace

int main() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* const window = SDL_CreateWindow(
        "Render2D — SDL Sandbox", static_cast<int>(imageWidth), static_cast<int>(imageHeight),
        SDL_WINDOW_RESIZABLE);
    if (window == nullptr) {
        std::cerr << "Window creation failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* const windowRenderer = SDL_CreateRenderer(window, nullptr);
    if (windowRenderer == nullptr) {
        std::cerr << "Renderer creation failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Texture* const texture = SDL_CreateTexture(
        windowRenderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING,
        static_cast<int>(imageWidth), static_cast<int>(imageHeight));
    if (texture == nullptr) {
        std::cerr << "Texture creation failed: " << SDL_GetError() << '\n';
        SDL_DestroyRenderer(windowRenderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    using namespace render2d::physics;
    World world {{.gravity = {0.0F, -9.81F}}};
    const BodyId floor = world.createBody({.type = BodyType::Static, .position = {0.0F, -3.0F}});
    static_cast<void>(world.createBoxFixture(floor, {.halfExtents = {7.0F, 0.3F}, .friction = 0.8F}));
    const BodyId leftWall = world.createBody({.type = BodyType::Static, .position = {-6.5F, 0.0F}});
    static_cast<void>(world.createBoxFixture(leftWall, {.halfExtents = {0.25F, 4.0F}, .friction = 0.8F}));
    const BodyId rightWall = world.createBody({.type = BodyType::Static, .position = {6.5F, 0.0F}});
    static_cast<void>(world.createBoxFixture(rightWall, {.halfExtents = {0.25F, 4.0F}, .friction = 0.8F}));

    std::vector<BodyId> balls;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 5; ++column) {
            const BodyId ball = world.createBody({
                .type = BodyType::Dynamic,
                .position = {-2.0F + static_cast<float>(column) * 1.0F,
                             -1.0F + static_cast<float>(row) * 0.9F},
                .mass = 1.0F,
                .linearDamping = 0.015F,
            });
            static_cast<void>(world.createCircleFixture(ball, {
                .radius = 0.32F,
                .friction = 0.45F,
                .restitution = 0.38F,
            }));
            balls.push_back(ball);
        }
    }

    render2d::render::Camera2D camera {imageWidth, imageHeight, 66.0F};
    camera.setPosition({0.0F, 0.0F});
    render2d::render::Image image {imageWidth, imageHeight};
    render2d::render::SoftwareRenderer softwareRenderer;

    bool running = true;
    std::uint64_t previousTicks = SDL_GetTicks();
    double accumulator = 0.0;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT ||
                (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                running = false;
            }
        }

        const std::uint64_t currentTicks = SDL_GetTicks();
        const double elapsed = std::min(
            static_cast<double>(currentTicks - previousTicks) / 1000.0, 0.25);
        previousTicks = currentTicks;
        accumulator += elapsed;
        while (accumulator >= fixedStep) {
            world.step(fixedStep);
            accumulator -= fixedStep;
        }

        drawWorld(world, balls, softwareRenderer, image, camera);
        if (!SDL_UpdateTexture(
                texture, nullptr, image.pixels().data(),
                static_cast<int>(imageWidth * sizeof(render2d::render::Color)))) {
            std::cerr << "Texture update failed: " << SDL_GetError() << '\n';
            running = false;
        }
        SDL_RenderClear(windowRenderer);
        SDL_RenderTexture(windowRenderer, texture, nullptr, nullptr);
        SDL_RenderPresent(windowRenderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(windowRenderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
