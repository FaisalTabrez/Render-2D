#include "render2d/physics/world.hpp"
#include "render2d/render/opengl_renderer.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {

constexpr std::uint32_t initialWidth = 960U;
constexpr std::uint32_t initialHeight = 540U;
constexpr float fixedStep = 1.0F / 120.0F;

render2d::render::GlProcAddress resolveOpenGl(const char* const name) {
    return SDL_GL_GetProcAddress(name);
}

void populateDrawList(
    const render2d::physics::World& world,
    const std::vector<render2d::physics::BodyId>& balls,
    const render2d::render::SpriteRegion& sprite,
    render2d::render::DrawList& drawList) {
    drawList.clear();
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
            drawList.addSprite(sprite, state->position, {0.12F, 0.12F},
                               render2d::render::Color {}, 2,
                               static_cast<std::uint64_t>(index));
        }
    }
}

} // namespace

int main(const int argc, char* argv[]) {
    int frameLimit = 0;
    if (argc == 3 && std::string_view {argv[1]} == "--frames") {
        const char* const frameTextEnd = argv[2] + std::char_traits<char>::length(argv[2]);
        const auto [parsedEnd, error] = std::from_chars(argv[2], frameTextEnd, frameLimit);
        if (error != std::errc {} || parsedEnd != frameTextEnd || frameLimit <= 0) {
            std::cerr << "--frames requires a positive integer\n";
            return 1;
        }
    } else if (argc != 1) {
        std::cerr << "Usage: sdl_opengl_sandbox [--frames positive-integer]\n";
        return 1;
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << '\n';
        return 1;
    }
    if (!SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3) ||
        !SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3) ||
        !SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE)) {
        std::cerr << "OpenGL context setup failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Window* const window = SDL_CreateWindow(
        "Render2D — OpenGL Sandbox", static_cast<int>(initialWidth), static_cast<int>(initialHeight),
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (window == nullptr) {
        std::cerr << "Window creation failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }
    SDL_GLContext context = SDL_GL_CreateContext(window);
    if (context == nullptr) {
        std::cerr << "OpenGL context creation failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    static_cast<void>(SDL_GL_SetSwapInterval(1));

    int width = static_cast<int>(initialWidth);
    int height = static_cast<int>(initialHeight);
    if (!SDL_GetWindowSizeInPixels(window, &width, &height)) {
        std::cerr << "Window size query failed: " << SDL_GetError() << '\n';
        SDL_GL_DestroyContext(context);
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

    try {
        render2d::render::Camera2D camera {
            static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), 66.0F};
        camera.setPosition({0.0F, 0.0F});
        render2d::render::OpenGlRenderer renderer {resolveOpenGl};
        render2d::render::DrawList drawList;
        render2d::render::Image spriteImage {2U, 2U};
        spriteImage.setPixel(0U, 0U, render2d::render::Color::rgb(255, 255, 255));
        spriteImage.setPixel(1U, 0U, render2d::render::Color::rgb(255, 220, 90));
        spriteImage.setPixel(0U, 1U, render2d::render::Color::rgb(80, 160, 255));
        spriteImage.setPixel(1U, 1U, render2d::render::Color::rgb(255, 110, 170));
        render2d::render::Texture spriteTexture {std::move(spriteImage)};
        render2d::render::SpriteAtlas spriteAtlas {spriteTexture};
        const render2d::render::SpriteRegion& sprite = spriteAtlas.add(
            "ball-marker", {.x = 0U, .y = 0U, .width = 2U, .height = 2U});
        bool running = true;
        int renderedFrames = 0;
        std::uint64_t previousTicks = SDL_GetTicks();
        double accumulator = 0.0;

        while (running) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
                if (event.type == SDL_EVENT_QUIT ||
                    (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_ESCAPE)) {
                    running = false;
                }
                if (event.type == SDL_EVENT_WINDOW_PIXEL_SIZE_CHANGED) {
                    width = event.window.data1;
                    height = event.window.data2;
                    if (width > 0 && height > 0) {
                        camera.setViewport(static_cast<std::uint32_t>(width),
                                           static_cast<std::uint32_t>(height));
                    }
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

            populateDrawList(world, balls, sprite, drawList);
            renderer.render(drawList, camera);
            if (!SDL_GL_SwapWindow(window)) {
                throw std::runtime_error(SDL_GetError());
            }
            ++renderedFrames;
            if (frameLimit > 0 && renderedFrames >= frameLimit) {
                running = false;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Sandbox error: " << error.what() << '\n';
        SDL_GL_DestroyContext(context);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_GL_DestroyContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
