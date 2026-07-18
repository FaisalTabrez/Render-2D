# Render2D

Render2D is a native C++20 2D physics and rendering engine. Its core has no
platform dependency and currently provides fixed-step rigid-body simulation,
circle/box collisions, contact events, spatial queries, a draw list, camera,
and a portable software reference renderer.

## Build

```powershell
# In a Visual Studio x64 Developer Command Prompt:
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

The `Debug` configuration is harmless with a single-config generator such as
NMake and is required by multi-config Visual Studio generators.

The core library and its tests have no third-party dependencies. The optional
SDL 3 sandbox presents the reference-rendered frame in a native window. A
direct OpenGL draw-list backend is the next rendering milestone.

## Interactive desktop sandbox

The SDL sandbox opens a native window and displays frames produced by the
engine's reference renderer. It is an optional target, keeping the library
headless and testable by default.

```powershell
cmake -S . -B build-sdl `
  -DCMAKE_TOOLCHAIN_FILE="C:/Program Files/Microsoft Visual Studio/18/Community/VC/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DRENDER2D_BUILD_SDL_SANDBOX=ON
cmake --build build-sdl --config Debug --target sdl_sandbox
```

Use Escape or close the window to exit. This CPU-upload path remains the
reference presentation backend; the direct GPU path is described below.

## Direct OpenGL sandbox

The OpenGL sandbox uses an OpenGL 3.3 Core context and renders the draw list
directly on the GPU. It loads the required functions through SDL, compiles its
own shaders, and batches colored circles, rectangles, and lines into one draw
call per frame.

```powershell
cmake --build build-sdl --config Debug --target sdl_opengl_sandbox
.\build-sdl\sdl_opengl_sandbox.exe
```

For a short non-interactive graphics smoke test, use:

```powershell
.\build-sdl\sdl_opengl_sandbox.exe --frames 3
```

## Implemented features

- Fixed-timestep dynamic, kinematic, and static bodies
- Angular velocity, torque, damping, and off-center force application
- Configurable sleeping and explicit wake-up for inactive dynamic bodies
- Distance joints with local anchors and deterministic sequential-impulse solving
- Revolute joints with free angular motion around coincident local anchors
- Prismatic joints with free travel along a local axis
- Bullet CCD for high-speed circular dynamic bodies against static or kinematic geometry
- Deterministic per-pair contact-impulse caching and warm starting
- Circle, oriented-box, and convex-polygon contacts with SAT (up to 8 vertices)
- Deterministic dynamic AABB-tree broad phase, collision layers, AABB and ray-cast queries, friction, restitution,
  force integration, and contact lifecycle events
- Stable render layers for circles, rectangles, and lines
- Camera pan, zoom, and rotation; alpha compositing; portable PPM frame output
- SDL native-window sandbox with a reproducible vcpkg manifest
- Direct OpenGL 3.3 primitive backend with a three-frame smoke-test mode
- Texture assets from binary PPM files, named atlas regions, sprites, and
  tile-map draw submission through either backend

## Sprite assets and tile maps

`TextureLibrary` owns image-backed textures, `SpriteAtlas` maps names to pixel
regions, and `TileMap` emits layer-sorted sprite commands. The software and
OpenGL renderers use the same `DrawList` commands. The current loader supports
binary P6 PPM images; PNG and compressed image decoding are the next asset
pipeline addition.

## Current API

```cpp
render2d::physics::World world{{.gravity = {0.0F, -9.81F}}};
const auto floor = world.createBody({.type = BodyType::Static});
world.createCircleFixture(floor, {.radius = 0.5F});
```

See [ENGINE_DESIGN.md](ENGINE_DESIGN.md) for the intended architecture and
delivery sequence.
