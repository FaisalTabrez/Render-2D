# Render2D

Render2D is a native C++20 2D physics and rendering engine for small games,
simulations, and editor tooling. The physics core is headless and independent
of SDL, OpenGL, and platform APIs; rendering is submitted through a stable
draw list and can target the software reference renderer or the OpenGL 3.3
backend.

## Highlights

- Fixed-step static, kinematic, and dynamic rigid bodies with forces, torque,
  damping, sleeping, collision filtering, sensors, friction, and restitution.
- Circle, oriented-box, and convex-polygon fixtures (up to eight vertices),
  SAT collision detection, clipped polygon contact patches, lifecycle events,
  and warm-started impulses.
- A deterministic AABB-tree broad phase, AABB/ray queries, and bounded CCD
  for circular bullet bodies against static or kinematic geometry.
- Distance, revolute, and prismatic joints; prismatic limits and force-limited
  motors are included.
- A portable software renderer, camera, draw layers, sprites, atlas regions,
  tile-map submission, binary PPM textures, plus an optional SDL/OpenGL 3.3
  presentation path.
- Physics debug draw, per-step telemetry, deterministic replay coverage, and a
  1,000-body sparse-scene regression test.

## Repository layout

| Path | Purpose |
| --- | --- |
| `include/render2d/physics` | Public physics API and value types. |
| `include/render2d/render` | Draw list, camera, images, textures, sprites, and renderers. |
| `include/render2d/debug` | Physics-to-draw-list diagnostics adapter. |
| `src` | Physics, software renderer, OpenGL renderer, and debug implementation. |
| `examples` | Headless, SDL software, and SDL OpenGL sandboxes. |
| `tests` | Physics, rendering, stress, replay, and visual-regression checks. |

## Build and test

The core library and test target have no runtime platform dependency.

```powershell
# Run from a Visual Studio x64 Developer Command Prompt.
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

`--config Debug` is required by multi-config generators and is harmless for
single-config generators.

## SDL sandboxes

SDL 3 is resolved through the included vcpkg manifest. Configure the optional
desktop targets as follows:

```powershell
cmake -S . -B build-sdl `
  -DCMAKE_TOOLCHAIN_FILE="C:/Program Files/Microsoft Visual Studio/18/Community/VC/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DRENDER2D_BUILD_SDL_SANDBOX=ON
cmake --build build-sdl
```

Run the software-reference window:

```powershell
.\build-sdl\sdl_sandbox.exe
```

Run the direct OpenGL 3.3 window:

```powershell
.\build-sdl\sdl_opengl_sandbox.exe
```

For a non-interactive OpenGL smoke test:

```powershell
.\build-sdl\sdl_opengl_sandbox.exe --frames 3
```

## Minimal physics example

```cpp
#include "render2d/physics/world.hpp"

using namespace render2d::physics;

World world{{.gravity = {0.0F, -9.81F}}};

const BodyId floor = world.createBody({
    .type = BodyType::Static,
    .position = {0.0F, -2.0F},
});
world.createBoxFixture(floor, {.halfExtents = {8.0F, 0.5F}});

const BodyId ball = world.createBody({
    .type = BodyType::Dynamic,
    .position = {0.0F, 2.0F},
    .mass = 1.0F,
});
world.createCircleFixture(ball, {.radius = 0.4F, .restitution = 0.3F});

world.step(1.0F / 120.0F);
```

## Diagnostics and verification

`WorldStats` reports body/fixture/contact counts, broad-phase activity, CCD
tests and hits, warm starts, solver iterations, and elapsed step time.
`PhysicsDebugRenderer` appends fixtures, AABBs, contact normals, joints, and
sleeping-body state to a normal `render::DrawList`.

The regression suite covers contact lifecycle, filters, joints and motors,
bullet CCD including multiple impacts, polygon contacts, software-rendered
debug output, exact replay on the same build, and a 1,000-body sparse scene.

## Scope

Render2D is intentionally compact. It supports convex rigid-body gameplay
physics and desktop rendering; it is not a fixed-point networking engine,
soft-body/fluids system, or general image-decoding library. Texture loading is
currently limited to binary P6 PPM files. See [ENGINE_DESIGN.md](ENGINE_DESIGN.md)
for the design rationale and architecture.
