# Render2D

Render2D is a native C++20 2D physics and rendering engine. The current
foundation is intentionally headless: fixed-step rigid-body simulation with
circles, contact events, impulse resolution, and no platform dependency.

## Build

```powershell
# In a Visual Studio x64 Developer Command Prompt:
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

The `Debug` configuration is harmless with a single-config generator such as
NMake and is required by multi-config Visual Studio generators.

The first milestone has no third-party dependencies. SDL 3 and the OpenGL
renderer are added after the physics core has a stable regression suite.

## Current API

```cpp
render2d::physics::World world{{.gravity = {0.0F, -9.81F}}};
const auto floor = world.createBody({.type = BodyType::Static});
world.createCircleFixture(floor, {.radius = 0.5F});
```

See [ENGINE_DESIGN.md](ENGINE_DESIGN.md) for the intended architecture and
delivery sequence.
