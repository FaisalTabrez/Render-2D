cmake_minimum_required(VERSION 3.24)

project(Render2D VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(RENDER2D_BUILD_SDL_SANDBOX "Build the interactive SDL desktop sandbox" OFF)

add_library(render2d STATIC
    src/physics/world.cpp
    src/debug/physics_debug_renderer.cpp
    src/render/software_renderer.cpp
)

add_library(render2d::render2d ALIAS render2d)

target_include_directories(render2d
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

if(MSVC)
    target_compile_options(render2d PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(render2d PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

add_executable(headless_demo examples/headless_demo.cpp)
target_link_libraries(headless_demo PRIVATE render2d::render2d)

enable_testing()

add_executable(render2d_tests tests/physics_tests.cpp)
target_link_libraries(render2d_tests PRIVATE render2d::render2d)
add_test(NAME render2d_tests COMMAND render2d_tests)

if(RENDER2D_BUILD_SDL_SANDBOX)
    find_package(SDL3 CONFIG REQUIRED)

    add_library(render2d_opengl STATIC src/render/opengl_renderer.cpp)
    target_link_libraries(render2d_opengl PUBLIC render2d::render2d SDL3::SDL3)
    target_include_directories(render2d_opengl PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
    if(MSVC)
        target_compile_options(render2d_opengl PRIVATE /W4 /permissive- /EHsc)
    else()
        target_compile_options(render2d_opengl PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
    endif()

    add_executable(sdl_sandbox examples/sdl_sandbox.cpp)
    target_link_libraries(sdl_sandbox PRIVATE render2d::render2d SDL3::SDL3)

    add_executable(sdl_opengl_sandbox examples/sdl_opengl_sandbox.cpp)
    target_link_libraries(sdl_opengl_sandbox PRIVATE render2d_opengl)
endif()
