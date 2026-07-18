#include "render2d/render/opengl_renderer.hpp"

#define SDL_OPENGL_1_FUNCTION_TYPEDEFS
#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_opengl_glext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace render2d::render {
namespace {

struct Vertex {
    float x {0.0F};
    float y {0.0F};
    float red {1.0F};
    float green {1.0F};
    float blue {1.0F};
    float alpha {1.0F};
};

struct SpriteVertex {
    float x {0.0F};
    float y {0.0F};
    float u {0.0F};
    float v {0.0F};
    float red {1.0F};
    float green {1.0F};
    float blue {1.0F};
    float alpha {1.0F};
};

[[nodiscard]] constexpr std::array<float, 4> normalizedColor(const Color color) noexcept {
    constexpr float divisor = 1.0F / 255.0F;
    return {
        static_cast<float>(color.red) * divisor,
        static_cast<float>(color.green) * divisor,
        static_cast<float>(color.blue) * divisor,
        static_cast<float>(color.alpha) * divisor,
    };
}

[[nodiscard]] math::Vec2 toClipSpace(const math::Vec2 world, const Camera2D& camera) noexcept {
    const math::Vec2 screen = camera.worldToScreen(world);
    return {
        .x = 2.0F * screen.x / static_cast<float>(camera.viewportWidth()) - 1.0F,
        .y = 1.0F - 2.0F * screen.y / static_cast<float>(camera.viewportHeight()),
    };
}

void appendVertex(
    std::vector<Vertex>& vertices,
    const math::Vec2 position,
    const Color color) {
    const auto channels = normalizedColor(color);
    vertices.push_back({
        .x = position.x,
        .y = position.y,
        .red = channels[0],
        .green = channels[1],
        .blue = channels[2],
        .alpha = channels[3],
    });
}

void appendCircle(
    std::vector<Vertex>& vertices,
    const DrawCommand& command,
    const Camera2D& camera) {
    constexpr int segments = 32;
    constexpr float tau = 6.28318530717958647692F;
    const math::Vec2 center = toClipSpace(command.center, camera);
    for (int segment = 0; segment < segments; ++segment) {
        const float firstAngle = tau * static_cast<float>(segment) / static_cast<float>(segments);
        const float secondAngle = tau * static_cast<float>(segment + 1) / static_cast<float>(segments);
        appendVertex(vertices, center, command.color);
        appendVertex(vertices, toClipSpace({
            .x = command.center.x + std::cos(firstAngle) * command.radius,
            .y = command.center.y + std::sin(firstAngle) * command.radius,
        }, camera), command.color);
        appendVertex(vertices, toClipSpace({
            .x = command.center.x + std::cos(secondAngle) * command.radius,
            .y = command.center.y + std::sin(secondAngle) * command.radius,
        }, camera), command.color);
    }
}

void appendRectangle(
    std::vector<Vertex>& vertices,
    const DrawCommand& command,
    const Camera2D& camera) {
    const math::Vec2 bottomLeft = toClipSpace(command.center - command.extent, camera);
    const math::Vec2 bottomRight = toClipSpace({
        .x = command.center.x + command.extent.x,
        .y = command.center.y - command.extent.y,
    }, camera);
    const math::Vec2 topRight = toClipSpace(command.center + command.extent, camera);
    const math::Vec2 topLeft = toClipSpace({
        .x = command.center.x - command.extent.x,
        .y = command.center.y + command.extent.y,
    }, camera);
    appendVertex(vertices, bottomLeft, command.color);
    appendVertex(vertices, bottomRight, command.color);
    appendVertex(vertices, topRight, command.color);
    appendVertex(vertices, bottomLeft, command.color);
    appendVertex(vertices, topRight, command.color);
    appendVertex(vertices, topLeft, command.color);
}

void appendSpriteVertex(
    std::vector<SpriteVertex>& vertices,
    const math::Vec2 position,
    const math::Vec2 uv,
    const Color color) {
    const auto channels = normalizedColor(color);
    vertices.push_back({
        .x = position.x,
        .y = position.y,
        .u = uv.x,
        .v = uv.y,
        .red = channels[0],
        .green = channels[1],
        .blue = channels[2],
        .alpha = channels[3],
    });
}

void appendSprite(
    std::vector<SpriteVertex>& vertices,
    const DrawCommand& command,
    const Camera2D& camera) {
    const math::Vec2 bottomLeft = toClipSpace(command.center - command.extent, camera);
    const math::Vec2 bottomRight = toClipSpace({
        .x = command.center.x + command.extent.x,
        .y = command.center.y - command.extent.y,
    }, camera);
    const math::Vec2 topRight = toClipSpace(command.center + command.extent, camera);
    const math::Vec2 topLeft = toClipSpace({
        .x = command.center.x - command.extent.x,
        .y = command.center.y + command.extent.y,
    }, camera);
    const math::Vec2 bottomLeftUv {command.uvMin.x, command.uvMax.y};
    const math::Vec2 bottomRightUv {command.uvMax.x, command.uvMax.y};
    const math::Vec2 topRightUv {command.uvMax.x, command.uvMin.y};
    const math::Vec2 topLeftUv {command.uvMin.x, command.uvMin.y};
    appendSpriteVertex(vertices, bottomLeft, bottomLeftUv, command.color);
    appendSpriteVertex(vertices, bottomRight, bottomRightUv, command.color);
    appendSpriteVertex(vertices, topRight, topRightUv, command.color);
    appendSpriteVertex(vertices, bottomLeft, bottomLeftUv, command.color);
    appendSpriteVertex(vertices, topRight, topRightUv, command.color);
    appendSpriteVertex(vertices, topLeft, topLeftUv, command.color);
}

void appendLine(
    std::vector<Vertex>& vertices,
    const DrawCommand& command,
    const Camera2D& camera) {
    const math::Vec2 start = camera.worldToScreen(command.center);
    const math::Vec2 end = camera.worldToScreen(command.end);
    const math::Vec2 direction = end - start;
    const float directionLength = math::length(direction);
    if (directionLength <= 1.0e-6F) {
        return;
    }

    const float halfThickness = command.thickness * camera.pixelsPerMetre() * 0.5F;
    const math::Vec2 perpendicular {
        .x = -direction.y / directionLength * halfThickness,
        .y = direction.x / directionLength * halfThickness,
    };
    const auto screenToClip = [&camera](const math::Vec2 screen) {
        return math::Vec2 {
            .x = 2.0F * screen.x / static_cast<float>(camera.viewportWidth()) - 1.0F,
            .y = 1.0F - 2.0F * screen.y / static_cast<float>(camera.viewportHeight()),
        };
    };
    const math::Vec2 first = screenToClip(start + perpendicular);
    const math::Vec2 second = screenToClip(start - perpendicular);
    const math::Vec2 third = screenToClip(end - perpendicular);
    const math::Vec2 fourth = screenToClip(end + perpendicular);
    appendVertex(vertices, first, command.color);
    appendVertex(vertices, second, command.color);
    appendVertex(vertices, third, command.color);
    appendVertex(vertices, first, command.color);
    appendVertex(vertices, third, command.color);
    appendVertex(vertices, fourth, command.color);
}

[[nodiscard]] std::string shaderLog(
    const GLuint shader,
    const PFNGLGETSHADERIVPROC getShaderiv,
    const PFNGLGETSHADERINFOLOGPROC getShaderInfoLog) {
    GLint length = 0;
    getShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
    getShaderInfoLog(shader, length, nullptr, log.data());
    return log;
}

[[nodiscard]] std::string programLog(
    const GLuint program,
    const PFNGLGETPROGRAMIVPROC getProgramiv,
    const PFNGLGETPROGRAMINFOLOGPROC getProgramInfoLog) {
    GLint length = 0;
    getProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
    getProgramInfoLog(program, length, nullptr, log.data());
    return log;
}

} // namespace

class OpenGlRenderer::Impl {
public:
    explicit Impl(const GlProcAddressResolver resolver) {
        if (resolver == nullptr) {
            throw std::invalid_argument("OpenGlRenderer requires an OpenGL procedure resolver");
        }

        createShader_ = load<PFNGLCREATESHADERPROC>(resolver, "glCreateShader");
        shaderSource_ = load<PFNGLSHADERSOURCEPROC>(resolver, "glShaderSource");
        compileShader_ = load<PFNGLCOMPILESHADERPROC>(resolver, "glCompileShader");
        getShaderiv_ = load<PFNGLGETSHADERIVPROC>(resolver, "glGetShaderiv");
        getShaderInfoLog_ = load<PFNGLGETSHADERINFOLOGPROC>(resolver, "glGetShaderInfoLog");
        deleteShader_ = load<PFNGLDELETESHADERPROC>(resolver, "glDeleteShader");
        createProgram_ = load<PFNGLCREATEPROGRAMPROC>(resolver, "glCreateProgram");
        attachShader_ = load<PFNGLATTACHSHADERPROC>(resolver, "glAttachShader");
        linkProgram_ = load<PFNGLLINKPROGRAMPROC>(resolver, "glLinkProgram");
        getProgramiv_ = load<PFNGLGETPROGRAMIVPROC>(resolver, "glGetProgramiv");
        getProgramInfoLog_ = load<PFNGLGETPROGRAMINFOLOGPROC>(resolver, "glGetProgramInfoLog");
        deleteProgram_ = load<PFNGLDELETEPROGRAMPROC>(resolver, "glDeleteProgram");
        useProgram_ = load<PFNGLUSEPROGRAMPROC>(resolver, "glUseProgram");
        genVertexArrays_ = load<PFNGLGENVERTEXARRAYSPROC>(resolver, "glGenVertexArrays");
        bindVertexArray_ = load<PFNGLBINDVERTEXARRAYPROC>(resolver, "glBindVertexArray");
        deleteVertexArrays_ = load<PFNGLDELETEVERTEXARRAYSPROC>(resolver, "glDeleteVertexArrays");
        genBuffers_ = load<PFNGLGENBUFFERSPROC>(resolver, "glGenBuffers");
        bindBuffer_ = load<PFNGLBINDBUFFERPROC>(resolver, "glBindBuffer");
        bufferData_ = load<PFNGLBUFFERDATAPROC>(resolver, "glBufferData");
        deleteBuffers_ = load<PFNGLDELETEBUFFERSPROC>(resolver, "glDeleteBuffers");
        enableVertexAttribArray_ = load<PFNGLENABLEVERTEXATTRIBARRAYPROC>(resolver, "glEnableVertexAttribArray");
        vertexAttribPointer_ = load<PFNGLVERTEXATTRIBPOINTERPROC>(resolver, "glVertexAttribPointer");
        viewport_ = load<PFNGLVIEWPORTPROC>(resolver, "glViewport");
        clearColor_ = load<PFNGLCLEARCOLORPROC>(resolver, "glClearColor");
        clear_ = load<PFNGLCLEARPROC>(resolver, "glClear");
        enable_ = load<PFNGLENABLEPROC>(resolver, "glEnable");
        blendFunc_ = load<PFNGLBLENDFUNCPROC>(resolver, "glBlendFunc");
        drawArrays_ = load<PFNGLDRAWARRAYSPROC>(resolver, "glDrawArrays");
        genTextures_ = load<PFNGLGENTEXTURESPROC>(resolver, "glGenTextures");
        bindTexture_ = load<PFNGLBINDTEXTUREPROC>(resolver, "glBindTexture");
        texImage2D_ = load<PFNGLTEXIMAGE2DPROC>(resolver, "glTexImage2D");
        texParameteri_ = load<PFNGLTEXPARAMETERIPROC>(resolver, "glTexParameteri");
        deleteTextures_ = load<PFNGLDELETETEXTURESPROC>(resolver, "glDeleteTextures");
        activeTexture_ = load<PFNGLACTIVETEXTUREPROC>(resolver, "glActiveTexture");
        getUniformLocation_ = load<PFNGLGETUNIFORMLOCATIONPROC>(resolver, "glGetUniformLocation");
        uniform1i_ = load<PFNGLUNIFORM1IPROC>(resolver, "glUniform1i");

        program_ = createProgram();
        spriteProgram_ = createSpriteProgram();
        genVertexArrays_(1, &vertexArray_);
        genBuffers_(1, &vertexBuffer_);
        bindVertexArray_(vertexArray_);
        bindBuffer_(GL_ARRAY_BUFFER, vertexBuffer_);
        enableVertexAttribArray_(0U);
        vertexAttribPointer_(0U, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);
        enableVertexAttribArray_(1U);
        vertexAttribPointer_(1U, 4, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)),
                             reinterpret_cast<void*>(offsetof(Vertex, red)));

        genVertexArrays_(1, &spriteVertexArray_);
        genBuffers_(1, &spriteVertexBuffer_);
        bindVertexArray_(spriteVertexArray_);
        bindBuffer_(GL_ARRAY_BUFFER, spriteVertexBuffer_);
        enableVertexAttribArray_(0U);
        vertexAttribPointer_(0U, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(SpriteVertex)), nullptr);
        enableVertexAttribArray_(1U);
        vertexAttribPointer_(1U, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(SpriteVertex)),
                             reinterpret_cast<void*>(offsetof(SpriteVertex, u)));
        enableVertexAttribArray_(2U);
        vertexAttribPointer_(2U, 4, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(SpriteVertex)),
                             reinterpret_cast<void*>(offsetof(SpriteVertex, red)));
        useProgram_(spriteProgram_);
        textureSamplerLocation_ = getUniformLocation_(spriteProgram_, "spriteTexture");
        enable_(GL_BLEND);
        blendFunc_(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    ~Impl() {
        for (const auto& [texture, glTexture] : textures_) {
            static_cast<void>(texture);
            deleteTextures_(1, &glTexture);
        }
        if (spriteVertexBuffer_ != 0U) {
            deleteBuffers_(1, &spriteVertexBuffer_);
        }
        if (spriteVertexArray_ != 0U) {
            deleteVertexArrays_(1, &spriteVertexArray_);
        }
        if (vertexBuffer_ != 0U) {
            deleteBuffers_(1, &vertexBuffer_);
        }
        if (vertexArray_ != 0U) {
            deleteVertexArrays_(1, &vertexArray_);
        }
        if (program_ != 0U) {
            deleteProgram_(program_);
        }
        if (spriteProgram_ != 0U) {
            deleteProgram_(spriteProgram_);
        }
    }

    void render(const DrawList& drawList, const Camera2D& camera) {
        const auto clear = normalizedColor(camera.clearColor());
        viewport_(0, 0, static_cast<GLsizei>(camera.viewportWidth()),
                  static_cast<GLsizei>(camera.viewportHeight()));
        clearColor_(clear[0], clear[1], clear[2], clear[3]);
        clear_(GL_COLOR_BUFFER_BIT);

        std::vector<DrawCommand> commands(drawList.commands().begin(), drawList.commands().end());
        std::stable_sort(commands.begin(), commands.end(), [](const DrawCommand& first,
                                                             const DrawCommand& second) {
            return first.layer == second.layer ? first.sortKey < second.sortKey
                                                : first.layer < second.layer;
        });

        std::vector<Vertex> coloredVertices;
        std::vector<SpriteVertex> spriteVertices;
        coloredVertices.reserve(commands.size() * 6U);
        spriteVertices.reserve(commands.size() * 6U);
        const Texture* spriteTexture = nullptr;
        const auto flushColored = [this, &coloredVertices]() {
            drawColored(coloredVertices);
            coloredVertices.clear();
        };
        const auto flushSprites = [this, &spriteVertices, &spriteTexture]() {
            if (spriteTexture != nullptr) {
                drawSprites(spriteVertices, *spriteTexture);
            }
            spriteVertices.clear();
        };

        for (const DrawCommand& command : commands) {
            if (command.primitive == PrimitiveType::Sprite) {
                if (command.texture == nullptr) {
                    continue;
                }
                flushColored();
                if (spriteTexture != command.texture) {
                    flushSprites();
                    spriteTexture = command.texture;
                }
                appendSprite(spriteVertices, command, camera);
                continue;
            }

            flushSprites();
            switch (command.primitive) {
            case PrimitiveType::Circle:
                appendCircle(coloredVertices, command, camera);
                break;
            case PrimitiveType::Rectangle:
                appendRectangle(coloredVertices, command, camera);
                break;
            case PrimitiveType::Line:
                appendLine(coloredVertices, command, camera);
                break;
            case PrimitiveType::Sprite:
                break;
            }
        }

        flushColored();
        flushSprites();
    }

private:
    void drawColored(const std::vector<Vertex>& vertices) const {
        if (vertices.empty()) {
            return;
        }
        useProgram_(program_);
        bindVertexArray_(vertexArray_);
        bindBuffer_(GL_ARRAY_BUFFER, vertexBuffer_);
        bufferData_(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                    vertices.data(), GL_DYNAMIC_DRAW);
        drawArrays_(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));
    }

    void drawSprites(const std::vector<SpriteVertex>& vertices, const Texture& texture) {
        if (vertices.empty()) {
            return;
        }
        useProgram_(spriteProgram_);
        activeTexture_(GL_TEXTURE0);
        bindTexture_(GL_TEXTURE_2D, glTexture(texture));
        uniform1i_(textureSamplerLocation_, 0);
        bindVertexArray_(spriteVertexArray_);
        bindBuffer_(GL_ARRAY_BUFFER, spriteVertexBuffer_);
        bufferData_(GL_ARRAY_BUFFER,
                    static_cast<GLsizeiptr>(vertices.size() * sizeof(SpriteVertex)),
                    vertices.data(), GL_DYNAMIC_DRAW);
        drawArrays_(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices.size()));
    }

    [[nodiscard]] GLuint glTexture(const Texture& texture) {
        if (const auto iterator = textures_.find(&texture); iterator != textures_.end()) {
            return iterator->second;
        }

        GLuint textureId = 0U;
        genTextures_(1, &textureId);
        activeTexture_(GL_TEXTURE0);
        bindTexture_(GL_TEXTURE_2D, textureId);
        texParameteri_(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        texParameteri_(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        texParameteri_(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        texParameteri_(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        const Image& image = texture.image();
        texImage2D_(GL_TEXTURE_2D, 0, GL_RGBA,
                    static_cast<GLsizei>(image.width()), static_cast<GLsizei>(image.height()), 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, image.pixels().data());
        textures_.emplace(&texture, textureId);
        return textureId;
    }

    template <typename Function>
    [[nodiscard]] static Function load(
        const GlProcAddressResolver resolver,
        const char* const name) {
        const GlProcAddress function = resolver(name);
        if (function == nullptr) {
            throw std::runtime_error(std::string("OpenGL function is unavailable: ") + name);
        }
        return reinterpret_cast<Function>(function);
    }

    [[nodiscard]] GLuint compile(const GLenum type, const char* const source) const {
        const GLuint shader = createShader_(type);
        shaderSource_(shader, 1, &source, nullptr);
        compileShader_(shader);
        GLint compiled = GL_FALSE;
        getShaderiv_(shader, GL_COMPILE_STATUS, &compiled);
        if (compiled == GL_TRUE) {
            return shader;
        }
        const std::string log = shaderLog(shader, getShaderiv_, getShaderInfoLog_);
        deleteShader_(shader);
        throw std::runtime_error("OpenGL shader compilation failed: " + log);
    }

    [[nodiscard]] GLuint createProgram() const {
        constexpr const char* vertexSource = R"(
            #version 330 core
            layout (location = 0) in vec2 inPosition;
            layout (location = 1) in vec4 inColor;
            out vec4 color;
            void main() {
                gl_Position = vec4(inPosition, 0.0, 1.0);
                color = inColor;
            }
        )";
        constexpr const char* fragmentSource = R"(
            #version 330 core
            in vec4 color;
            out vec4 outColor;
            void main() {
                outColor = color;
            }
        )";

        const GLuint vertexShader = compile(GL_VERTEX_SHADER, vertexSource);
        const GLuint fragmentShader = compile(GL_FRAGMENT_SHADER, fragmentSource);
        const GLuint program = createProgram_();
        attachShader_(program, vertexShader);
        attachShader_(program, fragmentShader);
        linkProgram_(program);
        deleteShader_(vertexShader);
        deleteShader_(fragmentShader);

        GLint linked = GL_FALSE;
        getProgramiv_(program, GL_LINK_STATUS, &linked);
        if (linked == GL_TRUE) {
            return program;
        }
        const std::string log = programLog(program, getProgramiv_, getProgramInfoLog_);
        deleteProgram_(program);
        throw std::runtime_error("OpenGL program link failed: " + log);
    }

    [[nodiscard]] GLuint createSpriteProgram() const {
        constexpr const char* vertexSource = R"(
            #version 330 core
            layout (location = 0) in vec2 inPosition;
            layout (location = 1) in vec2 inUv;
            layout (location = 2) in vec4 inColor;
            out vec2 uv;
            out vec4 color;
            void main() {
                gl_Position = vec4(inPosition, 0.0, 1.0);
                uv = inUv;
                color = inColor;
            }
        )";
        constexpr const char* fragmentSource = R"(
            #version 330 core
            in vec2 uv;
            in vec4 color;
            uniform sampler2D spriteTexture;
            out vec4 outColor;
            void main() {
                outColor = texture(spriteTexture, uv) * color;
            }
        )";

        const GLuint vertexShader = compile(GL_VERTEX_SHADER, vertexSource);
        const GLuint fragmentShader = compile(GL_FRAGMENT_SHADER, fragmentSource);
        const GLuint program = createProgram_();
        attachShader_(program, vertexShader);
        attachShader_(program, fragmentShader);
        linkProgram_(program);
        deleteShader_(vertexShader);
        deleteShader_(fragmentShader);

        GLint linked = GL_FALSE;
        getProgramiv_(program, GL_LINK_STATUS, &linked);
        if (linked == GL_TRUE) {
            return program;
        }
        const std::string log = programLog(program, getProgramiv_, getProgramInfoLog_);
        deleteProgram_(program);
        throw std::runtime_error("OpenGL sprite program link failed: " + log);
    }

    PFNGLCREATESHADERPROC createShader_ {nullptr};
    PFNGLSHADERSOURCEPROC shaderSource_ {nullptr};
    PFNGLCOMPILESHADERPROC compileShader_ {nullptr};
    PFNGLGETSHADERIVPROC getShaderiv_ {nullptr};
    PFNGLGETSHADERINFOLOGPROC getShaderInfoLog_ {nullptr};
    PFNGLDELETESHADERPROC deleteShader_ {nullptr};
    PFNGLCREATEPROGRAMPROC createProgram_ {nullptr};
    PFNGLATTACHSHADERPROC attachShader_ {nullptr};
    PFNGLLINKPROGRAMPROC linkProgram_ {nullptr};
    PFNGLGETPROGRAMIVPROC getProgramiv_ {nullptr};
    PFNGLGETPROGRAMINFOLOGPROC getProgramInfoLog_ {nullptr};
    PFNGLDELETEPROGRAMPROC deleteProgram_ {nullptr};
    PFNGLUSEPROGRAMPROC useProgram_ {nullptr};
    PFNGLGENVERTEXARRAYSPROC genVertexArrays_ {nullptr};
    PFNGLBINDVERTEXARRAYPROC bindVertexArray_ {nullptr};
    PFNGLDELETEVERTEXARRAYSPROC deleteVertexArrays_ {nullptr};
    PFNGLGENBUFFERSPROC genBuffers_ {nullptr};
    PFNGLBINDBUFFERPROC bindBuffer_ {nullptr};
    PFNGLBUFFERDATAPROC bufferData_ {nullptr};
    PFNGLDELETEBUFFERSPROC deleteBuffers_ {nullptr};
    PFNGLENABLEVERTEXATTRIBARRAYPROC enableVertexAttribArray_ {nullptr};
    PFNGLVERTEXATTRIBPOINTERPROC vertexAttribPointer_ {nullptr};
    PFNGLVIEWPORTPROC viewport_ {nullptr};
    PFNGLCLEARCOLORPROC clearColor_ {nullptr};
    PFNGLCLEARPROC clear_ {nullptr};
    PFNGLENABLEPROC enable_ {nullptr};
    PFNGLBLENDFUNCPROC blendFunc_ {nullptr};
    PFNGLDRAWARRAYSPROC drawArrays_ {nullptr};
    PFNGLGENTEXTURESPROC genTextures_ {nullptr};
    PFNGLBINDTEXTUREPROC bindTexture_ {nullptr};
    PFNGLTEXIMAGE2DPROC texImage2D_ {nullptr};
    PFNGLTEXPARAMETERIPROC texParameteri_ {nullptr};
    PFNGLDELETETEXTURESPROC deleteTextures_ {nullptr};
    PFNGLACTIVETEXTUREPROC activeTexture_ {nullptr};
    PFNGLGETUNIFORMLOCATIONPROC getUniformLocation_ {nullptr};
    PFNGLUNIFORM1IPROC uniform1i_ {nullptr};
    GLuint program_ {0U};
    GLuint spriteProgram_ {0U};
    GLuint vertexArray_ {0U};
    GLuint vertexBuffer_ {0U};
    GLuint spriteVertexArray_ {0U};
    GLuint spriteVertexBuffer_ {0U};
    GLint textureSamplerLocation_ {-1};
    std::unordered_map<const Texture*, GLuint> textures_;
};

OpenGlRenderer::OpenGlRenderer(const GlProcAddressResolver resolveProcAddress)
    : impl_(std::make_unique<Impl>(resolveProcAddress)) {}

OpenGlRenderer::~OpenGlRenderer() = default;

void OpenGlRenderer::render(const DrawList& drawList, const Camera2D& camera) {
    impl_->render(drawList, camera);
}

} // namespace render2d::render
