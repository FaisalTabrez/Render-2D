#pragma once

#include "render2d/math/vec2.hpp"

#include <cstdint>
#include <optional>
#include <set>
#include <span>
#include <vector>

namespace render2d::physics {

using math::Vec2;

constexpr std::uint32_t invalidIndex = 0xFFFF'FFFFU;

struct BodyId {
    std::uint32_t index {invalidIndex};
    std::uint32_t generation {0};

    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return index != invalidIndex;
    }

    constexpr bool operator==(const BodyId&) const noexcept = default;
};

struct FixtureId {
    std::uint32_t index {invalidIndex};
    std::uint32_t generation {0};

    [[nodiscard]] constexpr explicit operator bool() const noexcept {
        return index != invalidIndex;
    }

    constexpr bool operator==(const FixtureId&) const noexcept = default;
};

enum class BodyType {
    Static,
    Kinematic,
    Dynamic,
};

enum class ContactEventType {
    Begin,
    Stay,
    End,
};

struct WorldSettings {
    Vec2 gravity {0.0F, -9.81F};
    std::uint32_t velocityIterations {8};
    float penetrationSlop {0.005F};
    float positionCorrection {0.8F};
};

struct BodyDefinition {
    BodyType type {BodyType::Static};
    Vec2 position {};
    Vec2 linearVelocity {};
    float mass {1.0F};
    float linearDamping {0.0F};
    float gravityScale {1.0F};
};

struct CircleFixtureDefinition {
    float radius {0.5F};
    Vec2 localCenter {};
    float restitution {0.0F};
    bool sensor {false};
};

struct BodyState {
    BodyType type {BodyType::Static};
    Vec2 position {};
    Vec2 previousPosition {};
    Vec2 linearVelocity {};
    float mass {0.0F};
    float linearDamping {0.0F};
    float gravityScale {1.0F};
};

struct ContactEvent {
    ContactEventType type {ContactEventType::Begin};
    FixtureId fixtureA {};
    FixtureId fixtureB {};
    Vec2 normal {};
    float penetration {0.0F};
    bool sensor {false};
};

class World {
public:
    explicit World(WorldSettings settings = {});

    [[nodiscard]] BodyId createBody(const BodyDefinition& definition = {});
    [[nodiscard]] bool destroyBody(BodyId id) noexcept;
    [[nodiscard]] FixtureId createCircleFixture(
        BodyId body, const CircleFixtureDefinition& definition = {});
    [[nodiscard]] bool destroyFixture(FixtureId id) noexcept;

    [[nodiscard]] BodyState* body(BodyId id) noexcept;
    [[nodiscard]] const BodyState* body(BodyId id) const noexcept;
    [[nodiscard]] bool setLinearVelocity(BodyId id, Vec2 velocity) noexcept;
    [[nodiscard]] bool applyForce(BodyId id, Vec2 force) noexcept;

    void step(float deltaTime);

    [[nodiscard]] std::span<const ContactEvent> contactEvents() const noexcept;
    void clearContactEvents() noexcept;
    [[nodiscard]] const WorldSettings& settings() const noexcept;

private:
    struct BodySlot {
        std::uint32_t generation {1};
        std::optional<BodyState> value;
        Vec2 force {};
    };

    struct CircleFixture {
        BodyId body {};
        float radius {0.5F};
        Vec2 localCenter {};
        float restitution {0.0F};
        bool sensor {false};
    };

    struct FixtureSlot {
        std::uint32_t generation {1};
        std::optional<CircleFixture> value;
    };

    struct ContactConstraint {
        FixtureId fixtureA {};
        FixtureId fixtureB {};
        Vec2 normal {};
        float penetration {0.0F};
        bool sensor {false};
    };

    [[nodiscard]] BodySlot* bodySlot(BodyId id) noexcept;
    [[nodiscard]] const BodySlot* bodySlot(BodyId id) const noexcept;
    [[nodiscard]] FixtureSlot* fixtureSlot(FixtureId id) noexcept;
    [[nodiscard]] const FixtureSlot* fixtureSlot(FixtureId id) const noexcept;

    WorldSettings settings_;
    std::vector<BodySlot> bodies_;
    std::vector<FixtureSlot> fixtures_;
    std::vector<ContactConstraint> contacts_;
    std::vector<ContactEvent> events_;
    std::set<std::pair<std::uint64_t, std::uint64_t>> activeContacts_;
};

} // namespace render2d::physics
