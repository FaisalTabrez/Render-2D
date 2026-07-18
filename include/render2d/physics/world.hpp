#pragma once

#include "render2d/math/aabb.hpp"

#include <cstdint>
#include <cstddef>
#include <optional>
#include <set>
#include <span>
#include <vector>

namespace render2d::physics {

using math::Vec2;
using math::Aabb;

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

enum class ShapeType {
    Circle,
    Box,
};

struct CollisionFilter {
    std::uint32_t categoryBits {0x0000'0001U};
    std::uint32_t maskBits {0xFFFF'FFFFU};
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
    float angle {0.0F};
    float angularVelocity {0.0F};
    float mass {1.0F};
    float momentOfInertia {1.0F};
    float linearDamping {0.0F};
    float angularDamping {0.0F};
    float gravityScale {1.0F};
    bool fixedRotation {false};
};

struct CircleFixtureDefinition {
    float radius {0.5F};
    Vec2 localCenter {};
    float friction {0.5F};
    float restitution {0.0F};
    bool sensor {false};
    CollisionFilter filter {};
};

struct BoxFixtureDefinition {
    Vec2 halfExtents {0.5F, 0.5F};
    Vec2 localCenter {};
    float friction {0.5F};
    float restitution {0.0F};
    bool sensor {false};
    CollisionFilter filter {};
};

struct BodyState {
    BodyType type {BodyType::Static};
    Vec2 position {};
    Vec2 previousPosition {};
    Vec2 linearVelocity {};
    float angle {0.0F};
    float previousAngle {0.0F};
    float angularVelocity {0.0F};
    float mass {0.0F};
    float momentOfInertia {0.0F};
    float linearDamping {0.0F};
    float angularDamping {0.0F};
    float gravityScale {1.0F};
    bool fixedRotation {false};
};

struct ContactEvent {
    ContactEventType type {ContactEventType::Begin};
    FixtureId fixtureA {};
    FixtureId fixtureB {};
    Vec2 normal {};
    float penetration {0.0F};
    bool sensor {false};
};

struct WorldStats {
    std::size_t activeBodies {0};
    std::size_t activeFixtures {0};
    std::size_t broadPhasePairTests {0};
    std::size_t broadPhaseCandidatePairs {0};
    std::size_t narrowPhaseTests {0};
    std::size_t activeContacts {0};
};

class World {
public:
    explicit World(WorldSettings settings = {});

    [[nodiscard]] BodyId createBody(const BodyDefinition& definition = {});
    [[nodiscard]] bool destroyBody(BodyId id) noexcept;
    [[nodiscard]] FixtureId createCircleFixture(
        BodyId body, const CircleFixtureDefinition& definition = {});
    [[nodiscard]] FixtureId createBoxFixture(
        BodyId body, const BoxFixtureDefinition& definition = {});
    [[nodiscard]] bool destroyFixture(FixtureId id) noexcept;

    [[nodiscard]] BodyState* body(BodyId id) noexcept;
    [[nodiscard]] const BodyState* body(BodyId id) const noexcept;
    [[nodiscard]] bool setLinearVelocity(BodyId id, Vec2 velocity) noexcept;
    [[nodiscard]] bool setAngularVelocity(BodyId id, float angularVelocity) noexcept;
    [[nodiscard]] bool applyForce(BodyId id, Vec2 force) noexcept;
    [[nodiscard]] bool applyForceAtPoint(BodyId id, Vec2 force, Vec2 worldPoint) noexcept;
    [[nodiscard]] bool applyTorque(BodyId id, float torque) noexcept;

    [[nodiscard]] std::vector<FixtureId> queryAabb(const Aabb& bounds) const;

    void step(float deltaTime);

    [[nodiscard]] std::span<const ContactEvent> contactEvents() const noexcept;
    void clearContactEvents() noexcept;
    [[nodiscard]] const WorldSettings& settings() const noexcept;
    [[nodiscard]] const WorldStats& stats() const noexcept;

private:
    struct BodySlot {
        std::uint32_t generation {1};
        std::optional<BodyState> value;
        Vec2 force {};
        float torque {0.0F};
    };

    struct Fixture {
        BodyId body {};
        ShapeType shape {ShapeType::Circle};
        float radius {0.5F};
        Vec2 halfExtents {0.5F, 0.5F};
        Vec2 localCenter {};
        float friction {0.5F};
        float restitution {0.0F};
        bool sensor {false};
        CollisionFilter filter {};
    };

    struct FixtureSlot {
        std::uint32_t generation {1};
        std::optional<Fixture> value;
    };

    struct ContactConstraint {
        FixtureId fixtureA {};
        FixtureId fixtureB {};
        Vec2 normal {};
        Vec2 point {};
        float penetration {0.0F};
        bool sensor {false};
        float accumulatedNormalImpulse {0.0F};
        float accumulatedTangentImpulse {0.0F};
    };

    [[nodiscard]] BodySlot* bodySlot(BodyId id) noexcept;
    [[nodiscard]] const BodySlot* bodySlot(BodyId id) const noexcept;
    [[nodiscard]] FixtureSlot* fixtureSlot(FixtureId id) noexcept;
    [[nodiscard]] const FixtureSlot* fixtureSlot(FixtureId id) const noexcept;
    [[nodiscard]] Aabb fixtureAabb(const Fixture& fixture) const noexcept;

    WorldSettings settings_;
    std::vector<BodySlot> bodies_;
    std::vector<FixtureSlot> fixtures_;
    std::vector<ContactConstraint> contacts_;
    std::vector<ContactEvent> events_;
    std::set<std::pair<std::uint64_t, std::uint64_t>> activeContacts_;
    WorldStats stats_;
};

} // namespace render2d::physics
