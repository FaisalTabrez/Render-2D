#include "render2d/physics/world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace render2d::physics {
namespace {

using math::dot;
using math::isFinite;
using math::length;
using math::lengthSquared;
using math::normalized;
using math::overlaps;
using math::rotate;

[[nodiscard]] constexpr std::uint64_t fixtureKey(const FixtureId id) noexcept {
    return (static_cast<std::uint64_t>(id.generation) << 32U) | id.index;
}

[[nodiscard]] constexpr FixtureId fixtureIdFromKey(const std::uint64_t key) noexcept {
    return {
        .index = static_cast<std::uint32_t>(key & 0xFFFF'FFFFULL),
        .generation = static_cast<std::uint32_t>(key >> 32U),
    };
}

[[nodiscard]] constexpr std::uint32_t nextGeneration(const std::uint32_t generation) noexcept {
    return generation == std::numeric_limits<std::uint32_t>::max() ? 1U : generation + 1U;
}

[[nodiscard]] constexpr bool movesWithVelocity(const BodyType type) noexcept {
    return type == BodyType::Dynamic || type == BodyType::Kinematic;
}

[[nodiscard]] constexpr float inverseMass(const BodyState& body) noexcept {
    return body.type == BodyType::Dynamic ? 1.0F / body.mass : 0.0F;
}

[[nodiscard]] constexpr float inverseInertia(const BodyState& body) noexcept {
    return body.type == BodyType::Dynamic && !body.fixedRotation
        ? 1.0F / body.momentOfInertia
        : 0.0F;
}

[[nodiscard]] constexpr bool canCollide(
    const CollisionFilter first, const CollisionFilter second) noexcept {
    return (first.maskBits & second.categoryBits) != 0U &&
           (second.maskBits & first.categoryBits) != 0U;
}

[[nodiscard]] constexpr float mixRestitution(const float first, const float second) noexcept {
    return std::max(first, second);
}

[[nodiscard]] inline float mixFriction(const float first, const float second) noexcept {
    return std::sqrt(first * second);
}

[[nodiscard]] inline bool isValidMaterial(
    const float friction, const float restitution) noexcept {
    return std::isfinite(friction) && std::isfinite(restitution) && friction >= 0.0F &&
           restitution >= 0.0F && restitution <= 1.0F;
}

[[nodiscard]] float signedArea(const std::vector<Vec2>& vertices) noexcept {
    float twiceArea = 0.0F;
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        twiceArea += math::cross(vertices[index], vertices[(index + 1U) % vertices.size()]);
    }
    return twiceArea * 0.5F;
}

[[nodiscard]] bool isStrictlyConvexCounterClockwise(const std::vector<Vec2>& vertices) noexcept {
    if (vertices.size() < 3U || vertices.size() > 8U) {
        return false;
    }
    for (const Vec2 vertex : vertices) {
        if (!isFinite(vertex)) {
            return false;
        }
    }
    for (std::size_t index = 0; index < vertices.size(); ++index) {
        const Vec2 firstEdge = vertices[(index + 1U) % vertices.size()] - vertices[index];
        const Vec2 secondEdge = vertices[(index + 2U) % vertices.size()] -
            vertices[(index + 1U) % vertices.size()];
        if (math::cross(firstEdge, secondEdge) <= 1.0e-6F) {
            return false;
        }
    }
    return signedArea(vertices) > 1.0e-6F;
}

struct SweepHit {
    float fraction {1.0F};
    Vec2 normal {};
};

[[nodiscard]] bool sweepCircleAgainstCircle(
    const Vec2 start,
    const Vec2 translation,
    const float radius,
    const Vec2 targetCenter,
    const float targetRadius,
    SweepHit& hit) noexcept {
    const Vec2 offset = start - targetCenter;
    const float combinedRadius = radius + targetRadius;
    const float translationLengthSquared = lengthSquared(translation);
    const float c = lengthSquared(offset) - combinedRadius * combinedRadius;
    if (c <= 0.0F || translationLengthSquared <= 1.0e-12F) {
        return false;
    }

    const float b = dot(offset, translation);
    const float discriminant = b * b - translationLengthSquared * c;
    if (discriminant < 0.0F) {
        return false;
    }
    const float fraction = (-b - std::sqrt(discriminant)) / translationLengthSquared;
    if (fraction < 0.0F || fraction > 1.0F) {
        return false;
    }

    hit.fraction = fraction;
    hit.normal = normalized(start + translation * fraction - targetCenter);
    return true;
}

[[nodiscard]] bool sweepCircleAgainstAabb(
    const Vec2 start,
    const Vec2 translation,
    const float radius,
    Aabb targetBounds,
    SweepHit& hit) noexcept {
    targetBounds.min -= Vec2 {radius, radius};
    targetBounds.max += Vec2 {radius, radius};

    float entry = 0.0F;
    float exit = 1.0F;
    Vec2 entryNormal {};
    const auto intersectAxis = [&](const float startAxis, const float translationAxis,
                                   const float minimum, const float maximum,
                                   const Vec2 minimumNormal, const Vec2 maximumNormal) {
        if (std::abs(translationAxis) <= 1.0e-12F) {
            return startAxis >= minimum && startAxis <= maximum;
        }
        float nearFraction = (minimum - startAxis) / translationAxis;
        float farFraction = (maximum - startAxis) / translationAxis;
        Vec2 nearNormal = minimumNormal;
        if (nearFraction > farFraction) {
            std::swap(nearFraction, farFraction);
            nearNormal = maximumNormal;
        }
        if (nearFraction > entry) {
            entry = nearFraction;
            entryNormal = nearNormal;
        }
        exit = std::min(exit, farFraction);
        return entry <= exit;
    };

    if (!intersectAxis(start.x, translation.x, targetBounds.min.x, targetBounds.max.x,
                       {-1.0F, 0.0F}, {1.0F, 0.0F}) ||
        !intersectAxis(start.y, translation.y, targetBounds.min.y, targetBounds.max.y,
                       {0.0F, -1.0F}, {0.0F, 1.0F}) || entry < 0.0F || entry > 1.0F) {
        return false;
    }

    hit.fraction = entry;
    hit.normal = entryNormal;
    return true;
}

} // namespace

World::World(WorldSettings settings) : settings_(settings) {
    if (!isFinite(settings_.gravity) || settings_.velocityIterations == 0U ||
        settings_.penetrationSlop < 0.0F || settings_.positionCorrection < 0.0F ||
        settings_.positionCorrection > 1.0F || settings_.sleepLinearThreshold < 0.0F ||
        settings_.sleepAngularThreshold < 0.0F || settings_.sleepDelay < 0.0F) {
        throw std::invalid_argument("WorldSettings contains an invalid value");
    }
}

BodyId World::createBody(const BodyDefinition& definition) {
    if (!isFinite(definition.position) || !isFinite(definition.linearVelocity) ||
        !std::isfinite(definition.angle) || !std::isfinite(definition.angularVelocity) ||
        !std::isfinite(definition.mass) || !std::isfinite(definition.momentOfInertia) ||
        !std::isfinite(definition.linearDamping) || !std::isfinite(definition.angularDamping) ||
        !std::isfinite(definition.gravityScale) || definition.linearDamping < 0.0F ||
        definition.angularDamping < 0.0F ||
        (definition.type == BodyType::Dynamic &&
         (definition.mass <= 0.0F || definition.momentOfInertia <= 0.0F))) {
        throw std::invalid_argument("BodyDefinition contains an invalid value");
    }

    BodyState state {
        .type = definition.type,
        .position = definition.position,
        .previousPosition = definition.position,
        .linearVelocity = definition.linearVelocity,
        .angle = definition.angle,
        .previousAngle = definition.angle,
        .angularVelocity = definition.angularVelocity,
        .mass = definition.type == BodyType::Dynamic ? definition.mass : 0.0F,
        .momentOfInertia = definition.type == BodyType::Dynamic ? definition.momentOfInertia : 0.0F,
        .linearDamping = definition.linearDamping,
        .angularDamping = definition.angularDamping,
        .gravityScale = definition.gravityScale,
        .fixedRotation = definition.fixedRotation,
        .bullet = definition.bullet,
    };

    for (std::uint32_t index = 0; index < bodies_.size(); ++index) {
        BodySlot& slot = bodies_[index];
        if (!slot.value.has_value()) {
            slot.value = state;
            slot.force = {};
            slot.torque = 0.0F;
            return {.index = index, .generation = slot.generation};
        }
    }

    bodies_.push_back(BodySlot {.value = state});
    return {.index = static_cast<std::uint32_t>(bodies_.size() - 1U), .generation = 1U};
}

bool World::destroyBody(const BodyId id) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr) {
        return false;
    }

    for (FixtureSlot& fixtureSlot : fixtures_) {
        if (fixtureSlot.value.has_value() && fixtureSlot.value->body == id) {
            fixtureSlot.value.reset();
            fixtureSlot.generation = nextGeneration(fixtureSlot.generation);
        }
    }
    for (JointSlot& jointSlot : joints_) {
        if (jointSlot.value.has_value() &&
            (jointSlot.value->bodyA == id || jointSlot.value->bodyB == id)) {
            jointSlot.value.reset();
            jointSlot.generation = nextGeneration(jointSlot.generation);
        }
    }

    slot->value.reset();
    slot->force = {};
    slot->torque = 0.0F;
    slot->generation = nextGeneration(slot->generation);
    return true;
}

FixtureId World::createCircleFixture(
    const BodyId bodyId, const CircleFixtureDefinition& definition) {
    if (bodySlot(bodyId) == nullptr) {
        return {};
    }
    if (!std::isfinite(definition.radius) || definition.radius <= 0.0F ||
        !isFinite(definition.localCenter) ||
        !isValidMaterial(definition.friction, definition.restitution)) {
        throw std::invalid_argument("CircleFixtureDefinition contains an invalid value");
    }

    Fixture fixture {
        .body = bodyId,
        .shape = ShapeType::Circle,
        .radius = definition.radius,
        .localCenter = definition.localCenter,
        .friction = definition.friction,
        .restitution = definition.restitution,
        .sensor = definition.sensor,
        .filter = definition.filter,
    };

    for (std::uint32_t index = 0; index < fixtures_.size(); ++index) {
        FixtureSlot& slot = fixtures_[index];
        if (!slot.value.has_value()) {
            slot.value = fixture;
            return {.index = index, .generation = slot.generation};
        }
    }

    fixtures_.push_back(FixtureSlot {.value = fixture});
    return {.index = static_cast<std::uint32_t>(fixtures_.size() - 1U), .generation = 1U};
}

FixtureId World::createBoxFixture(const BodyId bodyId, const BoxFixtureDefinition& definition) {
    if (bodySlot(bodyId) == nullptr) {
        return {};
    }
    if (!isFinite(definition.halfExtents) || definition.halfExtents.x <= 0.0F ||
        definition.halfExtents.y <= 0.0F || !isFinite(definition.localCenter) ||
        !isValidMaterial(definition.friction, definition.restitution)) {
        throw std::invalid_argument("BoxFixtureDefinition contains an invalid value");
    }

    Fixture fixture {
        .body = bodyId,
        .shape = ShapeType::Box,
        .halfExtents = definition.halfExtents,
        .localCenter = definition.localCenter,
        .friction = definition.friction,
        .restitution = definition.restitution,
        .sensor = definition.sensor,
        .filter = definition.filter,
    };

    for (std::uint32_t index = 0; index < fixtures_.size(); ++index) {
        FixtureSlot& slot = fixtures_[index];
        if (!slot.value.has_value()) {
            slot.value = fixture;
            return {.index = index, .generation = slot.generation};
        }
    }

    fixtures_.push_back(FixtureSlot {.value = fixture});
    return {.index = static_cast<std::uint32_t>(fixtures_.size() - 1U), .generation = 1U};
}

FixtureId World::createPolygonFixture(
    const BodyId bodyId, const PolygonFixtureDefinition& definition) {
    if (bodySlot(bodyId) == nullptr) {
        return {};
    }
    if (!isFinite(definition.localCenter) ||
        !isValidMaterial(definition.friction, definition.restitution)) {
        throw std::invalid_argument("PolygonFixtureDefinition contains an invalid value");
    }

    std::vector<Vec2> vertices = definition.vertices;
    if (signedArea(vertices) < 0.0F) {
        std::reverse(vertices.begin(), vertices.end());
    }
    if (!isStrictlyConvexCounterClockwise(vertices)) {
        throw std::invalid_argument(
            "PolygonFixtureDefinition requires three to eight strictly convex vertices");
    }

    Fixture fixture {
        .body = bodyId,
        .shape = ShapeType::Polygon,
        .vertices = std::move(vertices),
        .localCenter = definition.localCenter,
        .friction = definition.friction,
        .restitution = definition.restitution,
        .sensor = definition.sensor,
        .filter = definition.filter,
    };

    for (std::uint32_t index = 0; index < fixtures_.size(); ++index) {
        FixtureSlot& slot = fixtures_[index];
        if (!slot.value.has_value()) {
            slot.value = std::move(fixture);
            return {.index = index, .generation = slot.generation};
        }
    }

    fixtures_.push_back(FixtureSlot {.value = std::move(fixture)});
    return {.index = static_cast<std::uint32_t>(fixtures_.size() - 1U), .generation = 1U};
}

bool World::destroyFixture(const FixtureId id) noexcept {
    FixtureSlot* const slot = fixtureSlot(id);
    if (slot == nullptr) {
        return false;
    }

    slot->value.reset();
    slot->generation = nextGeneration(slot->generation);
    return true;
}

JointId World::createDistanceJoint(const DistanceJointDefinition& definition) {
    if (bodySlot(definition.bodyA) == nullptr || bodySlot(definition.bodyB) == nullptr) {
        return {};
    }
    if (definition.bodyA == definition.bodyB || !isFinite(definition.localAnchorA) ||
        !isFinite(definition.localAnchorB) || !std::isfinite(definition.length) ||
        !std::isfinite(definition.stiffness) || definition.length < 0.0F ||
        definition.stiffness <= 0.0F || definition.stiffness > 1.0F) {
        throw std::invalid_argument("DistanceJointDefinition contains an invalid value");
    }

    DistanceJoint joint {
        .bodyA = definition.bodyA,
        .bodyB = definition.bodyB,
        .localAnchorA = definition.localAnchorA,
        .localAnchorB = definition.localAnchorB,
        .length = definition.length,
        .stiffness = definition.stiffness,
    };
    for (std::uint32_t index = 0; index < joints_.size(); ++index) {
        JointSlot& slot = joints_[index];
        if (!slot.value.has_value()) {
            slot.value = joint;
            static_cast<void>(wake(definition.bodyA));
            static_cast<void>(wake(definition.bodyB));
            return {.index = index, .generation = slot.generation};
        }
    }

    joints_.push_back(JointSlot {.value = joint});
    static_cast<void>(wake(definition.bodyA));
    static_cast<void>(wake(definition.bodyB));
    return {.index = static_cast<std::uint32_t>(joints_.size() - 1U), .generation = 1U};
}

bool World::destroyJoint(const JointId id) noexcept {
    JointSlot* const slot = jointSlot(id);
    if (slot == nullptr) {
        return false;
    }
    slot->value.reset();
    slot->generation = nextGeneration(slot->generation);
    return true;
}

BodyState* World::body(const BodyId id) noexcept {
    BodySlot* const slot = bodySlot(id);
    return slot != nullptr ? &*slot->value : nullptr;
}

const BodyState* World::body(const BodyId id) const noexcept {
    const BodySlot* const slot = bodySlot(id);
    return slot != nullptr ? &*slot->value : nullptr;
}

bool World::setLinearVelocity(const BodyId id, const Vec2 velocity) noexcept {
    BodyState* const state = body(id);
    if (state == nullptr || !isFinite(velocity)) {
        return false;
    }

    state->linearVelocity = velocity;
    state->asleep = false;
    state->sleepDuration = 0.0F;
    return true;
}

bool World::setAngularVelocity(const BodyId id, const float angularVelocity) noexcept {
    BodyState* const state = body(id);
    if (state == nullptr || !std::isfinite(angularVelocity)) {
        return false;
    }
    state->angularVelocity = angularVelocity;
    state->asleep = false;
    state->sleepDuration = 0.0F;
    return true;
}

bool World::applyForce(const BodyId id, const Vec2 force) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !isFinite(force)) {
        return false;
    }

    slot->force += force;
    slot->value->asleep = false;
    slot->value->sleepDuration = 0.0F;
    return true;
}

bool World::applyForceAtPoint(const BodyId id, const Vec2 force, const Vec2 worldPoint) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !isFinite(force) ||
        !isFinite(worldPoint)) {
        return false;
    }
    slot->force += force;
    slot->torque += math::cross(worldPoint - slot->value->position, force);
    slot->value->asleep = false;
    slot->value->sleepDuration = 0.0F;
    return true;
}

bool World::applyTorque(const BodyId id, const float torque) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !std::isfinite(torque)) {
        return false;
    }
    slot->torque += torque;
    slot->value->asleep = false;
    slot->value->sleepDuration = 0.0F;
    return true;
}

bool World::wake(const BodyId id) noexcept {
    BodyState* const state = body(id);
    if (state == nullptr) {
        return false;
    }
    state->asleep = false;
    state->sleepDuration = 0.0F;
    return true;
}

std::vector<FixtureId> World::queryAabb(const Aabb& bounds) const {
    if (!math::isValid(bounds)) {
        throw std::invalid_argument("World::queryAabb requires valid bounds");
    }

    std::vector<FixtureId> results;
    for (std::uint32_t index = 0; index < fixtures_.size(); ++index) {
        const FixtureSlot& slot = fixtures_[index];
        if (slot.value.has_value() && overlaps(bounds, fixtureAabb(*slot.value))) {
            results.push_back({.index = index, .generation = slot.generation});
        }
    }
    return results;
}

void World::step(const float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0F) {
        throw std::invalid_argument("World::step requires a positive finite delta time");
    }

    events_.clear();
    stats_ = {};

    for (const JointSlot& slot : joints_) {
        if (!slot.value.has_value()) {
            continue;
        }
        ++stats_.activeJoints;
        BodyState* const firstBody = body(slot.value->bodyA);
        BodyState* const secondBody = body(slot.value->bodyB);
        if (firstBody == nullptr || secondBody == nullptr) {
            continue;
        }
        if (firstBody->type == BodyType::Dynamic && secondBody->type == BodyType::Dynamic &&
            firstBody->asleep != secondBody->asleep) {
            firstBody->asleep = false;
            firstBody->sleepDuration = 0.0F;
            secondBody->asleep = false;
            secondBody->sleepDuration = 0.0F;
        }
    }

    for (BodySlot& slot : bodies_) {
        if (!slot.value.has_value()) {
            continue;
        }

        ++stats_.activeBodies;
        BodyState& state = *slot.value;
        state.previousPosition = state.position;
        state.previousAngle = state.angle;
        if (state.type == BodyType::Dynamic && state.asleep) {
            ++stats_.sleepingBodies;
            slot.force = {};
            slot.torque = 0.0F;
            continue;
        }
        if (state.type == BodyType::Dynamic) {
            const float invMass = inverseMass(state);
            state.linearVelocity +=
                (settings_.gravity * state.gravityScale + slot.force * invMass) * deltaTime;
            state.linearVelocity *= 1.0F / (1.0F + state.linearDamping * deltaTime);
            state.angularVelocity += slot.torque * inverseInertia(state) * deltaTime;
            state.angularVelocity *= 1.0F / (1.0F + state.angularDamping * deltaTime);
        }
        if (movesWithVelocity(state.type)) {
            state.position += state.linearVelocity * deltaTime;
            state.angle += state.angularVelocity * deltaTime;
        }
        slot.force = {};
        slot.torque = 0.0F;
    }

    for (std::uint32_t bulletBodyIndex = 0; bulletBodyIndex < bodies_.size(); ++bulletBodyIndex) {
        BodySlot& bulletSlot = bodies_[bulletBodyIndex];
        if (!bulletSlot.value.has_value()) {
            continue;
        }
        BodyState& bulletBody = *bulletSlot.value;
        if (bulletBody.type != BodyType::Dynamic || !bulletBody.bullet || bulletBody.asleep) {
            continue;
        }

        float earliestFraction = 1.0F;
        Vec2 impactNormal {};
        bool hitDetected = false;
        for (std::uint32_t bulletFixtureIndex = 0; bulletFixtureIndex < fixtures_.size();
             ++bulletFixtureIndex) {
            const FixtureSlot& bulletFixtureSlot = fixtures_[bulletFixtureIndex];
            if (!bulletFixtureSlot.value.has_value() ||
                bulletFixtureSlot.value->body.index != bulletBodyIndex ||
                bulletFixtureSlot.value->shape != ShapeType::Circle) {
                continue;
            }
            const Fixture& bulletFixture = *bulletFixtureSlot.value;
            const Vec2 start = bulletBody.previousPosition +
                rotate(bulletFixture.localCenter, bulletBody.previousAngle);
            const Vec2 end = bulletBody.position + rotate(bulletFixture.localCenter, bulletBody.angle);
            const Vec2 translation = end - start;
            const Aabb sweptBounds {
                .min = {
                    std::min(start.x, end.x) - bulletFixture.radius,
                    std::min(start.y, end.y) - bulletFixture.radius,
                },
                .max = {
                    std::max(start.x, end.x) + bulletFixture.radius,
                    std::max(start.y, end.y) + bulletFixture.radius,
                },
            };
            for (const FixtureSlot& targetSlot : fixtures_) {
                if (!targetSlot.value.has_value()) {
                    continue;
                }
                const Fixture& targetFixture = *targetSlot.value;
                if (targetFixture.body == bulletFixture.body || targetFixture.sensor ||
                    !canCollide(bulletFixture.filter, targetFixture.filter)) {
                    continue;
                }
                const BodyState* const targetBody = body(targetFixture.body);
                if (targetBody == nullptr ||
                    (targetBody->type != BodyType::Static && targetBody->type != BodyType::Kinematic)) {
                    continue;
                }
                const Aabb targetBounds = fixtureAabb(targetFixture);
                if (!overlaps(sweptBounds, targetBounds)) {
                    continue;
                }

                ++stats_.continuousCollisionTests;
                SweepHit candidate {};
                const bool hit = targetFixture.shape == ShapeType::Circle
                    ? sweepCircleAgainstCircle(
                          start,
                          translation,
                          bulletFixture.radius,
                          targetBody->position +
                              rotate(targetFixture.localCenter, targetBody->angle),
                          targetFixture.radius,
                          candidate)
                    : sweepCircleAgainstAabb(
                          start, translation, bulletFixture.radius, targetBounds, candidate);
                if (hit && candidate.fraction < earliestFraction) {
                    earliestFraction = candidate.fraction;
                    impactNormal = candidate.normal;
                    hitDetected = true;
                }
            }
        }

        if (hitDetected) {
            constexpr float impactOverlap = 1.0e-4F;
            const float resolvedFraction = std::min(earliestFraction + impactOverlap, 1.0F);
            bulletBody.position = bulletBody.previousPosition +
                (bulletBody.position - bulletBody.previousPosition) * resolvedFraction;
            bulletBody.angle = bulletBody.previousAngle +
                (bulletBody.angle - bulletBody.previousAngle) * resolvedFraction;
            const float closingSpeed = dot(bulletBody.linearVelocity, impactNormal);
            if (closingSpeed < 0.0F) {
                bulletBody.linearVelocity -= impactNormal * closingSpeed;
            }
            ++stats_.continuousCollisionHits;
        }
    }

    struct BroadPhaseEntry {
        FixtureId id {};
        const Fixture* fixture {nullptr};
        Aabb bounds {};
    };
    std::vector<BroadPhaseEntry> broadPhaseEntries;
    broadPhaseEntries.reserve(fixtures_.size());
    for (std::uint32_t index = 0; index < fixtures_.size(); ++index) {
        const FixtureSlot& slot = fixtures_[index];
        if (!slot.value.has_value() || body(slot.value->body) == nullptr) {
            continue;
        }
        ++stats_.activeFixtures;
        broadPhaseEntries.push_back({
            .id = {.index = index, .generation = slot.generation},
            .fixture = &*slot.value,
            .bounds = fixtureAabb(*slot.value),
        });
    }

    std::stable_sort(
        broadPhaseEntries.begin(), broadPhaseEntries.end(), [](const BroadPhaseEntry& first,
                                                               const BroadPhaseEntry& second) {
            return first.bounds.min.x == second.bounds.min.x
                ? fixtureKey(first.id) < fixtureKey(second.id)
                : first.bounds.min.x < second.bounds.min.x;
        });

    contacts_.clear();
    std::set<std::pair<std::uint64_t, std::uint64_t>> currentContacts;

    for (std::size_t firstIndex = 0; firstIndex < broadPhaseEntries.size(); ++firstIndex) {
        const BroadPhaseEntry& firstEntry = broadPhaseEntries[firstIndex];
        const Fixture& first = *firstEntry.fixture;
        const BodyState* const firstBody = body(first.body);
        if (firstBody == nullptr) {
            continue;
        }

        for (std::size_t secondIndex = firstIndex + 1U;
             secondIndex < broadPhaseEntries.size(); ++secondIndex) {
            const BroadPhaseEntry& secondEntry = broadPhaseEntries[secondIndex];
            if (secondEntry.bounds.min.x > firstEntry.bounds.max.x) {
                break;
            }
            ++stats_.broadPhasePairTests;
            const Fixture& second = *secondEntry.fixture;
            const BodyState* const secondBody = body(second.body);
            if (secondBody == nullptr || first.body == second.body ||
                (firstBody->type != BodyType::Dynamic && secondBody->type != BodyType::Dynamic) ||
                !overlaps(firstEntry.bounds, secondEntry.bounds)) {
                continue;
            }

            ++stats_.broadPhaseCandidatePairs;
            if (!canCollide(first.filter, second.filter)) {
                continue;
            }
            ++stats_.narrowPhaseTests;

            const Vec2 firstCenter = firstBody->position + rotate(first.localCenter, firstBody->angle);
            const Vec2 secondCenter = secondBody->position + rotate(second.localCenter, secondBody->angle);
            Vec2 normal {};
            Vec2 contactPoint {};
            float penetration = 0.0F;
            bool collided = false;

            if (first.shape == ShapeType::Circle && second.shape == ShapeType::Circle) {
                const Vec2 delta = secondCenter - firstCenter;
                const float combinedRadius = first.radius + second.radius;
                const float distanceSquared = lengthSquared(delta);
                if (distanceSquared < combinedRadius * combinedRadius) {
                    const float distance = std::sqrt(distanceSquared);
                    normal = distance > 1.0e-6F ? delta / distance : Vec2 {1.0F, 0.0F};
                    penetration = combinedRadius - distance;
                    contactPoint = firstCenter + normal * (first.radius - penetration * 0.5F);
                    collided = true;
                }
            } else {
                const auto worldVertices = [](const Fixture& fixture, const BodyState& bodyState) {
                    std::vector<Vec2> vertices;
                    if (fixture.shape == ShapeType::Box) {
                        vertices = {
                            {-fixture.halfExtents.x, -fixture.halfExtents.y},
                            {fixture.halfExtents.x, -fixture.halfExtents.y},
                            {fixture.halfExtents.x, fixture.halfExtents.y},
                            {-fixture.halfExtents.x, fixture.halfExtents.y},
                        };
                    } else {
                        vertices = fixture.vertices;
                    }
                    for (Vec2& vertex : vertices) {
                        vertex = bodyState.position + rotate(fixture.localCenter + vertex, bodyState.angle);
                    }
                    return vertices;
                };
                const auto polygonCenter = [](const std::vector<Vec2>& vertices) {
                    Vec2 center {};
                    for (const Vec2 vertex : vertices) {
                        center += vertex;
                    }
                    return center / static_cast<float>(vertices.size());
                };
                const auto project = [](const std::vector<Vec2>& vertices, const Vec2 axis,
                                        float& minimum, float& maximum) {
                    minimum = maximum = dot(vertices.front(), axis);
                    for (const Vec2 vertex : vertices) {
                        const float value = dot(vertex, axis);
                        minimum = std::min(minimum, value);
                        maximum = std::max(maximum, value);
                    }
                };
                const bool firstIsCircle = first.shape == ShapeType::Circle;
                const bool secondIsCircle = second.shape == ShapeType::Circle;
                const Fixture& polygonFixture = firstIsCircle ? second : first;
                const BodyState& polygonBody = firstIsCircle ? *secondBody : *firstBody;
                const std::vector<Vec2> firstPolygon = firstIsCircle ? std::vector<Vec2> {} : worldVertices(first, *firstBody);
                const std::vector<Vec2> secondPolygon = secondIsCircle ? std::vector<Vec2> {} : worldVertices(second, *secondBody);

                if (!firstIsCircle && !secondIsCircle) {
                    const Vec2 firstPolygonCenter = polygonCenter(firstPolygon);
                    const Vec2 secondPolygonCenter = polygonCenter(secondPolygon);
                    penetration = std::numeric_limits<float>::max();
                    collided = true;
                    for (const auto* polygon : {&firstPolygon, &secondPolygon}) {
                        for (std::size_t index = 0; index < polygon->size(); ++index) {
                            const Vec2 edge = (*polygon)[(index + 1U) % polygon->size()] - (*polygon)[index];
                            const Vec2 axis = normalized({-edge.y, edge.x});
                            float firstMinimum = 0.0F;
                            float firstMaximum = 0.0F;
                            float secondMinimum = 0.0F;
                            float secondMaximum = 0.0F;
                            project(firstPolygon, axis, firstMinimum, firstMaximum);
                            project(secondPolygon, axis, secondMinimum, secondMaximum);
                            const float overlap = std::min(firstMaximum, secondMaximum) -
                                std::max(firstMinimum, secondMinimum);
                            if (overlap <= 0.0F) {
                                collided = false;
                                break;
                            }
                            if (overlap < penetration) {
                                penetration = overlap;
                                normal = dot(secondPolygonCenter - firstPolygonCenter, axis) >= 0.0F ? axis : -axis;
                            }
                        }
                        if (!collided) {
                            break;
                        }
                    }
                    contactPoint = (firstPolygonCenter + secondPolygonCenter) * 0.5F;
                } else {
                    const Fixture& circle = firstIsCircle ? first : second;
                    const Vec2 circleCenter = firstIsCircle ? firstCenter : secondCenter;
                    const std::vector<Vec2> polygon = worldVertices(polygonFixture, polygonBody);
                    const Vec2 center = polygonCenter(polygon);
                    penetration = std::numeric_limits<float>::max();
                    collided = true;
                    std::vector<Vec2> axes;
                    axes.reserve(polygon.size() + 1U);
                    std::size_t closestVertex = 0U;
                    float closestDistance = lengthSquared(polygon.front() - circleCenter);
                    for (std::size_t index = 0; index < polygon.size(); ++index) {
                        const Vec2 edge = polygon[(index + 1U) % polygon.size()] - polygon[index];
                        axes.push_back(normalized({-edge.y, edge.x}));
                        const float distance = lengthSquared(polygon[index] - circleCenter);
                        if (distance < closestDistance) {
                            closestDistance = distance;
                            closestVertex = index;
                        }
                    }
                    if (closestDistance > 1.0e-12F) {
                        axes.push_back(normalized(polygon[closestVertex] - circleCenter));
                    }
                    for (const Vec2 axis : axes) {
                        float polygonMinimum = 0.0F;
                        float polygonMaximum = 0.0F;
                        project(polygon, axis, polygonMinimum, polygonMaximum);
                        const float circleProjection = dot(circleCenter, axis);
                        const float overlap = std::min(polygonMaximum, circleProjection + circle.radius) -
                            std::max(polygonMinimum, circleProjection - circle.radius);
                        if (overlap <= 0.0F) {
                            collided = false;
                            break;
                        }
                        if (overlap < penetration) {
                            penetration = overlap;
                            normal = dot(center - circleCenter, axis) >= 0.0F ? axis : -axis;
                        }
                    }
                    contactPoint = circleCenter + normal * circle.radius;
                    if (collided && !firstIsCircle) {
                        normal = -normal;
                    }
                }
            }

            if (!collided) {
                continue;
            }

            const std::uint64_t firstKey = fixtureKey(firstEntry.id);
            const std::uint64_t secondKey = fixtureKey(secondEntry.id);
            const auto pair = std::pair {std::min(firstKey, secondKey), std::max(firstKey, secondKey)};
            currentContacts.insert(pair);
            const ContactEventType eventType = activeContacts_.contains(pair)
                ? ContactEventType::Stay
                : ContactEventType::Begin;
            const bool sensor = first.sensor || second.sensor;
            contacts_.push_back({
                .fixtureA = firstEntry.id,
                .fixtureB = secondEntry.id,
                .normal = normal,
                .point = contactPoint,
                .penetration = penetration,
                .sensor = sensor,
            });
            events_.push_back({
                .type = eventType,
                .fixtureA = firstEntry.id,
                .fixtureB = secondEntry.id,
                .normal = normal,
                .penetration = penetration,
                .sensor = sensor,
            });
        }
    }

    for (ContactConstraint& contact : contacts_) {
        if (contact.sensor) {
            continue;
        }

        const FixtureSlot* const firstFixtureSlot = fixtureSlot(contact.fixtureA);
        const FixtureSlot* const secondFixtureSlot = fixtureSlot(contact.fixtureB);
        if (firstFixtureSlot == nullptr || secondFixtureSlot == nullptr) {
            continue;
        }
        BodyState* const firstBody = body(firstFixtureSlot->value->body);
        BodyState* const secondBody = body(secondFixtureSlot->value->body);
        if (firstBody == nullptr || secondBody == nullptr) {
            continue;
        }

        const float firstInverseMass = inverseMass(*firstBody);
        const float secondInverseMass = inverseMass(*secondBody);
        const float inverseMassSum = firstInverseMass + secondInverseMass;
        if (inverseMassSum <= 0.0F) {
            continue;
        }

        const float correctionMagnitude =
            (std::max(contact.penetration - settings_.penetrationSlop, 0.0F) /
             inverseMassSum) * settings_.positionCorrection;
        const Vec2 correction = contact.normal * correctionMagnitude;
        firstBody->position -= correction * firstInverseMass;
        secondBody->position += correction * secondInverseMass;
    }

    for (JointSlot& slot : joints_) {
        if (!slot.value.has_value()) {
            continue;
        }
        const DistanceJoint& joint = *slot.value;
        BodyState* const firstBody = body(joint.bodyA);
        BodyState* const secondBody = body(joint.bodyB);
        if (firstBody == nullptr || secondBody == nullptr) {
            continue;
        }

        const Vec2 firstArm = rotate(joint.localAnchorA, firstBody->angle);
        const Vec2 secondArm = rotate(joint.localAnchorB, secondBody->angle);
        const Vec2 delta = (secondBody->position + secondArm) -
            (firstBody->position + firstArm);
        const float distance = length(delta);
        const Vec2 normal = distance > 1.0e-6F ? delta / distance : Vec2 {1.0F, 0.0F};
        const float firstNormalArm = math::cross(firstArm, normal);
        const float secondNormalArm = math::cross(secondArm, normal);
        const float effectiveMass = inverseMass(*firstBody) + inverseMass(*secondBody) +
            inverseInertia(*firstBody) * firstNormalArm * firstNormalArm +
            inverseInertia(*secondBody) * secondNormalArm * secondNormalArm;
        if (effectiveMass <= 1.0e-6F) {
            continue;
        }

        const float positionLambda =
            (distance - joint.length) * joint.stiffness / effectiveMass;
        firstBody->position += normal * (positionLambda * inverseMass(*firstBody));
        secondBody->position -= normal * (positionLambda * inverseMass(*secondBody));
        firstBody->angle += positionLambda * inverseInertia(*firstBody) * firstNormalArm;
        secondBody->angle -= positionLambda * inverseInertia(*secondBody) * secondNormalArm;
    }

    for (std::uint32_t iteration = 0; iteration < settings_.velocityIterations; ++iteration) {
        for (ContactConstraint& contact : contacts_) {
            if (contact.sensor) {
                continue;
            }

            const FixtureSlot* const firstFixtureSlot = fixtureSlot(contact.fixtureA);
            const FixtureSlot* const secondFixtureSlot = fixtureSlot(contact.fixtureB);
            if (firstFixtureSlot == nullptr || secondFixtureSlot == nullptr) {
                continue;
            }
            BodyState* const firstBody = body(firstFixtureSlot->value->body);
            BodyState* const secondBody = body(secondFixtureSlot->value->body);
            if (firstBody == nullptr || secondBody == nullptr) {
                continue;
            }

            const float firstInverseMass = inverseMass(*firstBody);
            const float secondInverseMass = inverseMass(*secondBody);
            const float firstInverseInertia = inverseInertia(*firstBody);
            const float secondInverseInertia = inverseInertia(*secondBody);
            const Vec2 firstArm = contact.point - firstBody->position;
            const Vec2 secondArm = contact.point - secondBody->position;
            const float firstNormalArm = math::cross(firstArm, contact.normal);
            const float secondNormalArm = math::cross(secondArm, contact.normal);
            const float normalMass = firstInverseMass + secondInverseMass +
                firstInverseInertia * firstNormalArm * firstNormalArm +
                secondInverseInertia * secondNormalArm * secondNormalArm;
            if (normalMass <= 0.0F) {
                continue;
            }

            const Vec2 firstPointVelocity = firstBody->linearVelocity +
                math::cross(firstBody->angularVelocity, firstArm);
            const Vec2 secondPointVelocity = secondBody->linearVelocity +
                math::cross(secondBody->angularVelocity, secondArm);
            const Vec2 relativeVelocity = secondPointVelocity - firstPointVelocity;
            const float velocityAlongNormal = dot(relativeVelocity, contact.normal);
            const float restitution = mixRestitution(
                firstFixtureSlot->value->restitution, secondFixtureSlot->value->restitution);
            const float normalImpulseChange =
                -((1.0F + restitution) * velocityAlongNormal) / normalMass;
            const float oldNormalImpulse = contact.accumulatedNormalImpulse;
            contact.accumulatedNormalImpulse =
                std::max(oldNormalImpulse + normalImpulseChange, 0.0F);
            const Vec2 normalImpulse =
                contact.normal * (contact.accumulatedNormalImpulse - oldNormalImpulse);
            firstBody->linearVelocity -= normalImpulse * firstInverseMass;
            secondBody->linearVelocity += normalImpulse * secondInverseMass;
            firstBody->angularVelocity -= firstInverseInertia * math::cross(firstArm, normalImpulse);
            secondBody->angularVelocity += secondInverseInertia * math::cross(secondArm, normalImpulse);

            const Vec2 postNormalRelativeVelocity =
                (secondBody->linearVelocity + math::cross(secondBody->angularVelocity, secondArm)) -
                (firstBody->linearVelocity + math::cross(firstBody->angularVelocity, firstArm));
            const Vec2 tangentVector = postNormalRelativeVelocity -
                contact.normal * dot(postNormalRelativeVelocity, contact.normal);
            if (lengthSquared(tangentVector) <= 1.0e-12F) {
                continue;
            }

            const Vec2 tangent = normalized(tangentVector);
            const float firstTangentArm = math::cross(firstArm, tangent);
            const float secondTangentArm = math::cross(secondArm, tangent);
            const float tangentMass = firstInverseMass + secondInverseMass +
                firstInverseInertia * firstTangentArm * firstTangentArm +
                secondInverseInertia * secondTangentArm * secondTangentArm;
            if (tangentMass <= 0.0F) {
                continue;
            }
            const float tangentImpulseChange =
                -dot(postNormalRelativeVelocity, tangent) / tangentMass;
            const float maximumFrictionImpulse = mixFriction(
                firstFixtureSlot->value->friction, secondFixtureSlot->value->friction) *
                contact.accumulatedNormalImpulse;
            const float oldTangentImpulse = contact.accumulatedTangentImpulse;
            contact.accumulatedTangentImpulse = std::clamp(
                oldTangentImpulse + tangentImpulseChange,
                -maximumFrictionImpulse,
                maximumFrictionImpulse);
            const Vec2 tangentImpulse =
                tangent * (contact.accumulatedTangentImpulse - oldTangentImpulse);
            firstBody->linearVelocity -= tangentImpulse * firstInverseMass;
            secondBody->linearVelocity += tangentImpulse * secondInverseMass;
            firstBody->angularVelocity -= firstInverseInertia * math::cross(firstArm, tangentImpulse);
            secondBody->angularVelocity += secondInverseInertia * math::cross(secondArm, tangentImpulse);
        }

        for (JointSlot& slot : joints_) {
            if (!slot.value.has_value()) {
                continue;
            }
            const DistanceJoint& joint = *slot.value;
            BodyState* const firstBody = body(joint.bodyA);
            BodyState* const secondBody = body(joint.bodyB);
            if (firstBody == nullptr || secondBody == nullptr) {
                continue;
            }

            const Vec2 firstArm = rotate(joint.localAnchorA, firstBody->angle);
            const Vec2 secondArm = rotate(joint.localAnchorB, secondBody->angle);
            const Vec2 delta = (secondBody->position + secondArm) -
                (firstBody->position + firstArm);
            const float distance = length(delta);
            const Vec2 normal = distance > 1.0e-6F ? delta / distance : Vec2 {1.0F, 0.0F};
            const float firstNormalArm = math::cross(firstArm, normal);
            const float secondNormalArm = math::cross(secondArm, normal);
            const float firstInverseMass = inverseMass(*firstBody);
            const float secondInverseMass = inverseMass(*secondBody);
            const float firstInverseInertia = inverseInertia(*firstBody);
            const float secondInverseInertia = inverseInertia(*secondBody);
            const float effectiveMass = firstInverseMass + secondInverseMass +
                firstInverseInertia * firstNormalArm * firstNormalArm +
                secondInverseInertia * secondNormalArm * secondNormalArm;
            if (effectiveMass <= 1.0e-6F) {
                continue;
            }

            const Vec2 firstPointVelocity = firstBody->linearVelocity +
                math::cross(firstBody->angularVelocity, firstArm);
            const Vec2 secondPointVelocity = secondBody->linearVelocity +
                math::cross(secondBody->angularVelocity, secondArm);
            const float impulseMagnitude =
                -dot(secondPointVelocity - firstPointVelocity, normal) / effectiveMass;
            const Vec2 impulse = normal * impulseMagnitude;
            firstBody->linearVelocity -= impulse * firstInverseMass;
            secondBody->linearVelocity += impulse * secondInverseMass;
            firstBody->angularVelocity -= firstInverseInertia * math::cross(firstArm, impulse);
            secondBody->angularVelocity += secondInverseInertia * math::cross(secondArm, impulse);
        }
    }

    for (const auto& pair : activeContacts_) {
        if (!currentContacts.contains(pair)) {
            bool sensor = false;
            const FixtureSlot* const firstFixtureSlot = fixtureSlot(fixtureIdFromKey(pair.first));
            const FixtureSlot* const secondFixtureSlot = fixtureSlot(fixtureIdFromKey(pair.second));
            if (firstFixtureSlot != nullptr && secondFixtureSlot != nullptr) {
                sensor = firstFixtureSlot->value->sensor || secondFixtureSlot->value->sensor;
            }
            events_.push_back({
                .type = ContactEventType::End,
                .fixtureA = fixtureIdFromKey(pair.first),
                .fixtureB = fixtureIdFromKey(pair.second),
                .sensor = sensor,
            });
        }
    }
    activeContacts_ = std::move(currentContacts);
    stats_.activeContacts = activeContacts_.size();

    for (BodySlot& slot : bodies_) {
        if (!slot.value.has_value() || slot.value->type != BodyType::Dynamic) {
            continue;
        }
        BodyState& state = *slot.value;
        if (lengthSquared(state.linearVelocity) <=
                settings_.sleepLinearThreshold * settings_.sleepLinearThreshold &&
            std::abs(state.angularVelocity) <= settings_.sleepAngularThreshold) {
            state.sleepDuration += deltaTime;
            if (state.sleepDuration >= settings_.sleepDelay) {
                if (!state.asleep) {
                    ++stats_.sleepingBodies;
                }
                state.asleep = true;
                state.linearVelocity = {};
                state.angularVelocity = 0.0F;
            }
        } else {
            state.asleep = false;
            state.sleepDuration = 0.0F;
        }
    }
}

std::span<const ContactEvent> World::contactEvents() const noexcept {
    return events_;
}

void World::clearContactEvents() noexcept {
    events_.clear();
}

const WorldSettings& World::settings() const noexcept {
    return settings_;
}

const WorldStats& World::stats() const noexcept {
    return stats_;
}

World::BodySlot* World::bodySlot(const BodyId id) noexcept {
    if (!id || id.index >= bodies_.size()) {
        return nullptr;
    }
    BodySlot& slot = bodies_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

const World::BodySlot* World::bodySlot(const BodyId id) const noexcept {
    if (!id || id.index >= bodies_.size()) {
        return nullptr;
    }
    const BodySlot& slot = bodies_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

World::FixtureSlot* World::fixtureSlot(const FixtureId id) noexcept {
    if (!id || id.index >= fixtures_.size()) {
        return nullptr;
    }
    FixtureSlot& slot = fixtures_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

const World::FixtureSlot* World::fixtureSlot(const FixtureId id) const noexcept {
    if (!id || id.index >= fixtures_.size()) {
        return nullptr;
    }
    const FixtureSlot& slot = fixtures_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

World::JointSlot* World::jointSlot(const JointId id) noexcept {
    if (!id || id.index >= joints_.size()) {
        return nullptr;
    }
    JointSlot& slot = joints_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

const World::JointSlot* World::jointSlot(const JointId id) const noexcept {
    if (!id || id.index >= joints_.size()) {
        return nullptr;
    }
    const JointSlot& slot = joints_[id.index];
    return slot.generation == id.generation && slot.value.has_value() ? &slot : nullptr;
}

Aabb World::fixtureAabb(const Fixture& fixture) const noexcept {
    const BodyState* const state = body(fixture.body);
    if (state == nullptr) {
        return {};
    }

    const Vec2 center = state->position + rotate(fixture.localCenter, state->angle);
    if (fixture.shape == ShapeType::Circle) {
        const Vec2 extent {fixture.radius, fixture.radius};
        return {.min = center - extent, .max = center + extent};
    }
    if (fixture.shape == ShapeType::Polygon) {
        Vec2 minimum {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vec2 maximum {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};
        for (const Vec2 vertex : fixture.vertices) {
            const Vec2 worldVertex = state->position + rotate(fixture.localCenter + vertex, state->angle);
            minimum.x = std::min(minimum.x, worldVertex.x);
            minimum.y = std::min(minimum.y, worldVertex.y);
            maximum.x = std::max(maximum.x, worldVertex.x);
            maximum.y = std::max(maximum.y, worldVertex.y);
        }
        return {.min = minimum, .max = maximum};
    }
    const Vec2 xAxis = rotate({1.0F, 0.0F}, state->angle);
    const Vec2 yAxis = rotate({0.0F, 1.0F}, state->angle);
    const Vec2 extent {
        .x = std::abs(xAxis.x) * fixture.halfExtents.x + std::abs(yAxis.x) * fixture.halfExtents.y,
        .y = std::abs(xAxis.y) * fixture.halfExtents.x + std::abs(yAxis.y) * fixture.halfExtents.y,
    };
    return {.min = center - extent, .max = center + extent};
}

} // namespace render2d::physics
