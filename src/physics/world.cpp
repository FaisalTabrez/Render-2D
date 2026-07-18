#include "render2d/physics/world.hpp"

#include <algorithm>
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

} // namespace

World::World(WorldSettings settings) : settings_(settings) {
    if (!isFinite(settings_.gravity) || settings_.velocityIterations == 0U ||
        settings_.penetrationSlop < 0.0F || settings_.positionCorrection < 0.0F ||
        settings_.positionCorrection > 1.0F) {
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

bool World::destroyFixture(const FixtureId id) noexcept {
    FixtureSlot* const slot = fixtureSlot(id);
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
    return true;
}

bool World::setAngularVelocity(const BodyId id, const float angularVelocity) noexcept {
    BodyState* const state = body(id);
    if (state == nullptr || !std::isfinite(angularVelocity)) {
        return false;
    }
    state->angularVelocity = angularVelocity;
    return true;
}

bool World::applyForce(const BodyId id, const Vec2 force) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !isFinite(force)) {
        return false;
    }

    slot->force += force;
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
    return true;
}

bool World::applyTorque(const BodyId id, const float torque) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !std::isfinite(torque)) {
        return false;
    }
    slot->torque += torque;
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

    for (BodySlot& slot : bodies_) {
        if (!slot.value.has_value()) {
            continue;
        }

        ++stats_.activeBodies;
        BodyState& state = *slot.value;
        state.previousPosition = state.position;
        state.previousAngle = state.angle;
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

            const Vec2 firstCenter = firstBody->position + first.localCenter;
            const Vec2 secondCenter = secondBody->position + second.localCenter;
            Vec2 normal {};
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
                    collided = true;
                }
            } else if (first.shape == ShapeType::Box && second.shape == ShapeType::Box) {
                const Vec2 delta = secondCenter - firstCenter;
                const float overlapX = first.halfExtents.x + second.halfExtents.x - std::abs(delta.x);
                const float overlapY = first.halfExtents.y + second.halfExtents.y - std::abs(delta.y);
                if (overlapX > 0.0F && overlapY > 0.0F) {
                    if (overlapX < overlapY) {
                        normal = {delta.x >= 0.0F ? 1.0F : -1.0F, 0.0F};
                        penetration = overlapX;
                    } else {
                        normal = {0.0F, delta.y >= 0.0F ? 1.0F : -1.0F};
                        penetration = overlapY;
                    }
                    collided = true;
                }
            } else {
                const bool firstIsCircle = first.shape == ShapeType::Circle;
                const Fixture& circle = firstIsCircle ? first : second;
                const Vec2 circleCenter = firstIsCircle ? firstCenter : secondCenter;
                const Fixture& box = firstIsCircle ? second : first;
                const Vec2 boxCenter = firstIsCircle ? secondCenter : firstCenter;
                const Aabb boxBounds {
                    .min = boxCenter - box.halfExtents,
                    .max = boxCenter + box.halfExtents,
                };
                const Vec2 closestPoint = math::clamp(circleCenter, boxBounds);
                const Vec2 circleToBox = closestPoint - circleCenter;
                const float distanceSquared = lengthSquared(circleToBox);

                if (distanceSquared > 1.0e-12F) {
                    const float distance = std::sqrt(distanceSquared);
                    if (distance < circle.radius) {
                        normal = circleToBox / distance;
                        penetration = circle.radius - distance;
                        collided = true;
                    }
                } else if (math::contains(boxBounds, circleCenter)) {
                    const float left = circleCenter.x - boxBounds.min.x;
                    const float right = boxBounds.max.x - circleCenter.x;
                    const float bottom = circleCenter.y - boxBounds.min.y;
                    const float top = boxBounds.max.y - circleCenter.y;
                    const float nearestFace = std::min({left, right, bottom, top});
                    Vec2 outward {};
                    if (nearestFace == left) {
                        outward = {-1.0F, 0.0F};
                    } else if (nearestFace == right) {
                        outward = {1.0F, 0.0F};
                    } else if (nearestFace == bottom) {
                        outward = {0.0F, -1.0F};
                    } else {
                        outward = {0.0F, 1.0F};
                    }
                    normal = -outward;
                    penetration = circle.radius + nearestFace;
                    collided = true;
                }

                if (collided && !firstIsCircle) {
                    normal = -normal;
                }
            }

            if (!collided) {
                continue;
            }

            const auto pair = std::pair {fixtureKey(firstEntry.id), fixtureKey(secondEntry.id)};
            currentContacts.insert(pair);
            const ContactEventType eventType = activeContacts_.contains(pair)
                ? ContactEventType::Stay
                : ContactEventType::Begin;
            const bool sensor = first.sensor || second.sensor;
            contacts_.push_back({
                .fixtureA = firstEntry.id,
                .fixtureB = secondEntry.id,
                .normal = normal,
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
            const float inverseMassSum = firstInverseMass + secondInverseMass;
            if (inverseMassSum <= 0.0F) {
                continue;
            }

            const Vec2 relativeVelocity = secondBody->linearVelocity - firstBody->linearVelocity;
            const float velocityAlongNormal = dot(relativeVelocity, contact.normal);
            const float restitution = mixRestitution(
                firstFixtureSlot->value->restitution, secondFixtureSlot->value->restitution);
            const float normalImpulseChange =
                -((1.0F + restitution) * velocityAlongNormal) / inverseMassSum;
            const float oldNormalImpulse = contact.accumulatedNormalImpulse;
            contact.accumulatedNormalImpulse =
                std::max(oldNormalImpulse + normalImpulseChange, 0.0F);
            const Vec2 normalImpulse =
                contact.normal * (contact.accumulatedNormalImpulse - oldNormalImpulse);
            firstBody->linearVelocity -= normalImpulse * firstInverseMass;
            secondBody->linearVelocity += normalImpulse * secondInverseMass;

            const Vec2 postNormalRelativeVelocity =
                secondBody->linearVelocity - firstBody->linearVelocity;
            const Vec2 tangentVector = postNormalRelativeVelocity -
                contact.normal * dot(postNormalRelativeVelocity, contact.normal);
            if (lengthSquared(tangentVector) <= 1.0e-12F) {
                continue;
            }

            const Vec2 tangent = normalized(tangentVector);
            const float tangentImpulseChange =
                -dot(postNormalRelativeVelocity, tangent) / inverseMassSum;
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

Aabb World::fixtureAabb(const Fixture& fixture) const noexcept {
    const BodyState* const state = body(fixture.body);
    if (state == nullptr) {
        return {};
    }

    const Vec2 center = state->position + fixture.localCenter;
    if (fixture.shape == ShapeType::Circle) {
        const Vec2 extent {fixture.radius, fixture.radius};
        return {.min = center - extent, .max = center + extent};
    }
    return {.min = center - fixture.halfExtents, .max = center + fixture.halfExtents};
}

} // namespace render2d::physics
