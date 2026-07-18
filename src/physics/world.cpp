#include "render2d/physics/world.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace render2d::physics {
namespace {

using math::dot;
using math::isFinite;
using math::length;
using math::normalized;

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

[[nodiscard]] constexpr float mixRestitution(const float first, const float second) noexcept {
    return std::max(first, second);
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
        !std::isfinite(definition.mass) || !std::isfinite(definition.linearDamping) ||
        !std::isfinite(definition.gravityScale) || definition.linearDamping < 0.0F ||
        (definition.type == BodyType::Dynamic && definition.mass <= 0.0F)) {
        throw std::invalid_argument("BodyDefinition contains an invalid value");
    }

    BodyState state {
        .type = definition.type,
        .position = definition.position,
        .previousPosition = definition.position,
        .linearVelocity = definition.linearVelocity,
        .mass = definition.type == BodyType::Dynamic ? definition.mass : 0.0F,
        .linearDamping = definition.linearDamping,
        .gravityScale = definition.gravityScale,
    };

    for (std::uint32_t index = 0; index < bodies_.size(); ++index) {
        BodySlot& slot = bodies_[index];
        if (!slot.value.has_value()) {
            slot.value = state;
            slot.force = {};
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
    slot->generation = nextGeneration(slot->generation);
    return true;
}

FixtureId World::createCircleFixture(
    const BodyId bodyId, const CircleFixtureDefinition& definition) {
    if (bodySlot(bodyId) == nullptr) {
        return {};
    }
    if (!std::isfinite(definition.radius) || definition.radius <= 0.0F ||
        !isFinite(definition.localCenter) || !std::isfinite(definition.restitution) ||
        definition.restitution < 0.0F) {
        throw std::invalid_argument("CircleFixtureDefinition contains an invalid value");
    }

    CircleFixture fixture {
        .body = bodyId,
        .radius = definition.radius,
        .localCenter = definition.localCenter,
        .restitution = definition.restitution,
        .sensor = definition.sensor,
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

bool World::applyForce(const BodyId id, const Vec2 force) noexcept {
    BodySlot* const slot = bodySlot(id);
    if (slot == nullptr || slot->value->type != BodyType::Dynamic || !isFinite(force)) {
        return false;
    }

    slot->force += force;
    return true;
}

void World::step(const float deltaTime) {
    if (!std::isfinite(deltaTime) || deltaTime <= 0.0F) {
        throw std::invalid_argument("World::step requires a positive finite delta time");
    }

    events_.clear();

    for (BodySlot& slot : bodies_) {
        if (!slot.value.has_value()) {
            continue;
        }

        BodyState& state = *slot.value;
        state.previousPosition = state.position;
        if (state.type == BodyType::Dynamic) {
            const float invMass = inverseMass(state);
            state.linearVelocity += (settings_.gravity * state.gravityScale + slot.force * invMass) * deltaTime;
            state.linearVelocity *= 1.0F / (1.0F + state.linearDamping * deltaTime);
        }
        if (movesWithVelocity(state.type)) {
            state.position += state.linearVelocity * deltaTime;
        }
        slot.force = {};
    }

    contacts_.clear();
    std::set<std::pair<std::uint64_t, std::uint64_t>> currentContacts;

    for (std::uint32_t firstIndex = 0; firstIndex < fixtures_.size(); ++firstIndex) {
        const FixtureSlot& firstSlot = fixtures_[firstIndex];
        if (!firstSlot.value.has_value()) {
            continue;
        }
        const FixtureId firstId {.index = firstIndex, .generation = firstSlot.generation};
        const CircleFixture& first = *firstSlot.value;
        BodyState* const firstBody = body(first.body);
        if (firstBody == nullptr) {
            continue;
        }

        for (std::uint32_t secondIndex = firstIndex + 1U; secondIndex < fixtures_.size(); ++secondIndex) {
            const FixtureSlot& secondSlot = fixtures_[secondIndex];
            if (!secondSlot.value.has_value()) {
                continue;
            }
            const FixtureId secondId {.index = secondIndex, .generation = secondSlot.generation};
            const CircleFixture& second = *secondSlot.value;
            if (first.body == second.body) {
                continue;
            }
            BodyState* const secondBody = body(second.body);
            if (secondBody == nullptr ||
                (firstBody->type != BodyType::Dynamic && secondBody->type != BodyType::Dynamic)) {
                continue;
            }

            const Vec2 delta = (secondBody->position + second.localCenter) -
                               (firstBody->position + first.localCenter);
            const float combinedRadius = first.radius + second.radius;
            const float distance = length(delta);
            if (distance >= combinedRadius) {
                continue;
            }

            const auto pair = std::pair {fixtureKey(firstId), fixtureKey(secondId)};
            currentContacts.insert(pair);
            const ContactEventType eventType = activeContacts_.contains(pair)
                ? ContactEventType::Stay
                : ContactEventType::Begin;
            const Vec2 normal = normalized(delta);
            const float penetration = combinedRadius - distance;
            const bool sensor = first.sensor || second.sensor;

            contacts_.push_back({
                .fixtureA = firstId,
                .fixtureB = secondId,
                .normal = normal,
                .penetration = penetration,
                .sensor = sensor,
            });
            events_.push_back({
                .type = eventType,
                .fixtureA = firstId,
                .fixtureB = secondId,
                .normal = normal,
                .penetration = penetration,
                .sensor = sensor,
            });
        }
    }

    for (const ContactConstraint& contact : contacts_) {
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
        for (const ContactConstraint& contact : contacts_) {
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
            if (velocityAlongNormal >= 0.0F) {
                continue;
            }

            const float restitution = mixRestitution(
                firstFixtureSlot->value->restitution, secondFixtureSlot->value->restitution);
            const float impulseMagnitude =
                -((1.0F + restitution) * velocityAlongNormal) / inverseMassSum;
            const Vec2 impulse = contact.normal * impulseMagnitude;
            firstBody->linearVelocity -= impulse * firstInverseMass;
            secondBody->linearVelocity += impulse * secondInverseMass;
        }
    }

    for (const auto& pair : activeContacts_) {
        if (!currentContacts.contains(pair)) {
            events_.push_back({
                .type = ContactEventType::End,
                .fixtureA = fixtureIdFromKey(pair.first),
                .fixtureB = fixtureIdFromKey(pair.second),
            });
        }
    }
    activeContacts_ = std::move(currentContacts);
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

} // namespace render2d::physics
