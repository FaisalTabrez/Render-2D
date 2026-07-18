#include "render2d/debug/physics_debug_renderer.hpp"

#include "render2d/math/vec2.hpp"

namespace render2d::debug {

void PhysicsDebugRenderer::append(
    const physics::World& world,
    render::DrawList& drawList,
    const PhysicsDebugSettings& settings) const {
    using render2d::math::rotate;
    using render2d::render::Color;

    for (std::uint32_t index = 0; index < world.fixtures_.size(); ++index) {
        const auto& slot = world.fixtures_[index];
        if (!slot.value.has_value()) {
            continue;
        }
        const auto& fixture = *slot.value;
        const physics::BodyState* const body = world.body(fixture.body);
        if (body == nullptr) {
            continue;
        }
        const Color color = body->asleep ? Color::rgb(90, 130, 255) : Color::rgb(255, 200, 40);
        const auto center = body->position + rotate(fixture.localCenter, body->angle);
        if (fixture.shape == physics::ShapeType::Circle) {
            drawList.addCircle(center, fixture.radius, color, settings.layer, index);
        } else {
            const auto bounds = world.fixtureAabb(fixture);
            drawList.addRectangle(
                (bounds.min + bounds.max) * 0.5F,
                (bounds.max - bounds.min) * 0.5F,
                color,
                settings.layer,
                index);
        }
        if (settings.drawAabbs) {
            const auto bounds = world.fixtureAabb(fixture);
            drawList.addRectangle(
                (bounds.min + bounds.max) * 0.5F,
                (bounds.max - bounds.min) * 0.5F,
                Color {.red = 70, .green = 230, .blue = 170, .alpha = 110},
                settings.layer + 1,
                index);
        }
    }

    if (settings.drawContacts) {
        for (const auto& contact : world.contacts_) {
            drawList.addLine(
                contact.point,
                contact.point + contact.normal * 0.35F,
                settings.lineThickness,
                Color::rgb(255, 80, 80),
                settings.layer + 2);
        }
    }
    if (settings.drawJoints) {
        for (const auto& slot : world.joints_) {
            if (!slot.value.has_value()) {
                continue;
            }
            const auto& joint = *slot.value;
            const physics::BodyState* const first = world.body(joint.bodyA);
            const physics::BodyState* const second = world.body(joint.bodyB);
            if (first == nullptr || second == nullptr) {
                continue;
            }
            const auto firstAnchor = first->position + rotate(joint.localAnchorA, first->angle);
            const auto secondAnchor = second->position + rotate(joint.localAnchorB, second->angle);
            drawList.addLine(
                firstAnchor,
                secondAnchor,
                settings.lineThickness,
                Color::rgb(180, 80, 255),
                settings.layer + 3);
        }
    }
}

} // namespace render2d::debug
