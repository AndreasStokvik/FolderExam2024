#pragma once
#include <glm/glm.hpp>


struct VelocityComponent {
    glm::vec3 velocity;

    VelocityComponent(const glm::vec3& initialVelocity = glm::vec3(0.0f))
        : velocity(initialVelocity) {}
};