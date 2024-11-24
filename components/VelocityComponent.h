#pragma once
#include <glm/glm.hpp>


struct VelocityComponent {
    glm::vec3 velocity;
    float mass;

    VelocityComponent(const glm::vec3& initialVelocity = glm::vec3(0.0f), const float mass = 1.0f)
        : velocity(initialVelocity), mass(mass) {}
};