#pragma once
#include <vector>
#include <glm/vec3.hpp>

struct ParticleComponent {
    float spawnRadius;
    size_t maxParticleCount;
    float gravity;

    std::vector<glm::vec3> positions;

    ParticleComponent(float radius = 1.0f, size_t maxCount = 100, float gravity = -9.81f)
        : spawnRadius(radius), maxParticleCount(maxCount), gravity(gravity) {
    }
};