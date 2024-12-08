#pragma once
#include <vector>
#include <glm/vec3.hpp>

struct ParticleComponent {
    std::vector<glm::vec3> positions;
    float spawnRadius;
    size_t maxParticleCount;
    float timeDelta;

    ParticleComponent(float radius = 1.0f, size_t maxCount = 100, float delta = 0.1f)
        : spawnRadius(radius), maxParticleCount(maxCount), timeDelta(delta) {
        positions.reserve(maxParticleCount);
    }
};