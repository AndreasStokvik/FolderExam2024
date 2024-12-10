#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <random>
#include <iostream>
#include <cmath>

#include "../../components/ParticleComponent.h"
#include "../../components/TransformComponent.h"

class ParticleSystem {
public:
    ParticleSystem() {}

    void updateParticles(ParticleComponent& particleComponent, TransformComponent& transformComponent, float deltaTime) {
        float gravity = particleComponent.gravity;

        // Spawn new particles if below max count
        if (particleComponent.positions.size() < particleComponent.maxParticleCount) {
            spawnParticle(particleComponent, transformComponent);
        }

        // Update particle positions with gravity
        for (auto& position : particleComponent.positions) {
            position.y += gravity * deltaTime;
        }

        // Remove particles that fall below a certain height
        particleComponent.positions.erase(
            std::remove_if(particleComponent.positions.begin(), particleComponent.positions.end(),
                [](const glm::vec3& pos) { return pos.y < -5.0f; }),
            particleComponent.positions.end());
    }

private:
    void spawnParticle(ParticleComponent& particleComponent, TransformComponent& transformComponent) {
        // Generate random x and z within the spawn radius, (y is fixed)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(-particleComponent.spawnRadius, particleComponent.spawnRadius);

        float x, z;
        do {
            x = dist(gen);
            z = dist(gen);
        } while (x * x + z * z > particleComponent.spawnRadius * particleComponent.spawnRadius);

        x *= transformComponent.scale.x;
        z *= transformComponent.scale.z;
        float angle = transformComponent.rotation.y;
        float rotatedX = x * cos(angle) - z * sin(angle);
        float rotatedZ = x * sin(angle) + z * cos(angle);

        float fixedY = 20.0f; // Fixed starting height for the particles (sky height)

        particleComponent.positions.emplace_back(
            transformComponent.position.x + rotatedX,
            fixedY,
            transformComponent.position.z + rotatedZ
        );
    }
};
