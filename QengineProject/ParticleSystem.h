#include <vector>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <random>
#include <iostream>
#include <cmath> // For sin and cos

#include "../../components/ParticleComponent.h"
#include "../../components/TransformComponent.h"

class ParticleSystem {
public:
    float gravity;

    ParticleSystem(float gravityFactor = -9.81f) : gravity(gravityFactor) {}

    void updateParticles(ParticleComponent& particleComponent, TransformComponent& transformComponent, float deltaTime) {
        // Spawn new particles if below max count
        if (particleComponent.positions.size() < particleComponent.maxParticleCount) {
            spawnParticle(particleComponent, transformComponent);
        }

        // Update particle positions with gravity
        for (auto& position : particleComponent.positions) {
            position.y += gravity * deltaTime; // Apply gravity to y-coordinate
        }

        // Remove particles that fall below a certain height
        particleComponent.positions.erase(
            std::remove_if(particleComponent.positions.begin(), particleComponent.positions.end(),
                [](const glm::vec3& pos) { return pos.y < -5.0f; }),
            particleComponent.positions.end());
    }

private:
    void spawnParticle(ParticleComponent& particleComponent, TransformComponent& transformComponent) {
        // Generate random x and z within the spawn radius, with fixed y
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(-particleComponent.spawnRadius, particleComponent.spawnRadius);

        float x, z;
        do {
            x = dist(gen);
            z = dist(gen);
        } while (x * x + z * z > particleComponent.spawnRadius * particleComponent.spawnRadius); // Ensure inside circle

        // Apply scaling to the spawn position
        x *= transformComponent.scale.x;
        z *= transformComponent.scale.z;

        // Apply rotation (assuming around Y-axis)
        float angle = transformComponent.rotation.y; // Rotation around Y axis in radians
        float rotatedX = x * cos(angle) - z * sin(angle);
        float rotatedZ = x * sin(angle) + z * cos(angle);

        // Apply the position of the entity as an offset
        float fixedY = 20.0f; // Fixed starting height for the particles (sky height)

        // Add the particle with the transformation applied
        particleComponent.positions.emplace_back(
            transformComponent.position.x + rotatedX,
            fixedY, // Keep the fixed Y height for simplicity (could also add some randomness)
            transformComponent.position.z + rotatedZ
        );
    }
};
