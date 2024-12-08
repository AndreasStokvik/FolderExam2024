#include <vector>
#include <glm/vec3.hpp>
#include <random>
#include <iostream>
#include "../../components/ParticleComponent.h"

class ParticleSystem {
public:
    // Gravity factor for particles
    float gravity;

    ParticleSystem(float gravityFactor = -9.81f) : gravity(gravityFactor) {}

    void updateParticles(ParticleComponent& particleComponent, float deltaTime) {


        // Spawn new particles if below max count
        if (particleComponent.positions.size() < particleComponent.maxParticleCount) {
            spawnParticle(particleComponent);
        }

        // Update particle positions
        for (auto& position : particleComponent.positions) {
            position.y += gravity * deltaTime; // Apply gravity to y-coordinate
        }

        // Remove particles that fall below a certain height (optional cleanup)
        // For example, remove if `y < 0`
        particleComponent.positions.erase(
            std::remove_if(particleComponent.positions.begin(), particleComponent.positions.end(),
                [](const glm::vec3& pos) { return pos.y < -2.0f; }),
            particleComponent.positions.end());
    }

private:
    void spawnParticle(ParticleComponent& particleComponent) {
        // Check if adding a new particle would exceed the limit
        if (particleComponent.positions.size() >= particleComponent.maxParticleCount) {
            // Remove the oldest particle
            particleComponent.positions.erase(particleComponent.positions.begin());
        }

        // Generate random x and z within the spawn radius, with fixed y
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(-particleComponent.spawnRadius, particleComponent.spawnRadius);

        float x, z;
        do {
            x = dist(gen);
            z = dist(gen);
        } while (x * x + z * z > particleComponent.spawnRadius * particleComponent.spawnRadius); // Ensure inside circle

        float fixedY = 20.0f; // Fixed starting height (e.g., "sky height")
        particleComponent.positions.emplace_back(x, fixedY, z);
    }
};
