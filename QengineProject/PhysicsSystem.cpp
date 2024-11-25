#include "PhysicsSystem.h"

void PhysicsSystem::update(float deltaTime)
{
    for (int entity : entityManager.getEntities()) {
        if (transformManager.hasComponent(entity) && velocityManager.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            VelocityComponent& velocityComp = velocityManager.getComponent(entity);

            glm::vec3 totalForce(0.0f);

            // Check if the entity is a ball
            if (colliderManager.hasComponent(entity)) {
                ColliderComponent& colliderComp = colliderManager.getComponent(entity);

                if (colliderComp.type == ColliderType::SPHERE) {
                    glm::vec3 normal = getSurfaceNormal(transformComp.position, transformComp.scale);
                    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
                    glm::vec3 normalComponent = glm::dot(gravity, normal) * normal;
                    glm::vec3 gravityOnSurface = gravity - normalComponent;

                    totalForce += glm::vec3(gravityOnSurface.x, 0.0f, gravityOnSurface.z);

                    // Ensure the ball stays on the surface
                    float surfaceHeight = heightMapManager->getHeightAt(transformComp.position.x, transformComp.position.z);
                    transformComp.position.y = glm::max(transformComp.position.y, surfaceHeight);

                    // Calculate acceleration from force
                    glm::vec3 acceleration = totalForce / velocityComp.mass;

                    // Update velocity with acceleration
                    velocityComp.velocity += acceleration * deltaTime;

                    // Apply friction
                    float avgFriction = 0.01f; // Coefficient of dynamic friction
                    glm::vec3 horizontalVelocity = glm::vec3(velocityComp.velocity.x, 0.0f, velocityComp.velocity.z);
                    float velocityMagnitude = glm::length(horizontalVelocity);

                    if (velocityMagnitude > 0.01f && avgFriction > 0.0f) {
                        // Friction force = -mu * normal force * velocity direction
                        glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f); // Assume constant gravity for now
                        glm::vec3 frictionForce = -avgFriction * glm::length(gravityOnSurface) * glm::normalize(horizontalVelocity);

                        // Apply friction force to velocity
                        glm::vec3 velocityDelta = frictionForce * deltaTime;
                        if (glm::length(velocityDelta) > velocityMagnitude) {
                            // Stop velocity if friction would reverse it
                            velocityComp.velocity = glm::vec3(0.0f);
                        }
                        else {
                            velocityComp.velocity += velocityDelta;
                        }
                    }
                }
            }
            // Update position with velocity
            transformComp.position += velocityComp.velocity * deltaTime;
        }
    }
}

glm::vec3 PhysicsSystem::getSurfaceNormal(const glm::vec3& position, const glm::vec3& scale) {
    return heightMapManager->getClosestNormal(position.x, position.z, scale);
}