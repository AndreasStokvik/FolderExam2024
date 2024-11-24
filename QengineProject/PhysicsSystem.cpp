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
                    glm::vec3 normal = getSurfaceNormal(transformComp.position);
                    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
                    glm::vec3 gravityOnSurface = glm::dot(gravity, normal) * normal;

                    totalForce += gravityOnSurface;
                }
            }

            // Calculate acceleration from force
            glm::vec3 acceleration = totalForce / velocityComp.mass;

            // Update velocity with acceleration
            velocityComp.velocity += acceleration * deltaTime;

            // Update position with velocity
            transformComp.position += velocityComp.velocity * deltaTime;

            // Ensure the ball stays on the surface
            float surfaceHeight = heightMapManager->getHeightAt(transformComp.position.x, transformComp.position.z);
            transformComp.position.y = glm::max(transformComp.position.y, surfaceHeight);
        }
    }
}

glm::vec3 PhysicsSystem::getSurfaceNormal(const glm::vec3& position) {
    return heightMapManager->getClosestNormal(position.x, position.z);
}