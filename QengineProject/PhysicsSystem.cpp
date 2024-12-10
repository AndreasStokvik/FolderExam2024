#include "PhysicsSystem.h"

void PhysicsSystem::update(float deltaTime)
{
    for (int entity : entityManager.getEntities()) {
        // Check if the entity has transform and velocity components
        if (transformManager.hasComponent(entity) && velocityManager.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            VelocityComponent& velocityComp = velocityManager.getComponent(entity);

            glm::vec3 totalForce(0.0f); // Initialize total force acting on the entity

            // Check if the entity is a ball (has a collider component of type SPHERE)
            if (colliderManager.hasComponent(entity)) {
                ColliderComponent& colliderComp = colliderManager.getComponent(entity);

                if (colliderComp.type == ColliderType::SPHERE) {
                    // Calculate the surface normal at the ball's position
                    glm::vec3 normal = getSurfaceNormal(transformComp.position, transformComp.scale);
                    if (glm::length(normal) == 0.0f) continue; // Skip if the normal is invalid

                    // Gravitational force (constant acceleration due to gravity)
                    glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f) * 10.0f;

                    // Calculate the component of gravity along the surface normal (normals resist gravity)
                    glm::vec3 normalComponent = glm::dot(gravity, normal) * normal;

                    // Calculate the gravity force parallel to the surface
                    glm::vec3 gravityOnSurface = gravity - normalComponent;

                    // Accumulate forces acting parallel to the surface
                    totalForce += glm::vec3(gravityOnSurface.x, 0.0f, gravityOnSurface.z);

                    // Adjust the ball's vertical position to stay on the surface
                    if (heightMapManager) {
                        float surfaceHeight = heightMapManager->getHeightAt(
                            transformComp.position.x,
                            transformComp.position.y,
                            transformComp.position.z);
                        transformComp.position.y = surfaceHeight + transformComp.scale.y / 2;
                    }
                    else {
                        transformComp.position.y = transformComp.scale.y / 2;
                    }

                    // Apply Newton's second law: F = ma -> a = F / m
                    glm::vec3 acceleration = totalForce / velocityComp.mass;

                    // Update velocity using the calculated acceleration: v = v0 + a * dt
                    velocityComp.velocity += acceleration * deltaTime;

                    // Apply friction, proportional to normal force and opposing motion
                    float avgFriction = 0.9f; // Coefficient of dynamic friction
                    float velocityMagnitude = glm::length(velocityComp.velocity);

                    if (velocityMagnitude > 0.01f && avgFriction > 0.0f) {
                        glm::vec3 frictionForce = -avgFriction * glm::length(gravityOnSurface) * glm::normalize(velocityComp.velocity);

                        // Reduce velocity due to friction: dv = F_friction * dt
                        glm::vec3 velocityDelta = frictionForce * deltaTime;

                        if (glm::length(velocityDelta) > velocityMagnitude) {
                            // Stop the velocity if friction completely counters motion
                            velocityComp.velocity = glm::vec3(0.0f);
                        }
                        else {
                            velocityComp.velocity += velocityDelta;
                        }
                    }
                }
            }
            // Update position using velocity: p = p0 + v * dt
            transformComp.position += velocityComp.velocity * deltaTime;
        }
    }
}


glm::vec3 PhysicsSystem::getSurfaceNormal(const glm::vec3& position, const glm::vec3& scale) {
    if (heightMapManager) {
        return heightMapManager->getClosestNormal(position.x, position.z, scale);
    }
    else {
        return glm::vec3(0.0f, 1.0f, 0.0f);
    }
}