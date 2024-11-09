#pragma once
#include <iostream>
#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>

#include "EntityManager.h"
#include "ComponentManager.h"
#include "../../components/InputComponent.h"
#include "../../components/VelocityComponent.h"
#include "InputManager.h"

class InputSystem {
public:
    InputSystem(EntityManager& entityManager,
        ComponentManager<InputComponent>& inputComponents,
        ComponentManager<VelocityComponent>& velocityComponents,
        std::shared_ptr<InputManager> inputManagerInstance,
        ComponentManager<TransformComponent>& transformComponents)
        : entityManager(entityManager),
        inputComponents(inputComponents),
        velocityComponents(velocityComponents),
        inputManagerInstance(std::move(inputManagerInstance)),
        transformComponents(transformComponents) {}

    void update(const std::shared_ptr<Window>& window, float deltaTime) {
    for (int entity : entityManager.getEntities()) {
        if (inputComponents.hasComponent(entity) && velocityComponents.hasComponent(entity) && transformComponents.hasComponent(entity)) {
            InputComponent& inputComp = inputComponents.getComponent(entity);
            VelocityComponent& velocityComp = velocityComponents.getComponent(entity);
            TransformComponent& transformComp = transformComponents.getComponent(entity);
 
            inputComp.movementDirection = glm::vec3(0.0f); // Reset movement direction

            glm::vec3 rawDirection = inputManagerInstance->getMovementDirection(window);
            glm::vec3 rotatedDirection = rotateDirectionByYaw(rawDirection, transformComp.rotation.y);

            if (glm::any(glm::isnan(rotatedDirection))) {   // Check for NaN values in the rotated direction
                rotatedDirection = glm::vec3(0.0f);         // Reset to zero if NaN
            } else {
                if (glm::length(rotatedDirection) > 0.0f) {
                    rotatedDirection = glm::normalize(rotatedDirection);
                }
            }
            velocityComp.velocity = rotatedDirection * desiredSpeed;
        }
    }
}

glm::vec3 rotateDirectionByYaw(const glm::vec3& direction, float yaw) {
    float radians = glm::radians(yaw);
    glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), radians, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::vec4 rotatedDirection = rotationMatrix * glm::vec4(direction, 0.0f);
    return glm::vec3(rotatedDirection);
}

private:
    EntityManager& entityManager;
    ComponentManager<InputComponent>& inputComponents;
    ComponentManager<VelocityComponent>& velocityComponents;
    ComponentManager<TransformComponent>& transformComponents;
    std::shared_ptr<InputManager> inputManagerInstance;
    float desiredSpeed = 10.0f;                             // Movement speed
};
