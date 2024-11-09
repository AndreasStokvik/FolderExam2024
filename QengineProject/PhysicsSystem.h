#pragma once
#include <iostream>
#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>

#include "EntityManager.h"
#include "ComponentManager.h"
#include "../../components/TransformComponent.h"
#include "../../components/VelocityComponent.h"

class PhysicsSystem {
public:
    PhysicsSystem(EntityManager& entityManager,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<VelocityComponent>& velocityManager)
        : entityManager(entityManager), transformManager(transformManager), velocityManager(velocityManager) {}

    void update(float deltaTime) {
        for (int entity : entityManager.getEntities()) {
            if (transformManager.hasComponent(entity) && velocityManager.hasComponent(entity)) {
                TransformComponent& transformComp = transformManager.getComponent(entity);
                VelocityComponent& velocityComp = velocityManager.getComponent(entity);

                transformComp.position += velocityComp.velocity * deltaTime;
                //std::cout << "velocity: " << glm::to_string(velocityComp.velocity) << std::endl;
            }
        }
    }

private:
    EntityManager& entityManager;
    ComponentManager<TransformComponent>& transformManager;
    ComponentManager<VelocityComponent>& velocityManager;
};
