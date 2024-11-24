#pragma once
#include <iostream>
#include <string>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <glm/glm.hpp>

#include "EntityManager.h"
#include "ComponentManager.h"
#include "HeightMapHandler.h"
#include "../../components/TransformComponent.h"
#include "../../components/VelocityComponent.h"
#include "../../components/ColliderComponent.h"

class PhysicsSystem {
public:
    PhysicsSystem(EntityManager& entityManager,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<VelocityComponent>& velocityManager,
        ComponentManager<ColliderComponent>& colliderManager,
        std::shared_ptr<HeightMapHandler>& heightMapManager)
        : entityManager(entityManager), transformManager(transformManager), velocityManager(velocityManager),
        colliderManager(colliderManager), heightMapManager(heightMapManager) {}

    void update(float deltaTime);

    glm::vec3 getSurfaceNormal(const glm::vec3& position);

private:
    EntityManager& entityManager;
    ComponentManager<TransformComponent>& transformManager;
    ComponentManager<VelocityComponent>& velocityManager;
    ComponentManager<ColliderComponent>& colliderManager;
    std::shared_ptr<HeightMapHandler>& heightMapManager;
};
