#ifndef ENTITY_FACTORY_H
#define ENTITY_FACTORY_H

// Components
#include "../../components/RenderComponent.h"
#include "../../components/TransformComponent.h"
#include "../../components/VelocityComponent.h"
#include "../../components/InputComponent.h"
#include "../../components/ColliderComponent.h"
#include "../../components/PointCloudComponent.h"
#include "../../components/TriangleSurfaceMeshComponent.h"

#include "HeightMapHandler.h"
#include "RenderHandler.h"
#include "EntityManager.h"
#include "ComponentManager.h"

class EntityFactory {
public:
    EntityFactory(
        EntityManager& entityManager,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<RenderComponent>& renderManager,
        ComponentManager<VelocityComponent>& velocityManager,
        ComponentManager<InputComponent>& inputManagerComponent,
        ComponentManager<ColliderComponent>& colliderManager,
        ComponentManager<TriangleSurfaceMeshComponent>& triangleSurfaceManager,
        ComponentManager<PointCloudComponent>& pointCloudManager,
        std::shared_ptr<HeightMapHandler>& heightMapManager, 
        std::shared_ptr<RenderHandler>& renderHandler)
        : entityManager(entityManager),
        transformManager(transformManager),
        renderManager(renderManager),
        velocityManager(velocityManager),
        inputManagerComponent(inputManagerComponent),
        colliderManager(colliderManager),
        triangleSurfaceManager(triangleSurfaceManager),
        pointCloudManager(pointCloudManager),
        heightMapManager(heightMapManager),
        renderHandler(renderHandler){}

    int createPlayer(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& velocity);
    int createSphere(const glm::vec3& position, float radius, const glm::vec3& scale);
    int createSurface(const std::string& heightMapFile, int resolution, int skip, const float scale);
    int createPointCloud(const std::string& heightMapFile, int resolution, int skip, const float scale);

private:
    EntityManager& entityManager;
    ComponentManager<TransformComponent>& transformManager;
    ComponentManager<RenderComponent>& renderManager;
    ComponentManager<VelocityComponent>& velocityManager;
    ComponentManager<InputComponent>& inputManagerComponent;
    ComponentManager<ColliderComponent>& colliderManager;
    ComponentManager<TriangleSurfaceMeshComponent>& triangleSurfaceManager;
    ComponentManager<PointCloudComponent>& pointCloudManager;
    std::shared_ptr<HeightMapHandler>& heightMapManager;
    std::shared_ptr<RenderHandler>& renderHandler;
};

#endif 