#include "EntityFactory.h"

int EntityFactory::createPlayer(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& velocity) {
    int player = entityManager.createEntity();
    //renderManager.addComponent(player, RenderComponent(std::make_shared<Model>("models/cube2.obj")));
    //colliderManager.addComponent(player, ColliderComponent(ColliderType::SPHERE, position, glm::vec3(20.0f)));
    transformManager.addComponent(player, TransformComponent(position, glm::vec3(1.0f), scale));
    velocityManager.addComponent(player, VelocityComponent(velocity));
    inputManagerComponent.addComponent(player, InputComponent());
    return player;
}

int EntityFactory::createSphere(const glm::vec3& position, float radius, const glm::vec3& scale) {
    int sphere = entityManager.createEntity();

    auto sphereModel = std::make_shared<Model>();
    sphereModel->createSphere(radius, 36, 18);

    renderManager.addComponent(sphere, RenderComponent(sphereModel));
    //colliderManager.addComponent(sphere, ColliderComponent(ColliderType::SPHERE, position, glm::vec3(radius)));
    transformManager.addComponent(sphere, TransformComponent(position, glm::vec3(1.0f), scale));
    return sphere;
}

int EntityFactory::createSurface(const std::string& heightMapFile, int resolution, const glm::vec3& scale) {
    int surface = entityManager.createEntity();
    heightMapManager = std::make_shared<HeightMapHandler>(heightMapFile, resolution);
    triangleSurfaceManager.addComponent(surface, TriangleSurfaceMeshComponent(heightMapManager->getHeightMapVector(),heightMapManager->getIndices(),heightMapManager->generateNormals(heightMapManager->getTriangulationIndices())));
    transformManager.addComponent(surface, TransformComponent(glm::vec3(0.0f), glm::vec3(1.0f), scale));
    return surface;
}

int EntityFactory::createPointCloud(const std::string& heightMapFile, int resolution, const glm::vec3& scale) {
    int surface = entityManager.createEntity();
    heightMapManager = std::make_shared<HeightMapHandler>(heightMapFile, resolution);
    pointCloudManager.addComponent(surface, PointCloudComponent(heightMapManager->getHeightMapVector()));
    transformManager.addComponent(surface, TransformComponent(glm::vec3(0.0f), glm::vec3(1.0f), scale));
    return surface;
}