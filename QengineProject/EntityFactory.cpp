#include "EntityFactory.h"

int EntityFactory::createPlayer(const glm::vec3& position, const glm::vec3& scale, const glm::vec3& velocity) {
    int player = entityManager.createEntity();
    //renderManager.addComponent(player, RenderComponent(std::make_shared<Model>("models/cube2.obj")));

    auto sphereModel = std::make_shared<Model>();
    sphereModel->createSphere(0.5f, 36, 18);
    renderManager.addComponent(player, RenderComponent(sphereModel));
    colliderManager.addComponent(player, ColliderComponent(ColliderType::SPHERE, position, glm::vec3(1.0f)));

    transformManager.addComponent(player, TransformComponent(position, glm::vec3(1.0f), scale));
    velocityManager.addComponent(player, VelocityComponent(velocity, 1.0f));
    inputManagerComponent.addComponent(player, InputComponent());
    return player;
}

int EntityFactory::createSphere(const glm::vec3& position, float radius, const glm::vec3& scale) {
    int sphere = entityManager.createEntity();

    auto sphereModel = std::make_shared<Model>();
    sphereModel->createSphere(radius, 36, 18);

    renderManager.addComponent(sphere, RenderComponent(sphereModel));

    colliderManager.addComponent(sphere, ColliderComponent(ColliderType::SPHERE, position, glm::vec3(radius*2)));
    transformManager.addComponent(sphere, TransformComponent(position, glm::vec3(1.0f), scale));
    velocityManager.addComponent(sphere, VelocityComponent(glm::vec3(0.0f), 1.0f));
    return sphere;
}

int EntityFactory::createSurface(const std::string& heightMapFile, int resolution, int skip, const float scale) {
    int surface = entityManager.createEntity();
    heightMapManager = std::make_shared<HeightMapHandler>(heightMapFile, resolution, skip, scale);
    triangleSurfaceManager.addComponent(surface, TriangleSurfaceMeshComponent(
        heightMapManager->getHeightMapVector(),
        heightMapManager->getIndices(),
        heightMapManager->generateNormals(heightMapManager->getTriangulationIndices())));
    transformManager.addComponent(surface, TransformComponent(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(scale)));
    return surface;
}

int EntityFactory::createPointCloud(const std::string& heightMapFile, int resolution, int skip, const float scale) {
    int surface = entityManager.createEntity();
    heightMapManager = std::make_shared<HeightMapHandler>(heightMapFile, resolution, skip, scale);
    pointCloudManager.addComponent(surface, PointCloudComponent(heightMapManager->getHeightMapVector()));
    transformManager.addComponent(surface, TransformComponent(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(scale)));
    return surface;
}

int EntityFactory::createParticleEntity(const glm::vec3& position, float radius, size_t particleCount) {
    int particleEntity = entityManager.createEntity();
    particleManager.addComponent(particleEntity, ParticleComponent(radius, particleCount));
    transformManager.addComponent(particleEntity, TransformComponent(position, glm::vec3(0.0f), glm::vec3(1.0f)));
    return particleEntity;
}