#include "GameManager.h"
#include <glm/glm.hpp>
#include "InputManager.h"

GameManager::GameManager() {}

void GameManager::init() {
    camera = std::make_shared<Camera>(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    window = std::make_shared<Window>(1280, 720, "OpenGL Window", camera);
    inputManager = std::make_shared<InputManager>(window, camera, inputManagerComponent, entityManager, transformManager, *this);
    physicsSystem = std::make_shared<PhysicsSystem>(entityManager, transformManager, velocityManager);
    inputSystem = std::make_shared<InputSystem>(entityManager, inputManagerComponent, velocityManager, inputManager, transformManager);
    renderHandler = std::make_shared<RenderHandler>();
    imguiManager = std::make_shared<ImGuiManager>(window);


    // Entity creation  ----------------------------------------------------------------------------------------------------------------------------------------
    int player = entityManager.createEntity();
    //renderManager.addComponent(player, RenderComponent(std::make_shared<Model>("models/cube2.obj")));
    //colliderManager.addComponent(player, ColliderComponent(ColliderType::SPHERE, glm::vec3(0.0f), glm::vec3(20.0f)));
    transformManager.addComponent(player, TransformComponent(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f), glm::vec3(1.0f)));
    velocityManager.addComponent(player, VelocityComponent(glm::vec3(0.0f, 0.0f, 0.0f)));
    inputManagerComponent.addComponent(player, InputComponent());
    
    int surfaceEntity = entityManager.createEntity();
    heightMapManager = std::make_shared<HeightMapHandler>("external_files/HeightMap.txt", 50000);
    //heightMapManager->BSplineSurface();
    //pointCloudManager.addComponent(surfaceEntity, PointCloudComponent(heightMapManager->getHeightMapVector()));
    std::vector<unsigned int> indices = heightMapManager->getTriangulationIndices();
    std::vector<glm::vec3> normals = heightMapManager->generateNormals(indices);
    triangleSurfaceManager.addComponent(surfaceEntity, TriangleSurfaceMeshComponent (heightMapManager->getHeightMapVector(), indices, normals));
    //triangleSurfaceManager.addComponent(surfaceEntity, TriangleSurfaceMeshComponent (heightMapManager->getHeightMapVector(), heightMapManager->getIndices(), heightMapManager->getNormals()));
    transformManager.addComponent(surfaceEntity, TransformComponent(glm::vec3(1.0f), glm::vec3(0.0f), glm::vec3(1.0f)));
    //  --------------------------------------------------------------------------------------------------------------------------------------------------------


    shader = std::make_shared<Shader>("shaders/vertex_shader.glsl", "shaders/fragment_shader.glsl", camera);
    transform = std::make_shared<Transform>(camera, shader);

    shader->use();
    camera->setProjectionUniform(shader);
    transform->setViewUniform(shader);
    shader->setLightingUniforms(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f, 100.0f, 0.0f), camera->getPosition());
}

void GameManager::run()
{
    while (!window->shouldClose()) {
        glfwPollEvents();
        processInput();
        camera->updateProjectionMatrix(shader);
        render();
        window->swapBuffers();
    }
    shutdown();
}

void GameManager::update() {
    float currentFrameTime = glfwGetTime();
    float deltaTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;

    inputSystem->update(window, deltaTime);
    physicsSystem->update(deltaTime);

    for (int entity : entityManager.getEntities()) {
        if (transformManager.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            transform->update(transformComp, shader);
        }
        if (inputManagerComponent.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            camera->followObject(transformComp.position, transformComp.rotation.y);
        }
    }
    transform->updateViewMatrix(camera);
    transform->setViewUniform(shader);
}

void GameManager::render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shader->use();
    update();

    for (int entity : entityManager.getEntities()) {
        if (transformManager.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            transform->update(transformComp, shader);

            if (renderManager.hasComponent(entity)) {
                RenderComponent& renderComp = renderManager.getComponent(entity);  
                renderHandler->draw(*renderComp.model, shader, false);
            }

            if (pointCloudManager.hasComponent(entity)) {
                PointCloudComponent& pointCloudComp = pointCloudManager.getComponent(entity);
                shader->setUniform("pointColor", glm::vec3(1.0f, 0.0f, 0.0f));
                renderHandler->drawPointCloud(pointCloudComp.points, shader);
            }

            if (triangleSurfaceManager.hasComponent(entity)) {
                TriangleSurfaceMeshComponent& meshComp = triangleSurfaceManager.getComponent(entity);
                shader->setUniform("pointColor", glm::vec3(0.0f, 1.0f, 0.0f));
                renderHandler->drawTriangleMesh(meshComp.vertices, meshComp.indices, meshComp.normals, shader);
            }

            if (colliderManager.hasComponent(entity) && showWireframe) {
                ColliderComponent& colliderComp = colliderManager.getComponent(entity);
                Model colliderMesh = ColliderMeshFactory::createColliderMesh(colliderComp);
                renderHandler->draw(colliderMesh, shader, true);
            }
        }
    }

    //renderHandler->drawPointCloud(heightMapPoints, shader);

    if (showImguiDebug) {
        //imguiManager->DemoWindow("demo window");
        imguiManager->BasicCheckbox("Debug Options", "Show Wireframe", showWireframe);
    }
}

void GameManager::shutdown() {
    imguiManager->shutdown();
    glfwTerminate();
}

void GameManager::processInput() {
    if (!ImGui::GetIO().WantCaptureKeyboard) {
        inputManager->processInput(window, camera);
    }
}

void GameManager::toggleImguiDebug() {
    showImguiDebug = !showImguiDebug;
}