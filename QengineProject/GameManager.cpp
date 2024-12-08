#include "GameManager.h"
#include <glm/glm.hpp>
#include "InputManager.h"

GameManager::GameManager() {}

void GameManager::init() {
    camera = std::make_shared<Camera>(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    window = std::make_shared<Window>(1280, 720, "OpenGL Window", camera);
    inputManager = std::make_shared<InputManager>(window, camera, inputManagerComponent, entityManager, transformManager, *this);
    inputSystem = std::make_shared<InputSystem>(256.0f, entityManager, inputManagerComponent, velocityManager, inputManager, transformManager);
    renderHandler = std::make_shared<RenderHandler>();

    // Entity creation  ----------------------------------------------------------------------------------------------------------------------------------------
    entityFactory = std::make_shared<EntityFactory>(
        entityManager, transformManager, renderManager, 
        velocityManager, inputManagerComponent, colliderManager, 
        triangleSurfaceManager, pointCloudManager, heightMapManager, 
        renderHandler, particleManager);
    
    int player = entityFactory->createPlayer(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::vec3(1.0f));
    int triangleSurface = entityFactory->createSurface("external_files/testMap1.txt", -1, 0, 1.0f);
    int particleEntity = entityFactory->createParticleEntity(glm::vec3(0.0f, 0.0f, 0.0f), 25.0f, 100000);
    int particleEntity1 = entityFactory->createParticleEntity(glm::vec3(50.0f, 0.0f, 25.0f), 25.0f, 100000);
    int particleEntity2 = entityFactory->createParticleEntity(glm::vec3(25.0f, 0.0f, 50.0f), 25.0f, 100000);
    //  --------------------------------------------------------------------------------------------------------------------------------------------------------

    shader = std::make_shared<Shader>("shaders/vertex_shader.glsl", "shaders/fragment_shader.glsl", camera);
    transform = std::make_shared<Transform>(camera, shader);
    imguiManager = std::make_shared<ImGuiManager>(window);

    shader->use();
    camera->setProjectionUniform(shader);
    transform->setViewUniform(shader);
    shader->setLightingUniforms(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(50.0f, 50.0f, 0.0f), camera->getPosition());
    particleSystem = std::make_shared<ParticleSystem>(-9.81f / 4);
    physicsSystem = std::make_shared<PhysicsSystem>(entityManager, transformManager, velocityManager, colliderManager, heightMapManager);
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

            if (particleManager.hasComponent(entity)) {
                ParticleComponent& particleComp = particleManager.getComponent(entity);
                particleSystem->updateParticles(particleComp, transformComp, deltaTime);
            }
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
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
        if (transformManager.hasComponent(entity)) {
            TransformComponent& transformComp = transformManager.getComponent(entity);
            transform->update(transformComp, shader);

            if (renderManager.hasComponent(entity)) {
                shader->setUniform("pointColor", glm::vec3(1.0f, 0.0f, 0.0f));
                RenderComponent& renderComp = renderManager.getComponent(entity);
                renderHandler->draw(*renderComp.model, shader, false);
            }

            if (particleManager.hasComponent(entity)) {
                ParticleComponent& particleComp = particleManager.getComponent(entity);
                shader->setUniform("pointColor", glm::vec3(1.0f)); // Blue particles, for example
                shader->setUniform("pointSize", 2.0f); // Larger size for particles
                renderHandler->drawPointCloud(particleComp.positions, shader);
            }

            if (triangleSurfaceManager.hasComponent(entity)) {
                TriangleSurfaceMeshComponent& meshComp = triangleSurfaceManager.getComponent(entity);
                shader->setUniform("pointColor", glm::vec3(0.0f, 1.0f, 0.0f));
                //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);              // Wireframe for debug
                renderHandler->drawTriangleMesh(meshComp.vertices, meshComp.indices, meshComp.normals, shader);
            }

            if (colliderManager.hasComponent(entity) && showWireframe) {
                ColliderComponent& colliderComp = colliderManager.getComponent(entity);
                Model colliderMesh = ColliderMeshFactory::createColliderMesh(colliderComp);
                shader->setUniform("pointColor", glm::vec3(0.0f, 0.0f, 1.0f));
                renderHandler->draw(colliderMesh, shader, true);
            }
            //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }

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
        inputManager->processInput(window, camera, entityFactory);
    }
}