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
    int entityId = entityManager.createEntity();
    renderManager.addComponent(entityId, RenderComponent(std::make_shared<Model>("models/cube2.obj")));
    transformManager.addComponent(entityId, TransformComponent(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(0.0f), glm::vec3(1.0f)));
    velocityManager.addComponent(entityId, VelocityComponent(glm::vec3(0.0f, 0.0f, 0.0f)));
    inputManagerComponent.addComponent(entityId, InputComponent());
    colliderManager.addComponent(entityId, ColliderComponent(ColliderType::SPHERE, glm::vec3(0.0f), glm::vec3(20.0f)));

    int entity2 = entityManager.createEntity();
    renderManager.addComponent(entity2, RenderComponent(std::make_shared<Model>("models/test1.obj")));
    transformManager.addComponent(entity2, TransformComponent(glm::vec3(10.0f, 0.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.5f), glm::vec3(0.5f)));
    colliderManager.addComponent(entity2, ColliderComponent(ColliderType::BOX, glm::vec3(0.0f), glm::vec3(10.0f)));

    int entity3 = entityManager.createEntity();
    renderManager.addComponent(entity3, RenderComponent(std::make_shared<Model>("models/environment.obj")));
    transformManager.addComponent(entity3, TransformComponent(glm::vec3(2.0f, -5.0f, 2.0f), glm::vec3(0.0f, 0.0f, 0.5f), glm::vec3(0.12f)));

    int entityId4 = entityManager.createEntity();
    renderManager.addComponent(entityId4, RenderComponent(std::make_shared<Model>("models/cube2.obj")));
    transformManager.addComponent(entityId4, TransformComponent(glm::vec3(-20.0f, 10.0f, -20.0f), glm::vec3(45.0f, 0.0f, 45.0f), glm::vec3(10.0f)));
    //  --------------------------------------------------------------------------------------------------------------------------------------------------------


    shader = std::make_shared<Shader>("shaders/vertex_shader.glsl", "shaders/fragment_shader.glsl", camera);
    transform = std::make_shared<Transform>(camera, shader);

    shader->use();
    camera->setProjectionUniform(shader);
    transform->setViewUniform(shader);
    shader->setLightingUniforms(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(2.0f, 10.0f, 2.0f), camera->getPosition());
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

    transform->updateViewMatrix(camera);
    transform->setViewUniform(shader);
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

            if (colliderManager.hasComponent(entity) && showWireframe) {
                ColliderComponent& colliderComp = colliderManager.getComponent(entity);
                Model colliderMesh = ColliderMeshFactory::createColliderMesh(colliderComp);
                renderHandler->draw(colliderMesh, shader, true);
            }
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
        inputManager->processInput(window, camera);
    }
}

void GameManager::toggleImguiDebug() {
    showImguiDebug = !showImguiDebug;
}