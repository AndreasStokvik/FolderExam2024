#include "GameManager.h"
#include <glm/glm.hpp>
#include "InputManager.h"

void bindParticleComponentToLua(lua_State* L, ComponentManager<ParticleComponent>& particleManager, EntityManager& entityManager) {
    lua_getglobal(L, "newSpawnRadius");
    if (!lua_isfunction(L, -1)) {
        std::cerr << "Function 'newSpawnRadius' not found" << std::endl;
        lua_pop(L, 1);
        return;
    }

    float oldRadius = 0.0f;
    int oldMaxCount = 0;
    float oldGravity = -9.81f;

    for (int entity : entityManager.getEntities()) {
        if (particleManager.hasComponent(entity)) {
            ParticleComponent& particleComp = particleManager.getComponent(entity);
            oldRadius = particleComp.spawnRadius;
            oldMaxCount = particleComp.maxParticleCount;
            oldGravity = particleComp.gravity;
            break;
        }
    }

    std::cout << "Old Radius: " << oldRadius << ", Old Max Count: " << oldMaxCount << ", Old Gravity: " << oldGravity << std::endl;

    // Update spawnRadius
    if (lua_pcall(L, 0, 1, 0) != LUA_OK) {
        std::cerr << "Error calling newSpawnRadius: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1);
        return;
    }
    float newRadius = lua_isnumber(L, -1) ? static_cast<float>(lua_tonumber(L, -1)) : oldRadius;
    lua_pop(L, 1);

    // Update maxParticleCount
    lua_getglobal(L, "newMaxParticleCount");
    if (!lua_isfunction(L, -1)) {
        std::cerr << "Function 'newMaxParticleCount' not found" << std::endl;
        lua_pop(L, 1);
        return;
    }
    if (lua_pcall(L, 0, 1, 0) != LUA_OK) {
        std::cerr << "Error calling newMaxParticleCount: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1);
        return;
    }
    int newMaxCount = lua_isnumber(L, -1) ? static_cast<int>(lua_tonumber(L, -1)) : oldMaxCount;
    lua_pop(L, 1);

    // Update gravity
    lua_getglobal(L, "newGravity");
    if (!lua_isfunction(L, -1)) {
        std::cerr << "Function 'newGravity' not found" << std::endl;
        lua_pop(L, 1);
        return;
    }
    if (lua_pcall(L, 0, 1, 0) != LUA_OK) {
        std::cerr << "Error calling newGravity: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1);
        return;
    }
    float newGravity = lua_isnumber(L, -1) ? static_cast<float>(lua_tonumber(L, -1)) : oldGravity;
    lua_pop(L, 1);

    for (int entity : entityManager.getEntities()) {
        if (particleManager.hasComponent(entity)) {
            ParticleComponent& particleComp = particleManager.getComponent(entity);
            particleComp.spawnRadius = newRadius;
            particleComp.maxParticleCount = newMaxCount;
            particleComp.gravity = newGravity;
        }
    }

    std::cout << "Updated Radius: " << newRadius
        << ", Updated Max Count: " << newMaxCount
        << ", Updated Gravity: " << newGravity << std::endl;
}
void printLuaStack(lua_State* L) {
    int top = lua_gettop(L);
    std::cout << "Lua Stack (top=" << top << "):" << std::endl;
    for (int i = 1; i <= top; i++) {
        int t = lua_type(L, i);
        switch (t) {
        case LUA_TSTRING:
            std::cout << i << ": String -> " << lua_tostring(L, i) << std::endl;
            break;
        case LUA_TNUMBER:
            std::cout << i << ": Number -> " << lua_tonumber(L, i) << std::endl;
            break;
        case LUA_TFUNCTION:
            std::cout << i << ": Function" << std::endl;
            break;
        default:
            std::cout << i << ": Other -> " << lua_typename(L, t) << std::endl;
        }
    }
}

bool checkSphereCollision(
    const TransformComponent& transformA, const ColliderComponent& colliderA,
    const TransformComponent& transformB, const ColliderComponent& colliderB)
{
    if (colliderA.type == ColliderType::SPHERE && colliderB.type == ColliderType::SPHERE) {
        glm::vec3 posA = transformA.position;
        glm::vec3 posB = transformB.position;

        float radiusA = colliderA.dimensions.x / 2.0f;
        float radiusB = colliderB.dimensions.x / 2.0f;

        float deltaX = posA.x - posB.x;
        float deltaZ = posA.z - posB.z;
        float distanceSquared = (deltaX * deltaX) + (deltaZ * deltaZ);
        float radiusSum = radiusA + radiusB;

        return distanceSquared <= radiusSum * radiusSum;
    }
    return false;
}

void detectAndResolveCollisions(EntityManager& entityManager,
    ComponentManager<TransformComponent>& transformManager,
    ComponentManager<ColliderComponent>& colliderManager,
    ComponentManager<VelocityComponent>& velocityManager,
    float deltaTime)
{
    const auto& entities = entityManager.getEntities();

    for (auto itA = entities.begin(); itA != entities.end(); ++itA) {
        auto itB = itA;
        ++itB;
        for (; itB != entities.end(); ++itB) {
            int entityA = *itA;
            int entityB = *itB;

            if (transformManager.hasComponent(entityA) && colliderManager.hasComponent(entityA) &&
                transformManager.hasComponent(entityB) && colliderManager.hasComponent(entityB))
            {
                TransformComponent& transformA = transformManager.getComponent(entityA);
                ColliderComponent& colliderA = colliderManager.getComponent(entityA);

                TransformComponent& transformB = transformManager.getComponent(entityB);
                ColliderComponent& colliderB = colliderManager.getComponent(entityB);

                if (checkSphereCollision(transformA, colliderA, transformB, colliderB)) {
                    std::cout << "Collision detected between entities " << entityA << " and " << entityB << std::endl;

                    if (!colliderA.isStatic && !colliderB.isStatic) {
                        if (velocityManager.hasComponent(entityA) && velocityManager.hasComponent(entityB)) {
                            VelocityComponent& velA = velocityManager.getComponent(entityA);
                            VelocityComponent& velB = velocityManager.getComponent(entityB);

                            glm::vec3 collisionNormal = glm::normalize(transformB.position - transformA.position);
                            float relativeVelocity = glm::dot(velB.velocity - velA.velocity, collisionNormal);

                            if (relativeVelocity < 0) {
                                float restitution = 1.0f; // Elasticity factor
                                float impulse = (1 + restitution) * relativeVelocity / 2.0f;

                                velA.velocity += impulse * collisionNormal * deltaTime;
                                velB.velocity -= impulse * collisionNormal * deltaTime;
                            }
                        }
                    }
                }
            }
        }
    }
}



GameManager::GameManager() : luaState(nullptr) {}

void GameManager::init() {
    camera = std::make_shared<Camera>(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    window = std::make_shared<Window>(1280, 720, "OpenGL Window", camera);
    inputManager = std::make_shared<InputManager>(window, camera, inputManagerComponent, entityManager, transformManager, *this);
    inputSystem = std::make_shared<InputSystem>(8, entityManager, inputManagerComponent, velocityManager, inputManager, transformManager);
    renderHandler = std::make_shared<RenderHandler>();

    luaState = luaL_newstate();
    luaL_openlibs(luaState);

    // Entity creation  ----------------------------------------------------------------------------------------------------------------------------------------
    entityFactory = std::make_shared<EntityFactory>(
        entityManager, transformManager, renderManager, 
        velocityManager, inputManagerComponent, colliderManager, 
        triangleSurfaceManager, pointCloudManager, heightMapManager, 
        renderHandler, particleManager);
    
    int player = entityFactory->createPlayer(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f), glm::vec3(1.0f));
    //int triangleSurface = entityFactory->createSurface("external_files/testMap1.txt", -1, 0, 1.0f);
    int particleEntity = entityFactory->createParticleEntity(glm::vec3(0.0f, 0.0f, 0.0f), 25.0f, 100000);
    int particleEntity1 = entityFactory->createParticleEntity(glm::vec3(25.0f, 0.0f, 0.0f), 25.0f, 100000);
    int particleEntity2 = entityFactory->createParticleEntity(glm::vec3(0.0f, 0.0f, 25.0f), 25.0f, 100000);
    //  --------------------------------------------------------------------------------------------------------------------------------------------------------
    
    if (luaL_dofile(luaState, "config.lua") != LUA_OK) {
        std::cerr << "Error loading Lua script: " << lua_tostring(luaState, -1) << std::endl;
    }
    else {
        bindParticleComponentToLua(luaState, particleManager, entityManager);
    }
    printLuaStack(luaState);

    shader = std::make_shared<Shader>("shaders/vertex_shader.glsl", "shaders/fragment_shader.glsl", camera);
    transform = std::make_shared<Transform>(camera, shader);
    imguiManager = std::make_shared<ImGuiManager>(window);

    shader->use();
    camera->setProjectionUniform(shader);
    transform->setViewUniform(shader);
    shader->setLightingUniforms(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(50.0f, 50.0f, 0.0f), camera->getPosition());
    particleSystem = std::make_shared<ParticleSystem>();
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

    detectAndResolveCollisions(entityManager, transformManager, colliderManager, velocityManager, deltaTime*100);

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
                shader->setUniform("pointColor", glm::vec3(1.0f));
                shader->setUniform("pointSize", 2.0f);
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
    lua_close(luaState);
    imguiManager->shutdown();
    glfwTerminate();
}

bool keyPressedLastFrame = false;

void GameManager::processInput() {
    if (!ImGui::GetIO().WantCaptureKeyboard) {
        inputManager->processInput(window, camera, entityFactory);

        bool keyPressed = glfwGetKey(window->getWindow(), GLFW_KEY_R) == GLFW_PRESS;
        if (keyPressed && !keyPressedLastFrame) {
            if (luaL_dofile(luaState, "config.lua") != LUA_OK) {
                std::cerr << "Error loading Lua script: " << lua_tostring(luaState, -1) << std::endl;
            }
            else {
                bindParticleComponentToLua(luaState, particleManager, entityManager);
            }
            printLuaStack(luaState);
            std::cout << "Lua script reloaded." << std::endl;
        }
        keyPressedLastFrame = keyPressed;
    }
}