#include "GameManager.h"
#include <glm/glm.hpp>
#include "InputManager.h"

void bindParticleComponentToLua(lua_State* L, ComponentManager<ParticleComponent>& particleManager, EntityManager& entityManager) {
    lua_getglobal(L, "newSpawnRadius"); // Push the function onto the stack
    if (!lua_isfunction(L, -1)) {
        std::cerr << "Function 'newSpawnRadius' not found" << std::endl;
        lua_pop(L, 1); // Pop the invalid value off the stack
        return;
    }

    float oldRadius = 0.0f;
    int oldMaxCount = 0;

    // Retrieve old values from one of the components
    for (int entity : entityManager.getEntities()) {
        if (particleManager.hasComponent(entity)) {
            ParticleComponent& particleComp = particleManager.getComponent(entity);
            oldRadius = particleComp.spawnRadius;
            oldMaxCount = particleComp.maxParticleCount;
            break; // Only need one value
        }
    }

    std::cout << "Old Radius: " << oldRadius << ", Old Max Count: " << oldMaxCount << std::endl;

    // Push the argument for newSpawnRadius
    lua_pushnumber(L, oldRadius);

    // Call the Lua function (1 argument, 1 result)
    if (lua_pcall(L, 1, 1, 0) != LUA_OK) {
        std::cerr << "Error calling newSpawnRadius: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1); // Remove the error message
        return;
    }

    // Check if the return value is a number
    float newRadius = oldRadius; // Default to old value in case of error
    if (lua_isnumber(L, -1)) {
        newRadius = static_cast<float>(lua_tonumber(L, -1));
        std::cout << "New Radius: " << newRadius << std::endl;
    }
    else {
        std::cerr << "newSpawnRadius did not return a number" << std::endl;
    }

    // Clean up the Lua stack
    lua_pop(L, 1);

    // Update maxParticleCount
    lua_getglobal(L, "newMaxParticleCount"); // Push the function onto the stack
    if (!lua_isfunction(L, -1)) {
        std::cerr << "Function 'newMaxParticleCount' not found" << std::endl;
        lua_pop(L, 1); // Pop the invalid value off the stack
        return;
    }

    // Push the argument for newMaxParticleCount
    lua_pushnumber(L, oldMaxCount);

    // Call the Lua function (1 argument, 1 result)
    if (lua_pcall(L, 1, 1, 0) != LUA_OK) {
        std::cerr << "Error calling newMaxParticleCount: " << lua_tostring(L, -1) << std::endl;
        lua_pop(L, 1); // Remove the error message
        return;
    }

    // Check if the return value is a number
    int newMaxCount = oldMaxCount; // Default to old value in case of error
    if (lua_isnumber(L, -1)) {
        newMaxCount = static_cast<int>(lua_tonumber(L, -1));
        std::cout << "New Max Count: " << newMaxCount << std::endl;
    }
    else {
        std::cerr << "newMaxParticleCount did not return a number" << std::endl;
    }

    // Clean up the Lua stack
    lua_pop(L, 1);

    // Update the components
    for (int entity : entityManager.getEntities()) {
        if (particleManager.hasComponent(entity)) {
            ParticleComponent& particleComp = particleManager.getComponent(entity);
            particleComp.spawnRadius = newRadius;
            particleComp.maxParticleCount = newMaxCount;
        }
    }
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

GameManager::GameManager() : luaState(nullptr) {}

void GameManager::init() {
    camera = std::make_shared<Camera>(glm::vec3(0.0f, 0.0f, 3.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    window = std::make_shared<Window>(1280, 720, "OpenGL Window", camera);
    inputManager = std::make_shared<InputManager>(window, camera, inputManagerComponent, entityManager, transformManager, *this);
    inputSystem = std::make_shared<InputSystem>(256.0f, entityManager, inputManagerComponent, velocityManager, inputManager, transformManager);
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
    int triangleSurface = entityFactory->createSurface("external_files/testMap1.txt", -1, 0, 1.0f);
    int particleEntity = entityFactory->createParticleEntity(glm::vec3(0.0f, 0.0f, 0.0f), 25.0f, 100000);
    //  --------------------------------------------------------------------------------------------------------------------------------------------------------

    
    const std::string& scriptPath = "config.lua";
    
    if (luaL_dofile(luaState, scriptPath.c_str()) != LUA_OK) {
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
    particleSystem = std::make_shared<ParticleSystem>(-9.81f / 16);
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
            const std::string& scriptPath = "config.lua";

            if (luaL_dofile(luaState, scriptPath.c_str()) != LUA_OK) {
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