#include <memory>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <vector>
#include <glm/vec3.hpp>

#include "Window.h"
#include "Shader.h"
#include "Camera.h"
#include "Model.h"
#include "ColliderMeshFactory.h"
#include "EntityFactory.h"
#include "Transform.h"
#include "ImGuiManager.h"

// Systems
#include "PhysicsSystem.h"
#include "InputSystem.h"
#include "CameraSystem.h"
#include "RenderHandler.h"
#include "HeightMapHandler.h"
#include "ParticleSystem.h"

// ECS
#include "ComponentManager.h"
#include "EntityManager.h"
#include "../../components/RenderComponent.h"
#include "../../components/TransformComponent.h"
#include "../../components/VelocityComponent.h"
#include "../../components/InputComponent.h"
#include "../../components/ColliderComponent.h"
#include "../../components/PointCloudComponent.h"
#include "../../components/TriangleSurfaceMeshComponent.h"
#include "../../components/ParticleComponent.h"

extern "C" {
#include "lua54/include/lua.h"
#include "lua54/include/lauxlib.h"
#include "lua54/include/lualib.h"
}
#ifdef _WIN32
#pragma comment(lib, "lua54/lua54.lib")
#endif

class InputManager;

class GameManager {
public:
    GameManager();                     // Constructor
    void init();                       // Initializes systems
    void run();                        // Main game loop
    void shutdown();                   // Cleans up resources
    void toggleImguiDebug() {showImguiDebug = !showImguiDebug;}; // Toggles the debug UI

    std::shared_ptr<Camera> camera;
    std::shared_ptr<Window> window;

private:
    void update();                     // Updates game logic
    void render();                     // Renders the scene
    void processInput();               // Handles input processing

    bool showWireframe = false;
    bool showImguiDebug = false;
    std::unordered_map<int, Model> colliderMeshes;

    std::shared_ptr<RenderHandler> renderHandler;
    std::shared_ptr<InputManager> inputManager;
    std::shared_ptr<ImGuiManager> imguiManager;
    std::shared_ptr<Transform> transform;
    std::shared_ptr<HeightMapHandler> heightMapManager;

    std::shared_ptr<Model> model;
    std::shared_ptr<EntityFactory> entityFactory;
    std::shared_ptr<Shader> shader;
    lua_State* luaState;

    // ECS
    EntityManager entityManager;
    ComponentManager<TransformComponent> transformManager;
    ComponentManager<RenderComponent> renderManager;
    ComponentManager<VelocityComponent> velocityManager;
    ComponentManager<InputComponent> inputManagerComponent;
    ComponentManager<ColliderComponent> colliderManager;
    ComponentManager<PointCloudComponent> pointCloudManager;
    ComponentManager<TriangleSurfaceMeshComponent> triangleSurfaceManager;
    ComponentManager<ParticleComponent> particleManager;

    std::shared_ptr<PhysicsSystem> physicsSystem;
    std::shared_ptr<InputSystem> inputSystem;
    std::shared_ptr<ParticleSystem> particleSystem;

    float lastFrameTime = 0.0f;
};