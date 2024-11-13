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
#include "Transform.h"
#include "ImGuiManager.h"

// Systems
#include "PhysicsSystem.h"
#include "InputSystem.h"
#include "CameraSystem.h"
#include "RenderHandler.h"

// ECS
#include "ComponentManager.h"
#include "EntityManager.h"
#include "Entity.h"
#include "../../components/RenderComponent.h"
#include "../../components/TransformComponent.h"
#include "../../components/VelocityComponent.h"
#include "../../components/InputComponent.h"
#include "../../components/ColliderComponent.h"
#include "../../components/PointCloudComponent.h"

class InputManager;

class GameManager {
public:
    GameManager();                     // Constructor
    void init();                       // Initializes systems
    void run();                        // Main game loop
    void shutdown();                   // Cleans up resources

    void toggleImguiDebug();           // Toggles the debug UI

    std::vector<glm::vec3> loadPointsFromFile(const std::string& filePath, int maxPoints); // Reading and parsing points from external document

    std::shared_ptr<Camera> camera;
    std::shared_ptr<Window> window;

    std::vector<glm::vec3> heightMapPoints;
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

    std::shared_ptr<Model> model;
    //std::shared_ptr<ColliderMeshFactory> ColliderMeshFactory;

    std::shared_ptr<Shader> shader;

    // ECS
    EntityManager entityManager;
    Entity entity;
    ComponentManager<TransformComponent> transformManager;
    ComponentManager<RenderComponent> renderManager;
    ComponentManager<VelocityComponent> velocityManager;
    ComponentManager<InputComponent> inputManagerComponent;
    ComponentManager<ColliderComponent> colliderManager;
    ComponentManager<PointCloudComponent> pointCloudManager;

    std::shared_ptr<PhysicsSystem> physicsSystem;
    std::shared_ptr<InputSystem> inputSystem;

    float lastFrameTime = 0.0f;


    // helper functions to center the surface
    glm::vec3 calculateCentroid(const std::vector<glm::vec3>& points) {
        glm::vec3 sum(0.0f);
        for (const auto& point : points) {
            sum += point;
        }
        return sum / static_cast<float>(points.size());
    }
    void centerPointsAroundOrigin(std::vector<glm::vec3>& points) {
        glm::vec3 centroid = calculateCentroid(points);
        for (auto& point : points) {
            point -= centroid;
        }
    }
};