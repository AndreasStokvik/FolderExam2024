#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H
#include <array>
#include <memory>

#include "Window.h"
#include "Camera.h"
#include "ComponentManager.h"
#include "EntityManager.h"
#include "../../components/InputComponent.h"
#include "../../components/TransformComponent.h"

class GameManager;

class Timer {
public:
    float getDeltaTime() {
        float currentFrame = glfwGetTime();
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        return deltaTime;
    }
private:
    float lastFrame = 0.0f;
};

class InputManager {
public:
    InputManager(const std::shared_ptr<Window>& window, const std::shared_ptr<Camera>& camera, 
        ComponentManager<InputComponent>& inputManagerComponent, EntityManager& entityManager, 
        ComponentManager<TransformComponent>&transformManager, GameManager& gameManager);

    void processInput(const std::shared_ptr<Window>& window, const std::shared_ptr<Camera>& camera);
    bool isKeyPressedOnce(const std::shared_ptr<Window>& window, int key);
    bool isKeyPressed(const std::shared_ptr<Window>& window, int key);

    glm::vec3 getMovementDirection(const std::shared_ptr<Window>& window);

    void toggleMouseControl();
    void handleMouseMovement(double xpos, double ypos, const std::shared_ptr<Camera>& camera);
    void setMouseCallback(const std::shared_ptr<Window>& window, const std::shared_ptr<Camera>& camera);

private:
    bool mouseControl;
    bool firstMouse;
    float lastX, lastY;
    std::array<bool, 1024> keyStates;
    float objectSensitivity = -0.1f;

    ComponentManager<InputComponent>& inputManagerComponent;
    ComponentManager<TransformComponent>& transformManager;
    EntityManager& entityManager;
    GameManager& gameManager;
};

#endif // INPUTMANAGER_H
