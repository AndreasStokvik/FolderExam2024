#include "InputManager.h"
#include "GameManager.h"
#include <GLFW/glfw3.h>

struct InputCameraContext {
    InputManager* inputManager;
    std::shared_ptr<Camera> camera;
};

std::unique_ptr<Timer> timer = std::make_unique<Timer>();

InputManager::InputManager(const std::shared_ptr<Window>& window, const std::shared_ptr<Camera>& camera, 
    ComponentManager<InputComponent>& inputManagerComponent, EntityManager& entityManager,
    ComponentManager<TransformComponent>& transformManager, GameManager& gameManager)
    : lastX(400), lastY(300), firstMouse(true), inputManagerComponent(inputManagerComponent),
    entityManager(entityManager), transformManager(transformManager), gameManager(gameManager) {
    keyStates.fill(false);
    setMouseCallback(window, camera);
}

void InputManager::processInput(const std::shared_ptr <Window>& window, const std::shared_ptr<Camera>& camera) {
    float deltaTime = timer->getDeltaTime();
    // Movement keys (Arrow keys, Space, Shift)
    if (isKeyPressed(window, GLFW_KEY_UP))
        camera->processKeyboard(FORWARD, deltaTime);
    if (isKeyPressed(window, GLFW_KEY_DOWN))
        camera->processKeyboard(BACKWARD, deltaTime);
    if (isKeyPressed(window, GLFW_KEY_LEFT))
        camera->processKeyboard(LEFT, deltaTime);
    if (isKeyPressed(window, GLFW_KEY_RIGHT))
        camera->processKeyboard(RIGHT, deltaTime);
    if (isKeyPressed(window, GLFW_KEY_SPACE))
        camera->processKeyboard(UP, deltaTime);
    if (isKeyPressed(window, GLFW_KEY_LEFT_SHIFT))
        camera->processKeyboard(DOWN, deltaTime);

    // Toggle cursor and camera movement with Escape key
    if (isKeyPressedOnce(window, GLFW_KEY_ESCAPE)) {
        window->toggleCursor();
        toggleMouseControl();
        gameManager.toggleImguiDebug();
    }
}

// Checking if a key was pressed just once (spam protection)
bool InputManager::isKeyPressedOnce(const std::shared_ptr <Window>& window, int key) {
    if (glfwGetKey(window->getWindow(), key) == GLFW_PRESS) {
        if (!keyStates[key]) {
            keyStates[key] = true;
            return true;
        }
    }
    else if (glfwGetKey(window->getWindow(), key) == GLFW_RELEASE) {
        keyStates[key] = false;
    }
    return false;
}

bool InputManager::isKeyPressed(const std::shared_ptr <Window>& window, int key) {
    return glfwGetKey(window->getWindow(), key) == GLFW_PRESS;
}

glm::vec3 InputManager::getMovementDirection(const std::shared_ptr<Window>& window) {
    glm::vec3 direction(0.0f);

    if (isKeyPressed(window, GLFW_KEY_W))
        direction.z -= 1.0f;
    if (isKeyPressed(window, GLFW_KEY_S))
        direction.z += 1.0f;
    if (isKeyPressed(window, GLFW_KEY_A))
        direction.x -= 1.0f;
    if (isKeyPressed(window, GLFW_KEY_D))
        direction.x += 1.0f;
    if (isKeyPressed(window, GLFW_KEY_SPACE))
        direction.y += 1.0f;
    if (isKeyPressed(window, GLFW_KEY_LEFT_SHIFT))
        direction.y -= 1.0f;

    return glm::normalize(direction);
}

void InputManager::toggleMouseControl()
{
    mouseControl = !mouseControl;
}

void InputManager::handleMouseMovement(double xpos, double ypos, const std::shared_ptr<Camera>& camera) {
    if (firstMouse) {
        lastX = static_cast<float>(xpos);
        lastY = static_cast<float>(ypos);
        firstMouse = false;
    }

    float xoffset = static_cast<float>(xpos) - lastX;
    lastX = static_cast<float>(xpos);

    if (mouseControl) {
        for (int entity : entityManager.getEntities()) {
            if (inputManagerComponent.hasComponent(entity)) {
                TransformComponent& transformComp = transformManager.getComponent(entity);
                float objectYawOffset = xoffset * objectSensitivity;
                transformComp.rotation.y += objectYawOffset;
            }
        }
    }
}

void InputManager::setMouseCallback(const std::shared_ptr<Window>& window, const std::shared_ptr<Camera>& camera) {
    auto* context = new InputCameraContext{ this, camera };
    glfwSetWindowUserPointer(window->getWindow(), context);

    glfwSetCursorPosCallback(window->getWindow(), [](GLFWwindow* glfwWindow, double xpos, double ypos) {
        auto* context = static_cast<InputCameraContext*>(glfwGetWindowUserPointer(glfwWindow));
        context->inputManager->handleMouseMovement(xpos, ypos, context->camera);
        });
}

