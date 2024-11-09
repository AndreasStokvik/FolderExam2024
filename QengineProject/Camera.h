#ifndef CAMERA_H
#define CAMERA_H
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#include "Shader.h"

class Shader; // Forward declaration

enum CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera {
public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 right;
    glm::vec3 worldUp;

    glm::mat4 projection;

    float fov;           // Field of view
    float nearPlane;     // Near clipping plane
    float farPlane;      // Far clipping plane

    // Euler angles
    float yaw;
    float pitch;

    // Camera options
    float movementSpeed;
    float mouseSensitivity;
    float zoom;
    float aspectRatio;

    glm::vec3 cameraOffset = glm::vec3(0.0f, 5.0f, 0.0f);

    Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch);

    // Get the view matrix (camera transformation)
    glm::mat4 getViewMatrix();

    // Processes input from keyboard movement
    void processKeyboard(CameraMovement direction, float deltaTime);

    // Processes input from mouse movement
    void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true);

    void setProjectionUniform(const std::shared_ptr<Shader> shader);

    void updateProjectionMatrix(const std::shared_ptr<Shader> shader);

    // Processes input from the mouse scroll wheel (not used)
    void processMouseScroll(float yoffset);

    glm::vec3 getPosition() const;
    glm::mat4 getProjectionMatrix() const;

    void setPosition(const glm::vec3& newPosition);
    void setTarget(const glm::vec3& target);

    void setAspectRatio(float newAspectRatio) {aspectRatio = newAspectRatio;}

    void followObject(const glm::vec3& objectPosition, float objectYaw);
private:
    void updateCameraVectors();

    float lastX = 800.0f / 2.0f;
    float lastY = 600.0f / 2.0f;
    bool firstMouse = true;
};

#endif // CAMERA_H
