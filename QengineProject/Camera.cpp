#define GLM_ENABLE_EXPERIMENTAL
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#include "Camera.h"

Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitchw)
    : front(glm::vec3(0.0f, 0.0f, -1.0f)), movementSpeed(2.5f), mouseSensitivity(0.1f),
    fov(90.0f), nearPlane(0.1f), farPlane(100.0f) {
    up = glm::vec3(0.0f, 1.0f, 0.0f),
    yaw = -90.0f,
    pitch = 0.0f,

    this->position = position;
    this->worldUp = up;
    this->yaw = yaw;
    this->pitch = pitch;
    this->aspectRatio = 1.77778f;
    updateCameraVectors();
}

// Returns the view matrix calculated using the camera's position and orientation
glm::mat4 Camera::getViewMatrix() {
    return glm::lookAt(position, position + front, up);
}

void Camera::processKeyboard(CameraMovement direction, float deltaTime) {
    float velocity = movementSpeed * deltaTime;
    if (direction == FORWARD) {
        position += front * velocity;
    }
    if (direction == BACKWARD) {
        position -= front * velocity;
    }
    if (direction == LEFT) {
        position -= right * velocity;
    }
    if (direction == RIGHT) {
        position += right * velocity;
    }
    if (direction == UP) {
        position += worldUp * velocity;
    }
    if (direction == DOWN) {
        position -= worldUp * velocity;
    }
}

void Camera::processMouseMovement(float xoffset, float yoffset, bool constrainPitch) {
    xoffset *= mouseSensitivity;
    yoffset *= mouseSensitivity;
    yaw += xoffset;
    pitch += yoffset;

    if (constrainPitch) {
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;
    }

    updateCameraVectors();
}

void Camera::setProjectionUniform(const std::shared_ptr<Shader> shader)
{
    shader->setUniform("projection", projection);
}

void Camera::updateProjectionMatrix(const std::shared_ptr<Shader> shader) {
    projection = glm::perspective(glm::radians(fov), aspectRatio, nearPlane, farPlane);
    shader->setUniform("projection", projection);
    //std::cout << "Ratio: " << aspectRatio << std::endl;
}

// Processes input received from the scroll wheel (not used)
void Camera::processMouseScroll(float yoffset) {}

void Camera::updateCameraVectors() {
    glm::vec3 newFront;
    newFront.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    newFront.y = sin(glm::radians(pitch));
    newFront.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(newFront);

    right = glm::normalize(glm::cross(front, worldUp));
    up = glm::normalize(glm::cross(right, front));
}

glm::vec3 Camera::getPosition() const {
    return position;
}

glm::mat4 Camera::getProjectionMatrix() const
{
    return projection;
}

void Camera::setPosition(const glm::vec3& newPosition) {
    position = newPosition + cameraOffset;
    updateCameraVectors();
}

void Camera::setTarget(const glm::vec3& target) {
    front = glm::normalize(target - position);
    yaw = glm::degrees(atan2(front.z, front.x));
    pitch = glm::degrees(asin(front.y));
    updateCameraVectors();
}

void Camera::followObject(const glm::vec3& objectPosition, float objectYaw) {
    float distance = glm::length(cameraOffset);
    float offsetX = distance * glm::sin(glm::radians(objectYaw));
    float offsetZ = distance * glm::cos(glm::radians(objectYaw));

    glm::vec3 cameraPos = objectPosition - glm::vec3(-offsetX, cameraOffset.y, -offsetZ);

    setPosition(cameraPos);
    setTarget(objectPosition);
}
