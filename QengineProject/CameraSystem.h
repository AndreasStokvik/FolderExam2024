#pragma once
#include "../components/CameraComponent.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class CameraSystem {
public:
    void update(CameraComponent& camera, float deltaTime) {
        // Update the direction vectors based on yaw and pitch
        camera.front = glm::normalize(glm::vec3(
            cos(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch)),
            sin(glm::radians(camera.pitch)),
            sin(glm::radians(camera.yaw)) * cos(glm::radians(camera.pitch))
        ));
        camera.right = glm::normalize(glm::cross(camera.front, glm::vec3(0.0f, 1.0f, 0.0f)));
        camera.up = glm::normalize(glm::cross(camera.right, camera.front));

        // Compute the view matrix
        viewMatrix = glm::lookAt(camera.position, camera.position + camera.front, camera.up);

        // Compute the projection matrix
        projectionMatrix = glm::perspective(
            glm::radians(camera.fov),
            camera.aspectRatio,
            camera.nearClip,
            camera.farClip
        );
    }

    glm::mat4 getViewMatrix() const { return viewMatrix; }
    glm::mat4 getProjectionMatrix() const { return projectionMatrix; }

private:
    glm::mat4 viewMatrix;
    glm::mat4 projectionMatrix;
};