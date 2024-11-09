#pragma once
#include <glm/glm.hpp>

struct CameraComponent {
    glm::vec3 position;          // Camera position in world space
    glm::vec3 front;             // Direction the camera is facing
    glm::vec3 up;                // Up vector for the camera
    glm::vec3 right;             // Right vector (cross product of front and up)
    float yaw;                   // Horizontal rotation angle
    float pitch;                 // Vertical rotation angle
    float fov;                   // Field of view for perspective projection
    float aspectRatio;           // Aspect ratio of the viewport
    float nearClip;              // Near clipping plane
    float farClip;               // Far clipping plane
};
