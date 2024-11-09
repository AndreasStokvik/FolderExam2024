#include "Transform.h"
#include "Camera.h"
#include "Shader.h"
#include <glm/gtc/matrix_transform.hpp>

Transform::Transform(const std::shared_ptr<Camera>& camera, const std::shared_ptr<Shader>& shader) {
    view = camera->getViewMatrix();
    setViewUniform(shader);
}

void Transform::update(const TransformComponent& transformComp, const std::shared_ptr<Shader>& shader) {
    model = glm::mat4(1.0f);
    model = glm::translate(model, transformComp.position);
    model = glm::rotate(model, glm::radians(transformComp.rotation.x), glm::vec3(1, 0, 0));
    model = glm::rotate(model, glm::radians(transformComp.rotation.y), glm::vec3(0, 1, 0));
    model = glm::rotate(model, glm::radians(transformComp.rotation.z), glm::vec3(0, 0, 1));
    model = glm::scale(model, transformComp.scale);

    setModelUniform(shader);
}

void Transform::setViewUniform(const std::shared_ptr<Shader>& shader) {
    shader->setUniform("view", view);
}

void Transform::setModelUniform(const std::shared_ptr<Shader>& shader) {
    shader->setUniform("model", model);
}

void Transform::updateViewMatrix(const std::shared_ptr<Camera>& camera) {
    if (camera) { // Check if the camera is valid
        view = camera->getViewMatrix(); // Update the view matrix
    }
}