#pragma once
#include <glm/glm.hpp>
#include <memory>
#include "../components/TransformComponent.h"

class Camera;
class Shader;

class Transform {
public:
    glm::mat4 model;
    glm::mat4 view;

    Transform(const std::shared_ptr<Camera>& camera, const std::shared_ptr<Shader>& shader);

    void update(const TransformComponent& transformComp, const std::shared_ptr<Shader>& shader);

    void setViewUniform(const std::shared_ptr<Shader>& shader);
    void setModelUniform(const std::shared_ptr<Shader>& shader);
    void updateViewMatrix(const std::shared_ptr<Camera>& camera);
};