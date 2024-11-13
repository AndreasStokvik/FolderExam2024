// RenderHandler.h
#ifndef RENDER_HANDLER_H
#define RENDER_HANDLER_H

#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glad/glad.h>
#include "Shader.h"
#include "Model.h"

class RenderHandler {
public:
    void draw(const Model& model, const std::shared_ptr<Shader>& shader, bool wireframe);

    void drawPointCloud(const std::vector<glm::vec3>& points, const std::shared_ptr<Shader>& shader);

private:
    void setupMesh(const Model& model);
    GLuint VAO, VBO, EBO;
};

#endif
