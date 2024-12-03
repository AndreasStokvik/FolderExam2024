#include "RenderHandler.h"
#include <iostream>

void RenderHandler::draw(const Model& model, const std::shared_ptr<Shader>& shader, bool wireframe) {
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    bool hasTextures = !model.getTextures().empty();
    shader->setUniform("hasTexture", hasTextures);

    if (hasTextures) {
        const auto& textures = model.getTextures();
        for (unsigned int i = 0; i < textures.size(); i++) {
            glActiveTexture(GL_TEXTURE0 + i);
            glBindTexture(GL_TEXTURE_2D, textures[i]);
        }
    }

    glBindVertexArray(model.getVAO());
    glDrawElements(GL_TRIANGLES, model.getIndices().size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void RenderHandler::drawPointCloud(const std::vector<glm::vec3>& points, const std::shared_ptr<Shader>& shader) {
    GLuint pointVAO, pointVBO;
    glGenVertexArrays(1, &pointVAO);
    glGenBuffers(1, &pointVBO);

    glBindVertexArray(pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
    glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    shader->use();
    glBindVertexArray(pointVAO);
    shader->setUniform("pointSize", 1.0f);
    shader->setUniform("hasTexture", false);
    glDrawArrays(GL_POINTS, 0, points.size());

    glBindVertexArray(0);
    glDeleteBuffers(1, &pointVBO);
    glDeleteVertexArrays(1, &pointVAO);
}

void RenderHandler::drawTriangleMesh(
    const std::vector<glm::vec3>& vertices,
    const std::vector<unsigned int>& indices,
    const std::vector<glm::vec3>& normals,
    const std::shared_ptr<Shader>& shader)
{
    if (vertices.size() != normals.size()) {
        std::cerr << "Error: Vertex and normal sizes do not match!" << std::endl;
        return;
    }

    for (unsigned int idx : indices) {
        if (idx >= vertices.size()) {
            std::cerr << "Error: Index out of bounds in indices array!" << std::endl;
            return;
        }
    }

    GLuint meshVAO, meshVBO, meshEBO, meshNBO;

    // Generate and bind the Vertex Array Object (VAO)
    glGenVertexArrays(1, &meshVAO);
    glBindVertexArray(meshVAO);

    // Generate and bind the Vertex Buffer Object (VBO) for vertices
    glGenBuffers(1, &meshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointer for positions (location 0 in shader)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    // Generate and bind the Vertex Buffer Object (VBO) for normals
    glGenBuffers(1, &meshNBO);
    glBindBuffer(GL_ARRAY_BUFFER, meshNBO);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), normals.data(), GL_STATIC_DRAW);

    // Set vertex attribute pointer for normals (location 1 in shader)
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(1);

    // Generate and bind the Element Buffer Object (EBO) for indices
    glGenBuffers(1, &meshEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Use the shader and set uniform
    shader->use();
    shader->setUniform("hasTexture", false);

    // Draw the mesh
    glBindVertexArray(meshVAO);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    // Unbind VAO and delete buffers
    glBindVertexArray(0);
    glDeleteBuffers(1, &meshVBO);
    glDeleteBuffers(1, &meshNBO);
    glDeleteBuffers(1, &meshEBO);
    glDeleteVertexArrays(1, &meshVAO);
}