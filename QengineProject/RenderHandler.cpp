#include "RenderHandler.h"

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

    setupMesh(model);

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, model.getIndices().size(), GL_UNSIGNED_INT, 0);

    // Cleanup
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
    const std::shared_ptr<Shader>& shader)
{
    GLuint meshVAO, meshVBO, meshEBO;

    glGenVertexArrays(1, &meshVAO);
    glBindVertexArray(meshVAO);

    glGenBuffers(1, &meshVBO);
    glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &meshEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, meshEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    shader->use();
    shader->setUniform("hasTexture", false);

    glBindVertexArray(meshVAO); 
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

    glBindVertexArray(0);
    glDeleteBuffers(1, &meshVBO);
    glDeleteBuffers(1, &meshEBO);
    glDeleteVertexArrays(1, &meshVAO);
}

void RenderHandler::setupMesh(const Model& model) {
    const auto& vertices = model.getVertices();
    const auto& indices = model.getIndices();

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Position));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
    glEnableVertexAttribArray(1);

    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, TexCoords));
    glEnableVertexAttribArray(2);
}
