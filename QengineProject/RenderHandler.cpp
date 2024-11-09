// RenderHandler.cpp
#include "RenderHandler.h"

void RenderHandler::draw(const Model& model, const std::shared_ptr<Shader>& shader, bool wireframe) {
    if (wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }
    else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    
    // Set textures
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

    // Draw elements
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, model.getIndices().size(), GL_UNSIGNED_INT, 0);

    // Cleanup
    glBindVertexArray(0);
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
