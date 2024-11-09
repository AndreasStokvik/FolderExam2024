#include "ColliderMeshFactory.h"
#include <glm/gtc/constants.hpp>

Model ColliderMeshFactory::createColliderMesh(const ColliderComponent& collider) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    switch (collider.type) {
    case ColliderType::BOX:
        generateBoxMesh(vertices, indices, collider.dimensions);
        break;
    case ColliderType::SPHERE:
        generateSphereMesh(vertices, indices, collider.dimensions.x / 2.0f, 16, 16); // Use x dimension as diameter
        break;
    case ColliderType::CAPSULE:
        // Implement capsule mesh generation here
        break;
    }

    Model colliderModel;
    colliderModel.setMesh(vertices, indices);  // Assume setMesh function in Model
    return colliderModel;
}

// Generate a box mesh
void ColliderMeshFactory::generateBoxMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, const glm::vec3& dimensions) {
    glm::vec3 halfSize = dimensions * 0.5f;

    // Define the 8 corners of the box
    glm::vec3 positions[8] = {
        {-halfSize.x, -halfSize.y, -halfSize.z},
        { halfSize.x, -halfSize.y, -halfSize.z},
        { halfSize.x,  halfSize.y, -halfSize.z},
        {-halfSize.x,  halfSize.y, -halfSize.z},
        {-halfSize.x, -halfSize.y,  halfSize.z},
        { halfSize.x, -halfSize.y,  halfSize.z},
        { halfSize.x,  halfSize.y,  halfSize.z},
        {-halfSize.x,  halfSize.y,  halfSize.z}
    };

    for (const auto& pos : positions) {
        vertices.push_back(Vertex{ pos, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f) });
    }

    // Define indices for the box (12 triangles)
    unsigned int boxIndices[] = {
        0, 1, 2, 2, 3, 0, // front face
        4, 5, 6, 6, 7, 4, // back face
        0, 4, 7, 7, 3, 0, // left face
        1, 5, 6, 6, 2, 1, // right face
        0, 1, 5, 5, 4, 0, // bottom face
        3, 2, 6, 6, 7, 3  // top face
    };
    indices.insert(indices.end(), std::begin(boxIndices), std::end(boxIndices));
}

// Generate a simple sphere mesh (latitude-longitude)
void ColliderMeshFactory::generateSphereMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, float radius, int sectors, int stacks) {
    for (int i = 0; i <= stacks; ++i) {
        float stackAngle = glm::pi<float>() / 2 - i * glm::pi<float>() / stacks; // starting from pi/2 to -pi/2
        float xy = radius * cosf(stackAngle);  // r * cos(u)
        float z = radius * sinf(stackAngle);   // r * sin(u)

        for (int j = 0; j <= sectors; ++j) {
            float sectorAngle = j * 2 * glm::pi<float>() / sectors; // starting from 0 to 2pi

            float x = xy * cosf(sectorAngle); // r * cos(u) * cos(v)
            float y = xy * sinf(sectorAngle); // r * cos(u) * sin(v)

            vertices.push_back(Vertex{ glm::vec3(x, y, z), glm::normalize(glm::vec3(x, y, z)), glm::vec2((float)j / sectors, (float)i / stacks) });
        }
    }

    // Generate indices
    for (int i = 0; i < stacks; ++i) {
        int k1 = i * (sectors + 1);     // beginning of current stack
        int k2 = k1 + sectors + 1;      // beginning of next stack

        for (int j = 0; j < sectors; ++j, ++k1, ++k2) {
            if (i != 0) {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }

            if (i != (stacks - 1)) {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }
        }
    }
}
