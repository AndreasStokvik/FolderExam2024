#pragma once
#include <vector>
#include <glm/glm.hpp>

struct TriangleSurfaceMeshComponent
{
    std::vector<glm::vec3> vertices;    // List of vertices (positions)
    std::vector<glm::vec3> normals;    // List of vertices (positions)
    std::vector<unsigned int> indices;  // Indices for triangles

    TriangleSurfaceMeshComponent(
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::vec3>& normals,
        const std::vector<unsigned int>& indices)
        : vertices(vertices), normals(normals), indices(indices) {
    }

    TriangleSurfaceMeshComponent() = default;
};

