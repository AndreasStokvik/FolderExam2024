#pragma once
#include <vector>
#include <glm/glm.hpp>

struct TriangleSurfaceMeshComponent
{
    std::vector<glm::vec3> vertices;    // List of vertices (positions)
    std::vector<unsigned int> indices;  // Indices for triangles
    std::vector<glm::vec3> normals;    // List of vertices (positions)

    TriangleSurfaceMeshComponent(
        const std::vector<glm::vec3>& vertices,
        const std::vector<unsigned int>& indices,
        const std::vector<glm::vec3>& normals)
        : vertices(vertices), indices(indices), normals(normals)  {
    }

    TriangleSurfaceMeshComponent() = default;
};

