#pragma once
#include "Model.h"
#include "../../components/ColliderComponent.h"

class ColliderMeshFactory {
public:
    static Model createColliderMesh(const ColliderComponent& collider);

private:
    static void generateBoxMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, const glm::vec3& dimensions);
    static void generateSphereMesh(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, float radius, int sectors, int stacks);
    // Add capsule mesh generation later if needed
};
