#ifndef MODEL_H
#define MODEL_H

#include <string>
#include <vector>
#include <memory>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glad/glad.h>

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec2 TexCoords;
};

struct TextureData {
    unsigned int id;
    std::string type;
    std::string path;
};

class Model {
public:
    Model(const std::string& path);
    Model() = default;

    void loadModel(const std::string& path);
    void setMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);

    std::vector<unsigned int> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);

    const std::vector<Vertex>& getVertices() const { return vertices; }
    const std::vector<unsigned int>& getIndices() const { return indices; }
    const std::vector<unsigned int>& getTextures() const { return textures; }
private:
    void processNode(aiNode* node, const aiScene* scene);

    void processMesh(aiMesh* mesh, const aiScene* scene);

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::string directory;
    std::vector<unsigned int> textures;
};

#endif // MODEL_H
