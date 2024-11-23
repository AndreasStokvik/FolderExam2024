#include "Model.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
#include <filesystem>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Model::Model(const std::string& path) {
    loadModel(path);
}

// Load model function
void Model::loadModel(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        std::cerr << "ERROR::MODEL::File not found: " << path << std::endl;
        std::exit(EXIT_FAILURE);
        return;
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "ERROR::ASSIMP::" << importer.GetErrorString() << std::endl;
        std::exit(EXIT_FAILURE);
        return;
    }

    directory = path.substr(0, path.find_last_of('/'));

    processNode(scene->mRootNode, scene);
}

void Model::setMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices)
{
    this->vertices = vertices;
    this->indices = indices;
}

// Process each node in the scene
void Model::processNode(aiNode* node, const aiScene* scene) {
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMesh(mesh, scene); // Directly process each mesh
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        processNode(node->mChildren[i], scene);
    }
}

// Load texture from file
unsigned int TextureFromFile(const char* path, const std::string& directory) {
    std::string filename = std::string(path);
    filename = directory + '/' + filename;

    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrChannels;
    unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nrChannels, 0);
    if (data) {
        GLenum format;
        if (nrChannels == 1)
            format = GL_RED;
        else if (nrChannels == 3)
            format = GL_RGB;
        else if (nrChannels == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else {
        std::cerr << "Failed to load texture: " << filename << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}

// Process a single mesh
void Model::processMesh(aiMesh* mesh, const aiScene* scene) {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<unsigned int> meshTextures;

    // Process vertices
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;

        // Position
        vertex.Position = glm::vec3(
            mesh->mVertices[i].x,
            mesh->mVertices[i].y,
            mesh->mVertices[i].z
        );

        // Normals
        if (mesh->HasNormals()) {
            vertex.Normal = glm::vec3(
                mesh->mNormals[i].x,
                mesh->mNormals[i].y,
                mesh->mNormals[i].z
            );
        }
        else {
            vertex.Normal = glm::vec3(0.0f, 0.0f, 0.0f); // Default normal
        }

        // Texture coordinates
        if (mesh->mTextureCoords[0]) {
            vertex.TexCoords = glm::vec2(
                mesh->mTextureCoords[0][i].x,
                mesh->mTextureCoords[0][i].y
            );
        }
        else {
            vertex.TexCoords = glm::vec2(0.0f, 0.0f); // Default to (0, 0)
        }

        vertices.push_back(vertex);
    }

    // Process indices
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
            indices.push_back(face.mIndices[j]);
        }
    }

    // Process material textures
    if (mesh->mMaterialIndex >= 0) {
        aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
        std::vector<unsigned int> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
        textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    }

    // Store vertices and indices
    this->vertices.insert(this->vertices.end(), vertices.begin(), vertices.end());
    this->indices.insert(this->indices.end(), indices.begin(), indices.end());
}

std::vector<unsigned int> Model::loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName) {
    std::vector<unsigned int> textures;
    for (unsigned int i = 0; i < mat->GetTextureCount(type); i++) {
        aiString str;
        mat->GetTexture(type, i, &str);
        unsigned int textureID = TextureFromFile(str.C_Str(), directory);
        textures.push_back(textureID);
    }
    return textures;
}

void generateSphere(std::vector<Vertex>& vertices, std::vector<unsigned int>& indices, float radius, unsigned int sectorCount, unsigned int stackCount) {
    vertices.clear();
    indices.clear();

    float x, y, z, xy;                           // vertex position
    float nx, ny, nz, lengthInv = 1.0f / radius; // normal
    float s, t;                                  // texture coordinates

    float sectorStep = 2 * M_PI / sectorCount;
    float stackStep = M_PI / stackCount;
    float stackAngle, sectorAngle;

    // Generate vertices
    for (unsigned int i = 0; i <= stackCount; ++i) {
        stackAngle = M_PI / 2 - i * stackStep; // starting from pi/2 to -pi/2
        xy = radius * cosf(stackAngle);       // r * cos(u)
        z = radius * sinf(stackAngle);        // r * sin(u)

        for (unsigned int j = 0; j <= sectorCount; ++j) {
            sectorAngle = j * sectorStep; // starting from 0 to 2pi

            x = xy * cosf(sectorAngle); // x = r * cos(u) * cos(v)
            y = xy * sinf(sectorAngle); // y = r * cos(u) * sin(v)

            Vertex vertex;
            vertex.Position = glm::vec3(x, y, z);
            vertex.Normal = glm::vec3(x * lengthInv, y * lengthInv, z * lengthInv);
            vertex.TexCoords = glm::vec2((float)j / sectorCount, (float)i / stackCount);
            vertices.push_back(vertex);
        }
    }

    // Generate indices
    unsigned int k1, k2;
    for (unsigned int i = 0; i < stackCount; ++i) {
        k1 = i * (sectorCount + 1);
        k2 = k1 + sectorCount + 1;

        for (unsigned int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
            if (i != 0) {
                indices.push_back(k1);
                indices.push_back(k2);
                indices.push_back(k1 + 1);
            }
            if (i != (stackCount - 1)) {
                indices.push_back(k1 + 1);
                indices.push_back(k2);
                indices.push_back(k2 + 1);
            }
        }
    }
}

void Model::createSphere(float radius, unsigned int sectorCount, unsigned int stackCount) {
    std::vector<Vertex> sphereVertices;
    std::vector<unsigned int> sphereIndices;
    generateSphere(sphereVertices, sphereIndices, radius, sectorCount, stackCount);
    setMesh(sphereVertices, sphereIndices);
}