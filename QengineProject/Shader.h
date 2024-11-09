#ifndef SHADER_H
#define SHADER_H
#include <string>
#include <glm/glm.hpp>
#include <memory>

#include "Camera.h"

class Camera;   // Forward declaration

class Shader {
public:
    Shader(const std::string& vertexPath, const std::string& fragmentPath, std::shared_ptr<Camera> camera);
    ~Shader();

    void use() const;                                                         // Activate the shader program
    void setUniform(const std::string& name, float value);                    // Set float uniform
    void setUniform(const std::string& name, int value);                      // Set int uniform
    void setUniform(const std::string& name, const glm::mat4& matrix);        // Set matrix uniform
    void setUniform(const std::string& name, const glm::vec3& vector);        // Set vec3 uniform
    void setLightingUniforms(const glm::vec3& lightColor, const glm::vec3& lightPos, const glm::vec3& viewPos);

private:
    unsigned int programID;                                                   // The OpenGL shader program ID

    std::string loadShaderSource(const std::string& path);                    // Load shader code from a file
    unsigned int compileShader(unsigned int type, const std::string& source); // Compile a shader (vertex/fragment)
};

#endif  // SHADER_H
