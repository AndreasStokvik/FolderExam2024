#include "Shader.h"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

Shader::Shader(const std::string& vertexPath, const std::string& fragmentPath, std::shared_ptr<Camera> camera) {
    std::string vertexCode = loadShaderSource(vertexPath);
    std::string fragmentCode = loadShaderSource(fragmentPath);

    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexCode);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentCode);

    programID = glCreateProgram();
    glAttachShader(programID, vertexShader);
    glAttachShader(programID, fragmentShader);
    glLinkProgram(programID);

    // Check for linking errors
    int success;
    char infoLog[512];
    glGetProgramiv(programID, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(programID, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    // Clean up
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    use();
    setLightingUniforms(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(2.0f, 2.0f, 2.0f), camera->getPosition());
}

Shader::~Shader(){glDeleteProgram(programID);}

void Shader::use() const {glUseProgram(programID);}

// Setting different uniforms in the shader
void Shader::setUniform(const std::string& name, float value){glUniform1f(glGetUniformLocation(programID, name.c_str()), value);}
void Shader::setUniform(const std::string& name, int value){glUniform1i(glGetUniformLocation(programID, name.c_str()), value);}
void Shader::setUniform(const std::string& name, const glm::mat4& matrix) {glUniformMatrix4fv(glGetUniformLocation(programID, name.c_str()), 1, GL_FALSE, &matrix[0][0]);}
void Shader::setUniform(const std::string& name, const glm::vec3& vector) {
    int location = glGetUniformLocation(programID, name.c_str());
    if (location == -1) {
        std::cerr << "Warning: uniform '" << name << "' not found or not used in the shader program!" << std::endl;
    }
    glUniform3fv(location, 1, glm::value_ptr(vector));
}
void Shader::setLightingUniforms(const glm::vec3& lightColor, const glm::vec3& lightPos, const glm::vec3& viewPos)
{
    setUniform("lightColor", lightColor);
    setUniform("lightPos", lightPos);
    setUniform("viewPos", viewPos);
}

// Load the shader source code from a file
std::string Shader::loadShaderSource(const std::string& path) {
    std::ifstream shaderFile;
    shaderFile.open(path);

    if (!shaderFile.is_open()) {
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << path << std::endl;
        throw std::runtime_error("Shader file couldn't be read.");
    }

    std::stringstream buffer;
    buffer << shaderFile.rdbuf();
    shaderFile.close();

    return buffer.str();
}

// Compile a shader (vertex or fragment)
unsigned int Shader::compileShader(unsigned int type, const std::string& source) {
    unsigned int shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    // Check for compilation errors
    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    return shader;
}
