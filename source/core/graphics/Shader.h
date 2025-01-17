#ifndef PROGRAM_H
#define PROGRAM_H

#include <string>
#include <glm/glm.hpp>
#include <glad/glad.h>


namespace n2m::graphics {
class Shader {
public:
    Shader();

    ~Shader();

    // Load vertex & fragment shaders from file paths
    bool loadShaders(const std::string &vertPath, const std::string &fragPath);

    void use() const;

    // set uniforms overloads
    void setUniform(const std::string &uniformName, int value);

    void setUniform(const std::string &uniformName, float value);

    void setUniform(const std::string &uniformName, bool value);

    void setUniform(const std::string &name, const glm::vec4 &vector);

    void setUniform(const std::string &name, const glm::vec3 &vector);

    void setUniform(const std::string &name, const glm::mat4 &matrix);


    unsigned int getID() const { return programID; }

private:
    GLuint programID = 0;

    bool createShaderFromFile(const std::string &filePath,
                              GLuint shaderType,
                              GLuint &shaderID);

    bool linkProgram(GLuint vertexShader, GLuint fragmentShader);

    // Helpers for reading files and checking errors
    bool checkCompileErrors(GLuint shader, const std::string &type);

    bool checkLinkErrors(GLuint program);
};
}


#endif //PROGRAM_H
