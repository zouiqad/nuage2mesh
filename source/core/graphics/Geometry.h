#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace n2m::graphics {
class Geometry {
public:
    Geometry() = default;

    virtual ~Geometry();


    /**
     * Each derived class can internally call the protected "internal..."
     * methods if it wants to reuse the logic.
     */
    virtual void upload(const std::vector<GLfloat> &vertexData,
                        int componentsPerVertex,
                        const std::vector<unsigned int> &indices = {}) = 0;

    virtual void draw() const = 0;

    // Called by your renderer or your Drawable
    void bind() const;

    void unbind() const;

    // Geometry metrics
    glm::vec3 getCenterOfMass() const;

    void setExtents(const GLfloat &x,
                    const GLfloat &y,
                    const GLfloat &z);

    GLsizei vertexCount = 0;
    GLsizei indicesCount = 0;

    const std::vector<glm::vec3> &getVertices() const { return vertices; }
    const std::vector<unsigned int> &getIndices() const { return indices; }
    const std::vector<glm::vec3> &getNormals() const { return normals; }

protected:
    GLuint VAO;
    GLuint VBO;
    GLuint EBO;

    std::vector<glm::vec3> vertices;
    std::vector<unsigned int> indices;
    std::vector<glm::vec3> normals;

    GLenum primitive_type = GL_POINTS;

    // Helper: create, bind, or delete buffers.
    void createBuffers();

    void deleteBuffers();

    void bindVAO() const;

    void unbindVAO() const;

    // Geometry bounds
    GLfloat maxX = 0.0f;
    GLfloat maxY = 0.0f;
    GLfloat maxZ = 0.0f;

    GLfloat minX = 0.0f;
    GLfloat minY = 0.0f;
    GLfloat minZ = 0.0f;
};
}


#endif //GEOMETRY_H
