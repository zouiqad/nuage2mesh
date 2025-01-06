#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace n2m::graphics {
enum class PrimitiveType {
    Points,
    Lines,
    Triangles
};

class Geometry {
public:
    Geometry ();
    virtual ~Geometry ();


    // Upload data to the GPU. Overloads or templated methods are possible.
    // 'indices' can be empty if we are drawing non-indexed geometry (like a pure point cloud).
    void upload (const std::vector<GLfloat>& vertices,
        int componentsPerVertex,
        PrimitiveType type,
        const std::vector<unsigned int>& indices = {});

    void cleanup () const;
    // Called by your renderer or your Drawable
    void bind () const;
    void unbind () const;

    // The actual draw call
    void draw () const;

    // Geometry metrics
    glm::vec3 getCenterOfMass () const;

    void setExtents (const GLfloat& x,
        const GLfloat& y,
        const GLfloat& z);

    const std::vector<glm::vec3>& getVertices () const {
        return vertices;
    }

protected:
    GLuint vao;
    GLuint vbo;
    GLuint ebo;

    std::vector<glm::vec3> vertices;

    PrimitiveType type;
    // Internally used for glDrawArrays or glDrawElements
    static GLenum convertPrimitiveType (PrimitiveType type);

    GLsizei vertexCount = 0;
    GLsizei indexCount  = 0;

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
