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
        PrimitiveType primitive,
        const std::vector<unsigned int>& indices = {});

    void cleanup () const;
    // Called by your renderer or your Drawable
    void bind () const;
    void unbind () const;

    // The actual draw call
    void draw () const;

protected:
    GLuint vao;
    GLuint vbo;
    GLuint ebo;

    GLsizei vertexCount     = 0;
    GLsizei indexCount      = 0;
    PrimitiveType primitive = PrimitiveType::Points;

    // Internally used for glDrawArrays or glDrawElements
    static GLenum convertPrimitiveType (PrimitiveType type);
};
}


#endif //GEOMETRY_H
