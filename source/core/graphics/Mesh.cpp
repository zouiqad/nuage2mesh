#include "Mesh.h"

namespace n2m::graphics {
void Mesh::upload (const std::vector<GLfloat>& vertexData,
    int componentsPerVertex,
    const std::vector<unsigned int>& indices) {
    this->primitive_type = GL_TRIANGLES;
    Geometry::upload (vertexData, componentsPerVertex,
        indices);
}

void Mesh::draw () const {
    Geometry::draw ();
}
} // namespace n2m::graphics
