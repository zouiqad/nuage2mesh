#include "Geometry.h"
#include <iostream>

namespace n2m::graphics {
Geometry::Geometry () {
}

Geometry::~Geometry () {
    std::cout << "Geometry destroyed!!!.\n";
}

void Geometry::cleanup () const {
    if (ebo)
        glDeleteBuffers (1, &ebo);
    if (vbo)
        glDeleteBuffers (1, &vbo);
    if (vao)
        glDeleteVertexArrays (1, &vao);
}

void Geometry::upload (const std::vector<GLfloat>& vertexData,
    int componentsPerVertex,
    const std::vector<unsigned int>& indices) {
    // init vao vbo ebo
    glGenVertexArrays (1, &vao);
    glGenBuffers (1, &vbo);
    glGenBuffers (1, &ebo);

    vertexCount = static_cast<int> (vertexData.size () / componentsPerVertex);

    glBindVertexArray (vao);

    // upload vertex data
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glBufferData (GL_ARRAY_BUFFER,
        vertexCount * componentsPerVertex * sizeof (GLfloat),
        vertexData.data (),
        GL_STATIC_DRAW);

    // If we have indices, use them
    if (!indices.empty ()) {
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData (GL_ELEMENT_ARRAY_BUFFER,
            indices.size () * sizeof (unsigned int),
            indices.data (),
            GL_STATIC_DRAW);

        indexCount = static_cast<GLsizei> (indices.size ());
    }

    // Setup vertex attribute (location = 0 for positions)
    glVertexAttribPointer (0,
        componentsPerVertex,
        GL_FLOAT,
        GL_FALSE,
        componentsPerVertex * sizeof (GLfloat),
        (void*)0
        );
    glEnableVertexAttribArray (0);

    // Unbind
    glBindVertexArray (0);
    glBindBuffer (GL_ARRAY_BUFFER, 0);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Geometry::draw (const PrimitiveType& type) const {
    GLenum mode = convertPrimitiveType (type);

    glBindVertexArray (vao);
    if (indexCount > 0) {
        glDrawElements (mode, indexCount, GL_UNSIGNED_INT, nullptr);
    } else {
        glDrawArrays (mode, 0, vertexCount);
    }
    glBindVertexArray (0);
}


GLenum Geometry::convertPrimitiveType (const PrimitiveType type) {
    switch (type) {
    case PrimitiveType::Points: return GL_POINTS;
    case PrimitiveType::Lines: return GL_LINES;
    case PrimitiveType::Triangles: return GL_TRIANGLES;
    }
    // default
    return GL_TRIANGLES;
}

// Metrics
glm::vec3 Geometry::getCenterOfMass () const {
    return glm::vec3 ((this->maxX + this->minX) / 2,
        (this->maxY + this->minY) / 2, (this->maxZ + this->minZ) / 2);
}

void Geometry::setExtents (const GLfloat& x,
    const GLfloat& y,
    const GLfloat& z) {
    if (x > this->maxX) maxX = x;
    if (y > this->maxY) maxY = y;
    if (z > this->maxZ) maxZ = z;

    if (x < this->minX) minX = x;
    if (y < this->minY) minY = y;
    if (z < this->minZ) minZ = z;
}
}