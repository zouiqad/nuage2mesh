#include "Geometry.h"
#include <iostream>

namespace n2m::graphics {
Geometry::~Geometry () {
    deleteBuffers ();
}

void Geometry::createBuffers () {
    // Generate a VAO
    glGenVertexArrays (1, &VAO);
    // Generate a VBO
    glGenBuffers (1, &VBO);
    // We won't generate EBO here unless needed
}

void Geometry::deleteBuffers () {
    if (EBO != 0) {
        glDeleteBuffers (1, &EBO);
        EBO = 0;
    }
    if (VBO != 0) {
        glDeleteBuffers (1, &VBO);
        VBO = 0;
    }
    if (VAO != 0) {
        glDeleteVertexArrays (1, &VAO);
        VAO = 0;
    }
}

void Geometry::bindVAO () const {
    glBindVertexArray (VAO);
}

void Geometry::unbindVAO () const {
    glBindVertexArray (0);
}

void Geometry::upload (const std::vector<GLfloat>& vertexData,
    int componentsPerVertex,
    const std::vector<unsigned int>& indices) {
    // init vao vbo ebo
    glGenVertexArrays (1, &VAO);
    glGenBuffers (1, &VBO);
    glGenBuffers (1, &EBO);

    std::cout << "uploading vertex data" << vertexData.size () << std::endl;

    // Ensure componentsPerVertex is 3 for glm::vec3 storage
    if (componentsPerVertex != 3) {
        throw std::invalid_argument (
            "upload() only supports 3 components per vertex for glm::vec3 storage.");
    }

    // Convert vertexData to cachedVertices (std::vector<glm::vec3>)
    vertices.clear ();
    for (size_t i = 0; i < vertexData.size (); i += componentsPerVertex) {
        vertices.emplace_back (
            vertexData[i],
            vertexData[i + 1],
            vertexData[i + 2]
            );
    }

    this->indices.clear ();
    this->indices = indices;

    vertexCount = static_cast<int> (vertexData.size () / componentsPerVertex);

    glBindVertexArray (VAO);
    // upload vertex data
    glBindBuffer (GL_ARRAY_BUFFER, VBO);
    glBufferData (GL_ARRAY_BUFFER,
        vertexCount * componentsPerVertex * sizeof (GLfloat),
        vertexData.data (),
        GL_STATIC_DRAW);

    // If we have indices, use them
    if (!indices.empty ()) {
        glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData (GL_ELEMENT_ARRAY_BUFFER,
            indices.size () * sizeof (unsigned int),
            indices.data (),
            GL_STATIC_DRAW);

        indicesCount = static_cast<GLsizei> (indices.size ());
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

void Geometry::draw () const {
    glBindVertexArray (VAO);
    if (indicesCount > 0) {
        glDrawElements (primitive_type, indicesCount, GL_UNSIGNED_INT, nullptr);
    } else {
        glDrawArrays (primitive_type, 0, vertexCount);
    }
    glBindVertexArray (0);
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