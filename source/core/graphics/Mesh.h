#ifndef MESH_H
#define MESH_H

#include "Geometry.h"

namespace n2m::graphics {
class Mesh : public Geometry {
public:
    Mesh () = default;

    virtual ~Mesh () = default;


    void upload (const std::vector<GLfloat>& vertexData,
        int componentsPerVertex,
        const std::vector<unsigned int>& indices = {}) override;

    void draw () const override;
};
} // namespace n2m::graphics

#endif
