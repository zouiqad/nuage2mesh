#include "Node.h"
#include <glm/gtc/matrix_transform.hpp>

namespace n2m::graphics {
Node::Node (std::shared_ptr<Geometry> geometry)
    : geometry (std::move (geometry)) {
}

void Node::draw () const {
    this->geometry->draw ();
}

glm::mat4 Node::getTransformationMatrix () const {
    glm::mat4 mat = glm::mat4 (1.0f);
    mat           = glm::translate (mat, transform.translation);

    mat = glm::rotate (mat, glm::radians (transform.rotation.x),
        {1.0f, 0.0f, 0.0f});
    mat = glm::rotate (mat, glm::radians (transform.rotation.y),
        {0.0f, 1.0f, 0.0f});
    mat = glm::rotate (mat, glm::radians (transform.rotation.z),
        {0.0f, 0.0f, 1.0f});

    mat = glm::scale (mat, transform.scale);
    return mat;
}
}