#include "Scene.h"
#include <glm/gtc/matrix_transform.hpp>

namespace n2m::graphics {
// ---------------- Scene Implementation -----------------
Scene::Scene () : focusGeometry (nullptr) {
}

void Scene::clear () {
    nodes.clear ();
}

void Scene::addNode (const std::shared_ptr<Node>& node) {
    nodes.push_back (node);
}

void Scene::removeNode (const std::shared_ptr<Node>& node) {
    std::erase (nodes, node);
}

void Scene::setFocusGeometry (const std::shared_ptr<Geometry>& geometry) {
    this->clear ();
    focusGeometry = geometry;

    if (focusGeometry) {
        // Automatically center the geometry
        auto center = focusGeometry->getCenterOfMass ();
        auto node   = std::make_shared<Node> (focusGeometry);
        node->setTranslation (-center); // Offset by the center
        addNode (node);
    }
}

const std::vector<std::shared_ptr<Node> >& Scene::getAllNodes () const {
    return nodes;
}

// void Scene::render () {
//     // Loop through nodes and render associated geometry
//     for (const auto& node : nodes) {
//         if (auto geo = node->getGeometry (); geo) {
//             Geometry& geometry = *geo; // Retrieve the geometry
//
//             // Apply node's transformation matrix
//             glm::mat4 modelMatrix = node->getTransformationMatrix ();
//
//             // // 4. Set uniform
//             // YourShader.setUniform("u_model", modelMatrix);
//             // geometry.draw ();
//         }
//     }
// }
} // namespace n2m::graphics