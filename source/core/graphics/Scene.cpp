#include "Scene.h"

#include "patterns/singleton/EventDispatcher.h"
#include "patterns/events/SceneStateEvent.h"


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

    SceneStateEvent::SceneMetrics metrics;
    metrics.vertexCount   = geometry->vertexCount;
    metrics.triangleCount = geometry->indicesCount / 3.0f;
    EventDispatcher::Instance ().publish (SceneStateEvent (metrics));
}

const std::vector<std::shared_ptr<Node> >& Scene::getAllNodes () const {
    return nodes;
}

// @todo add per scene render
// void Scene::render () {
//     // Loop through nodes and render associated geometry
//     for (const auto& node : nodes) {
//         if (auto geo = node->getGeometry (); geo) {
//             Geometry& geometry = *geo; // Retrieve the geometry
//
//             // Apply node's transformation matrix
//             glm::mat4 modelMatrix = node->getTransformationMatrix ();
//
//             // Set uniform
//             // shader.setUniform("u_model", modelMatrix);
//             // geometry.draw ();
//         }
//     }
// }
}