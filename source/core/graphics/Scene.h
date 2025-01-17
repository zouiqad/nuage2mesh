#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <memory>
#include "Geometry.h"
#include "Camera.h"
#include "Node.h"

namespace n2m::graphics {
class Scene {
public:
    Scene();

    // clear the scene from all the nodes
    void clear();

    // Add a new node to the scene
    void addNode(const std::shared_ptr<Node> &node);

    void removeNode(const std::shared_ptr<Node> &node);

    // Set a specific geometry to display at the center
    std::shared_ptr<Geometry> getFocusGeometry() const {
        return focusGeometry;
    }

    void setFocusGeometry(const std::shared_ptr<Geometry> &geometry);

    std::shared_ptr<Geometry> getLoadedGeometry() const {
        return loadedGeometry;
    }

    void setLoadedGeometry(const std::shared_ptr<Geometry> &geometry) {
        loadedGeometry = geometry;
    }

    // Get all nodes in the scene
    const std::vector<std::shared_ptr<Node> > &getAllNodes() const;

private:
    std::vector<std::shared_ptr<Node> > nodes;
    std::shared_ptr<Geometry> focusGeometry;
    std::shared_ptr<Geometry> loadedGeometry; // usefull so user can reset
};
} // namespace n2m::graphics

#endif // SCENE_H
