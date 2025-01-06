#include "Renderer.h"

#include "../io/FileLoader.h"

#include <glad/glad.h>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

namespace n2m::graphics {
Renderer::Renderer () {
}

Renderer::~Renderer () {
    cleanup ();
}

bool Renderer::init () {
    // 1. Load a basic vertex/fragment shader program
    if (!shader.loadShaders ("resources/shaders/simple.vert",
        "resources/shaders/simple.frag")) {
        // Log error if needed
        return false;
    }

    // 2. Initialize camera
    //    - e.g., perspective at 45 degrees, aspect ratio 16:9, near=0.1, far=100
    camera.setPerspective (45.0f, 16.0 / 9.0f, 0.1f, 100.0f);
    //
    // // Position the camera 5 units along +Z and look at origin
    camera.setPosition (glm::vec3 (0.0f, 0.0f, 2.0f));
    camera.lookAt (glm::vec3 (0.0f, 0.0f, 0.0f));

    // Create a scene with single node (mesh)
    std::shared_ptr<Geometry> geo = io::FileLoader::loadOBJ (
        "resources/models/ceillac-saintantoine.obj");

    // this will create a node in the scene
    scene.setFocusGeometry (geo);

    return true;
}

void Renderer::drawFrame () {
    // 1. Clear screen
    glClearColor (0.1f, 0.1f, 0.2f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 2. Use the default shader program
    shader.use ();

    // Make the camera look at the origin
    glm::mat4 model      = scene.getAllNodes ()[0]->getTransformationMatrix ();
    glm::mat4 view       = camera.getViewMatrix ();
    glm::mat4 projection = camera.getProjectionMatrix ();

    // // 4. Set uniform
    shader.setUniform ("u_model", model);
    shader.setUniform ("u_view", view);
    shader.setUniform ("u_proj", projection);
    //
    // if we want to control point size
    glEnable (GL_PROGRAM_POINT_SIZE);

    shader.setUniform ("u_pointSize", 1.0f);

    // 5. Draw the mesh
    scene.getAllNodes ()[0]->draw ();
}

void Renderer::cleanup () {
    scene.clear ();
}
}
