#include "Renderer.h"


namespace n2m::graphics {
Renderer::Renderer () {
}

Renderer::~Renderer () {
    cleanup ();
}

bool Renderer::init () {
    // Load a basic vertex/fragment shader program
    if (!shader.loadShaders ("resources/shaders/simple.vert",
        "resources/shaders/simple.frag")) {
        // Log error if needed
        return false;
    }

    // Initialize camera
    camera.setPerspective (45.0f, 16.0 / 9.0f, 0.1f, 100.0f);
    camera.setPosition (glm::vec3 (0.0f, 0.0f, 2.0f));
    camera.lookAt (glm::vec3 (0.0f, 0.0f, 0.0f));

    return true;
}

void Renderer::drawFrame () {
    glClearColor (0.1f, 0.1f, 0.2f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shader.use ();

    if (scene.getFocusGeometry () == nullptr) return;

    // Make the camera look at the origin
    glm::mat4 model      = scene.getAllNodes ()[0]->getTransformationMatrix ();
    glm::mat4 view       = camera.getViewMatrix ();
    glm::mat4 projection = camera.getProjectionMatrix ();

    // Set uniform
    shader.setUniform ("u_model", model);
    shader.setUniform ("u_view", view);
    shader.setUniform ("u_proj", projection);


    // if we want to control point size
    glEnable (GL_PROGRAM_POINT_SIZE);

    shader.setUniform ("u_pointSize", 1.0f);

    // 5. Draw the mesh
    scene.getFocusGeometry ()->draw ();
}

void Renderer::cleanup () {
    scene.clear ();
}
}
