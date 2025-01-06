#ifndef RENDERER_H
#define RENDERER_H

#include "Camera.h"
#include "Shader.h"
#include "Scene.h"

namespace n2m::graphics {
class Renderer {
public:
    Renderer ();
    ~Renderer ();

    bool init ();
    // Loads shaders, initializes camera, maybe sets up default meshes
    void drawFrame ();
    // Called every frame to render the scene
    void cleanup (); // Frees GPU resources if needed

    // Getters & setters
    void setScene (const Scene& scene) { this->scene = scene; }
    Scene& getScene () { return scene; }

    void setShader (const Shader& shader) { this->shader = shader; }
    Shader& getShader () { return shader; }

    void setCamera (const Camera& camera) { this->camera = camera; }
    Camera& getCamera () { return camera; }

private:
    Shader shader;
    Camera camera;
    Scene scene;
};
}
#endif // RENDERER_H
