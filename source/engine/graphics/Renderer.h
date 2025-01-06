#ifndef RENDERER_H
#define RENDERER_H

#include "Camera.h"
#include "Program.h"
#include "Geometry.h"

namespace n2m::graphics {
class Renderer {
public:
    Renderer ();
    ~Renderer ();

    bool init ();
    // Loads shaders, initializes camera, maybe sets up default meshes
    void drawFrame (float rotationX, float rotationY, float zoomFactor);
    // Called every frame to render the scene
    void cleanup (); // Frees GPU resources if needed

    // Access camera for movement or configuration
    Camera& getCamera () { return camera; }
    Program defaultProgram; // A basic shader program

private:
    Camera camera;    // Our camera (view/projection)
    Geometry testGeo; // Example of geometry

    // For demonstration: a method to create a simple test mesh (e.g., a cube)
    Geometry createPointCloudCube (int Nx = 10) const;
    Geometry createTriangleDemo () const;
};
}
#endif // RENDERER_H
