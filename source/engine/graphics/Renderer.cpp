#include "Renderer.h"
#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

namespace n2m::graphics {
Renderer::Renderer () {
    // Constructor body, if needed
}

Renderer::~Renderer () {
    cleanup ();
}

bool Renderer::init () {
    // 1. Load a basic vertex/fragment shader program
    if (!defaultProgram.loadShaders ("resources/shaders/simple.vert",
        "resources/shaders/simple.frag")) {
        // Log error if needed
        return false;
    }

    // 2. Initialize camera
    //    - e.g., perspective at 45 degrees, aspect ratio 16:9, near=0.1, far=100
    camera.setPerspective (45.0f, 4.0f / 3.0f, 0.1f, 100.0f);
    //
    // // Position the camera 5 units along +Z and look at origin
    camera.setPosition (glm::vec3 (0.0f, 0.0f, 3.0f));
    camera.lookAt (glm::vec3 (0.0f, 0.0f, 0.0f));

    // 3. Create a test mesh (for demonstration, a simple cube)
    testGeo = createPointCloudCube (100);

    return true;
}

void Renderer::drawFrame (float rotationX, float rotationY, float zoomFactor) {
    // 1. Clear screen
    glClearColor (0.1f, 0.1f, 0.2f, 1.0f);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 2. Use the default shader program
    defaultProgram.use ();
    std::cout << "zoom factor: " << zoomFactor << std::endl;

    glm::mat4 model = glm::mat4 (1.0f); // Identity matrix
    model           = glm::scale (model, glm::vec3 (zoomFactor));
    // Calculate camera position on a virtual arcball
    camera.setPosition (glm::vec3 (
        sin (glm::radians (rotationY)) * cos (glm::radians (rotationX)) * 5.0f,
        sin (glm::radians (rotationX)) * 5.0f,
        cos (glm::radians (rotationY)) * cos (glm::radians (rotationX)) * 5.0f
        ));

    // Make the camera look at the origin
    glm::mat4 view = camera.getViewMatrix ();
    glm::mat4 proj = camera.getProjectionMatrix ();

    // glm::mat4 proj = glm::mat4 (1.0f);

    //
    // // 4. Set uniform
    defaultProgram.setUniform ("u_model", model);
    defaultProgram.setUniform ("u_view", view);
    defaultProgram.setUniform ("u_proj", proj);
    //
    // if we want to control point size
    glEnable (GL_PROGRAM_POINT_SIZE);

    defaultProgram.setUniform ("u_pointSize", 2.0f);

    // 5. Draw the mesh
    testGeo.draw ();
}

void Renderer::cleanup () {
    // If Program or Mesh store GPU resources,
    // their destructors or explicit methods will handle cleanup.
    // e.g., defaultProgram.~Program(); testMesh.cleanup();
}

Geometry Renderer::createTriangleDemo () const {
    Geometry geo;

    std::vector<GLfloat> vertices = {
        -0.5f, -0.5f, 0.0f,
        0.5f, -0.5f, 0.0f,
        0.0f, 0.5f, 0.0f
    };

    geo.upload (vertices, 3, PrimitiveType::Points);

    return geo;
}

// Example: create a simple unit cube with positions (and optional normals/UVs)
Geometry Renderer::createPointCloudCube (const int Nx)
const {
    Geometry geo;

    std::cout << "creating point cloud cube points N#" << Nx * Nx * Nx <<
        std::endl;
    // We want Nx^3 = 1e6 => Nx = 100
    // We'll distribute points in [-1, 1] along each axis
    // step = (2.0) / (Nx - 1) so that Nx points go from -1 to +1 inclusive
    const float step = 2.0f / static_cast<float> (Nx - 1);

    // // Reserve enough space: 3 floats per point => 3 million floats
    std::vector<float> vertices;
    vertices.reserve (static_cast<size_t> (Nx) * Nx * Nx * 3);


    // std::vector<float> vertices = {
    //     0.0f, 0.0f, 0.0f,
    //     0.5f, -0.5f, 0.0f,
    //     0.0f, 0.5f, 0.0f
    // };
    //
    // Generate the points
    for (int i = 0; i < Nx; ++i) {
        for (int j = 0; j < Nx; ++j) {
            for (int k = 0; k < Nx; ++k) {
                float x = i * step;
                float y = j * step;
                float z = k * step;
                vertices.push_back (x);
                vertices.push_back (y);
                vertices.push_back (z);
            }
        }
    }

    // Now upload these points to our Geometry object
    geo.upload (vertices, 3, PrimitiveType::Points);

    return geo;
}
}
