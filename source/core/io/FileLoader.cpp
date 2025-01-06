#include "FileLoader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "tinyobj/tiny_obj_loader.h"

#include "../graphics/Geometry.h"
#include "tinyfiledialogs/tinyfiledialogs.h"

namespace n2m::io {
std::string FileLoader::readFile (const std::string& filePath) {
    std::ifstream file (filePath, std::ios::in | std::ios::binary);
    if (!file) {
        std::cerr << "Could not open shader file: " << filePath << std::endl;
        return {};
    }
    std::stringstream buffer;
    buffer << file.rdbuf ();
    return buffer.str ();
}

std::shared_ptr<graphics::Geometry> FileLoader::loadOBJ (
    const std::string& filepath) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn;
    std::string err;

    // Load OBJ file
    bool ret = tinyobj::LoadObj (&attrib, &shapes, &materials, &warn, &err,
        filepath.c_str ());

    if (!warn.empty ()) {
        std::cout << "TinyObjLoader Warning: " << warn << std::endl;
    }

    if (!err.empty ()) {
        std::cerr << "TinyObjLoader Error: " << err << std::endl;
    }

    if (!ret) {
        std::cerr << "Failed to load OBJ file: " << filepath << std::endl;
        return nullptr;
    }

    // Create Geometry object
    auto geometry = std::make_shared<graphics::Geometry> ();

    std::vector<GLfloat> vertices;
    // Since we're loading as a point cloud, we can ignore indices
    // and just store vertex positions
    vertices.reserve (attrib.vertices.size ());

    // Extract vertex positions
    for (size_t v = 0; v < attrib.vertices.size () / 3; v++) {
        float x = attrib.vertices[3 * v + 0];
        float y = attrib.vertices[3 * v + 1];
        float z = attrib.vertices[3 * v + 2];

        vertices.push_back (x); // X
        vertices.push_back (y); // Y
        vertices.push_back (z); // Z

        geometry->setExtents (x, y, z); // Update extents
    }

    geometry->upload (attrib.vertices, 3);

    return geometry;
}

std::string FileLoader::openFileDialog () {
    char const* lFilterPatterns[1] = {"*.obj"};

    // there is also a wchar_t version
    std::string selection = tinyfd_openFileDialog (
        "Select file", // title
        "",            // optional initial directory
        1,             // number of filter patterns
        lFilterPatterns,
        // char const * lFilterPatterns[2] = { "*.txt", "*.jpg" };
        NULL, // optional filter description
        0     // forbids multiple selections
        );

    std::cout << "Selected file: " << selection << std::endl;
    return selection;
}
}