#include "FileIO.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "tinyfiledialogs/tinyfiledialogs.h"
#include "tinyobj/tiny_obj_loader.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>


namespace n2m::io {
std::string FileIO::readFile (const std::string& filePath) {
    std::ifstream file (filePath, std::ios::in | std::ios::binary);
    if (!file) {
        std::cerr << "Could not open shader file: " << filePath << std::endl;
        return {};
    }
    std::stringstream buffer;
    buffer << file.rdbuf ();
    return buffer.str ();
}

std::shared_ptr<graphics::PointCloud> FileIO::loadOBJ (
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
    auto cloud = std::make_shared<graphics::PointCloud> ();

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

        cloud->setExtents (x, y, z); // Update extents
    }

    cloud->upload (attrib.vertices, 3);

    return cloud;
}

bool FileIO::exportOBJ (const std::string& filepath,
    const std::vector<glm::vec3>& vertices,
    const std::vector<unsigned int>& indices,
    const std::vector<glm::vec3>& normals) {
    // Validate input
    if (vertices.empty ()) {
        std::cerr << "No vertices provided for OBJ export." << std::endl;
        return false;
    }

    if (!indices.empty () && (indices.size () % 3 != 0)) {
        std::cerr <<
            "Number of indices is not a multiple of 3. Cannot form triangles for OBJ export."
            << std::endl;
        return false;
    }

    std::ofstream outFile (filepath);
    if (!outFile.is_open ()) {
        std::cerr << "Failed to open file for writing: " << filepath <<
            std::endl;
        return false;
    }

    // Write OBJ Header
    outFile << "# Exported by FileIO\n";

    // Write vertices
    for (const auto& vertex : vertices) {
        outFile << "v " << vertex.x << " " << vertex.y << " " << vertex.z <<
            "\n";
    }

    // Write normals if provided
    if (!normals.empty ()) {
        for (const auto& normal : normals) {
            outFile << "vn " << normal.x << " " << normal.y << " " << normal.z
                << "\n";
        }
    }

    // Write faces (triangles)
    if (!indices.empty ()) {
        // Assuming one normal per face or per vertex
        bool hasNormals = !normals.empty ();
        for (size_t i = 0; i < indices.size (); i += 3) {
            // OBJ indices start at 1
            unsigned int idx1 = indices[i] + 1;
            unsigned int idx2 = indices[i + 1] + 1;
            unsigned int idx3 = indices[i + 2] + 1;

            if (hasNormals) {
                // Assuming normals correspond to vertices
                outFile << "f "
                    << idx1 << "//" << idx1 << " "
                    << idx2 << "//" << idx2 << " "
                    << idx3 << "//" << idx3 << "\n";
            } else {
                outFile << "f " << idx1 << " " << idx2 << " " << idx3 << "\n";
            }
        }
    } else {
        // If no indices provided, export as points
        for (size_t i = 1; i <= vertices.size (); ++i) {
            // OBJ indices start at 1
            outFile << "p " << i;
            if (!normals.empty ()) {
                outFile << "//" << i; // Assuming one normal per vertex
            }
            outFile << "\n";
        }
    }

    outFile.close ();
    std::cout << "Successfully exported OBJ file: " << filepath << std::endl;
    return true;
}

bool FileIO::exportSTL (const std::string& filepath,
    const std::vector<glm::vec3>& vertices,
    const std::vector<unsigned int>& indices,
    const std::vector<glm::vec3>& normals) {
    // STL format requires triangular facets. Thus, you need to define triangles.
    // This implementation uses binary STL.

    if (indices.empty ()) {
        std::cerr <<
            "No indices provided for STL export. Cannot form triangles." <<
            std::endl;
        return false;
    }

    if (indices.size () % 3 != 0) {
        std::cerr <<
            "Number of indices is not a multiple of 3. Cannot form complete triangles for STL export."
            << std::endl;
        return false;
    }

    std::ofstream outFile (filepath, std::ios::binary);
    if (!outFile.is_open ()) {
        std::cerr << "Failed to open file for writing: " << filepath <<
            std::endl;
        return false;
    }

    // Write STL Header (80 bytes)
    std::string header = "Exported by FileIO";
    header.resize (80, ' ');
    outFile.write (header.c_str (), 80);

    // Number of triangles
    uint32_t numTriangles = static_cast<uint32_t> (indices.size () / 3);
    outFile.write (reinterpret_cast<char*> (&numTriangles), 4);

    // Write each triangle
    for (size_t i = 0; i < indices.size (); i += 3) {
        glm::vec3 normal (0.0f, 0.0f, 0.0f);
        if (!normals.empty ()) {
            // If normals are provided per vertex, compute face normal
            glm::vec3 v0    = vertices[indices[i]];
            glm::vec3 v1    = vertices[indices[i + 1]];
            glm::vec3 v2    = vertices[indices[i + 2]];
            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            normal          = glm::normalize (glm::cross (edge1, edge2));
        }

        // Write normal vector
        outFile.write (reinterpret_cast<char*> (&normal.x), 4);
        outFile.write (reinterpret_cast<char*> (&normal.y), 4);
        outFile.write (reinterpret_cast<char*> (&normal.z), 4);

        // Write vertices of the triangle
        for (int j = 0; j < 3; ++j) {
            const glm::vec3& vertex = vertices[indices[i + j]];
            float vertexArray[3]    = {vertex.x, vertex.y, vertex.z};
            outFile.write (reinterpret_cast<char*> (vertexArray), 12);
            // 3 floats
        }

        // Attribute byte count (set to 0)
        uint16_t attributeByteCount = 0;
        outFile.write (reinterpret_cast<char*> (&attributeByteCount), 2);
    }

    outFile.close ();
    std::cout << "Successfully exported STL file: " << filepath << std::endl;
    return true;
}

std::string FileIO::openFileDialog () {
    // Define supported file extensions
    const char* filterPatterns[] = {"*.pcd", "*.ply", "*.obj"};
    const int numFilters = sizeof(filterPatterns) / sizeof(filterPatterns[0]);

    // Open the file dialog
    std::string selection = tinyfd_openFileDialog (
        "Select Point Cloud File", // title
        "",                        // optional initial directory
        numFilters,                // number of filter patterns
        filterPatterns,            // filter patterns
        "Point Cloud Files",       // optional filter description
        0                          // forbids multiple selections
        );

    if (!selection.empty ()) {
        std::cout << "Selected file: " << selection << std::endl;
    } else {
        std::cout << "No file selected." << std::endl;
    }

    return selection;
}
}
