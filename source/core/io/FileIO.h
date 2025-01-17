#ifndef FILEIO_H
#define FILEIO_H

#define TINYOBJLOADER_IMPLEMENTATION

#include <memory>
#include <string>
#include "core/graphics/Geometry.h"
#include "core/graphics/PointCloud.h"
#include "core/graphics/Mesh.h"


namespace n2m::io {
class FileIO {
public:
    FileIO () = default;

    ~FileIO () = default;

    static std::string readFile (const std::string& filePath);

    static std::shared_ptr<graphics::PointCloud> loadOBJ (
        const std::string& filepath);

    static bool exportOBJ (const std::string& filepath,
        const std::vector<glm::vec3>& vertices,
        const std::vector<unsigned int>& indices,
        const std::vector<glm::vec3>& normals = {});

    static bool exportSTL (const std::string& filepath,
        const std::vector<glm::vec3>& vertices,
        const std::vector<unsigned int>& indices,
        const std::vector<glm::vec3>& normals = {});

    static std::string openFileDialog ();
};
}

#endif
