#ifndef FILELOADER_H
#define FILELOADER_H
#define TINYOBJLOADER_IMPLEMENTATION

#include <memory>
#include <string>


// forward declare
namespace n2m::graphics {
class Geometry;
}

namespace n2m::io {
class FileLoader {
public:
    FileLoader ()  = default;
    ~FileLoader () = default;

    static std::string readFile (const std::string& filePath);
    static std::shared_ptr<graphics::Geometry> loadOBJ (
        const std::string& filepath);

    static std::string openFileDialog ();
};
}


#endif //FILELOADER_H
