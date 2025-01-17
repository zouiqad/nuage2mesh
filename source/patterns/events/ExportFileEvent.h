#ifndef EXPORTFILEEVENT_H
#define EXPORTFILEEVENT_H

#include "Event.h"

#include <string>
#include <vector>
#include <glm/vec3.hpp>

namespace n2m {
enum class ExportFormat {
    OBJ,
    STL
};


class ExportFileEvent : public Event {
public:
    ExportFileEvent(const std::string &filePath,
                    ExportFormat format)
        : filePath(filePath), format(format) {
    }

    std::string filePath;

    ExportFormat format;
};
} // namespace n2m

#endif // EXPORTFILEEVENT_H
