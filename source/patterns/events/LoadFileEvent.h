#ifndef LOADFILEEVENT_H
#define LOADFILEEVENT_H
#include "Event.h"

#include <string>
#include <utility>

namespace n2m {
class LoadFileEvent : public Event {
public:
    LoadFileEvent (std::string filePath) : filePath (filePath) {
    };

    std::string filePath;
};
}

#endif //LOADFILEEVENT_H
