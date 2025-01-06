#ifndef MOUSEDRAGEVENT_H
#define MOUSEDRAGEVENT_H
#include "Event.h"

namespace n2m {
class MouseDragEvent : public Event {
public:
    MouseDragEvent (const double& x, const double& y) : x (x), y (y) {
    };

    double x, y;
};
}

#endif //MOUSEDRAGEVENT_H
