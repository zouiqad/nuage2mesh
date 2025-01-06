#ifndef MOUSESCROLLEVENT_H
#define MOUSESCROLLEVENT_H

namespace n2m {
class MouseScrollEvent : public Event {
public:
    MouseScrollEvent (const double& xoffset,
        const double& yoffset) : xoffset (xoffset), yoffset (yoffset) {
    }

    double xoffset, yoffset;
};
}

#endif //MOUSESCROLLEVENT_H
