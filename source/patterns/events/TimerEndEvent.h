#ifndef TIMERENDEVENT_H
#define TIMERENDEVENT_H

#include "Event.h"

namespace n2m {
class TimerEndEvent : public Event {
public:
    TimerEndEvent (float dur) : duration (dur) {
    };

    float duration;
};
}
#endif
