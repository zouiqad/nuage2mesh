#ifndef APPLYMARCHINGCUBESEVENT_H
#define APPLYMARCHINGCUBESEVENT_H
#include "Event.h"

namespace n2m {
class ApplyMarchingCubesEvent : public Event {
public:
    ApplyMarchingCubesEvent (const int& gridResolution) : gridResolution (
        gridResolution) {
    };

    int gridResolution;
};
}

#endif //APPLYMARCHINGCUBESEVENT_H
