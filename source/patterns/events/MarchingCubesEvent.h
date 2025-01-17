#ifndef MarchingCubesEvent_H
#define MarchingCubesEvent_H
#include "Event.h"

namespace n2m {
class MarchingCubesEvent : public Event {
public:
    MarchingCubesEvent (const int& gridResolution) : gridResolution (
        gridResolution) {
    };

    int gridResolution;
};
}

#endif //MarchingCubesEvent_H
