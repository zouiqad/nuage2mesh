#ifndef FASTTRIANGULATIONEVENT_H
#define FASTTRIANGULATIONEVENT_H
#include "Event.h"

namespace n2m {
struct FastTriangulationParameters {
    float searchRadius = 0.025f; /**< Maximum edge length for triangulation */
    int kSearch = 20; /**< Number of nearest neighbors for normal estimation */
    bool normalConsistency = false; /**< Flag to enforce normal consistency */
};

class FastTriangulationEvent : public Event {
public:
    FastTriangulationEvent() = default;


    FastTriangulationEvent(const FastTriangulationParameters &params)
        : parameters(params) {
    }

    FastTriangulationParameters parameters;
};
}
#endif //FASTTRIANGULATIONEVENT_H
