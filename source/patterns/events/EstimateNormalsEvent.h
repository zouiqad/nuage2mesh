#ifndef ESTIMATENORMALSEVENT_H
#define ESTIMATENORMALSEVENT_H
#include "Event.h"

namespace n2m {
class EstimateNormalsEvent : public Event {
public:
    EstimateNormalsEvent (const int& kSearchNormals) : kSearch (
        kSearchNormals) {
    } ;

    int kSearch = 20;
};
}

#endif //ESTIMATENORMALSEVENT_H
