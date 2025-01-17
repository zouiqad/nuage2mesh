#ifndef REMOVEOUTLIERSEVENT_H
#define REMOVEOUTLIERSEVENT_H


namespace n2m {
class RemoveOutliersEvent : public Event {
public:
    RemoveOutliersEvent (const int& meanK,
        const float& stddevMul) : meanK (meanK), stddevMul (stddevMul) {
    }

    int meanK        = 50;
    float stddevMul = 1.0;
};
}


#endif //REMOVEOUTLIERSEVENT_H
