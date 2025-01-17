#ifndef SCENESTATEEVENT_H
#define SCENESTATEEVENT_H
#include "Event.h"


namespace n2m {
class SceneStateEvent : public Event {
public:
    struct SceneMetrics {
        size_t vertexCount;
        size_t triangleCount;
    };

    SceneStateEvent(const SceneMetrics& scene_metrics) : metrics(scene_metrics) {}

    SceneMetrics metrics;
};
}
#endif //SCENESTATEEVENT_H
