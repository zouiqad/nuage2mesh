#ifndef EVENTDISPATCHER_H
#define EVENTDISPATCHER_H

#include "patterns/events/Event.h"
#include <functional>
#include <map>
#include <vector>
#include <typeindex>
#include <typeinfo>
#include <memory>

namespace n2m {
class EventDispatcher {
public:
    static EventDispatcher &Instance() {
        static EventDispatcher instance;
        return instance;
    }

    template<typename T>
    void subscribe(std::function<void (const T &)> callback) {
        // Identify event type by std::type_index
        auto typeIndex = std::type_index(typeid(T));

        // We store a std::function<void(const BaseEvent&)> in the map,
        // but we do a little type-erasing trick to wrap the typed callback.
        auto wrapper = [fn = std::move(callback)](const Event &event) {
            // Try to cast the base event to the derived type
            // If it fails, ignore. If it succeeds, call the function.
            const T *derived = dynamic_cast<const T *>(&event);
            if (derived) {
                fn(*derived);
            }
        };

        m_subscribers[typeIndex].push_back(std::move(wrapper));
    }

    // Dispatch any event. We'll figure out which subscribers should get it.
    void publish(const Event &event) {
        auto typeIndex = std::type_index(typeid(event));
        auto it = m_subscribers.find(typeIndex);
        if (it != m_subscribers.end()) {
            for (auto &callback: it->second) {
                callback(event);
            }
        }
    }

private:
    EventDispatcher() = default;

    ~EventDispatcher() = default;

    EventDispatcher(const EventDispatcher &) = delete;

    EventDispatcher &operator=(const EventDispatcher &) = delete;

    // Map from event's type_index to a list of callbacks that handle that event type
    std::map<std::type_index, std::vector<std::function<void
        (const Event &)> > >
    m_subscribers;
};
}


#endif //EVENTDISPATCHER_H
