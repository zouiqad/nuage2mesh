#ifndef EVENT_H
#define EVENT_H
#include <string>

// enum class EventType {
//     WindowResize,
//     WindowClose,
//     MouseEvent,
//     KeyPress,
//     UIEvent,
// };
namespace n2m {
class Event {
public:
    virtual ~Event () = default;
};
}

#endif //EVENT_H
