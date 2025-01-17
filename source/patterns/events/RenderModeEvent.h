#ifndef RENDERMODEEVENT_H
#define RENDERMODEEVENT_H
#include "Event.h"

namespace n2m {
class RenderModeEvent : public Event {
public:
    enum class renderMode {
        Solid,
        Wireframe,
        Count
    };


    static const std::unordered_map<renderMode, std::string>& renderModeToStringMap() {
        static const std::unordered_map<renderMode, std::string> map = {
            { renderMode::Wireframe, "Wireframe" },
            { renderMode::Solid, "Solid" }
        };
        return map;
    }

    static renderMode fromString(const std::string& modeStr) {
        const auto& map = renderModeToStringMap();
        for (const auto& [key, value] : map) {
            if (value == modeStr) {
                return key;
            }
        }
    }

    RenderModeEvent(renderMode mode) : render_mode(mode) {
    }

    RenderModeEvent(const std::string& mode)
    : render_mode(fromString(mode)) {}

    renderMode render_mode = renderMode::Solid;
};
}
#endif //RENDERMODEEVENT_H
