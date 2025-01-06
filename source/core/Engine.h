//
// Created by zouiqad on 31/12/24.
//

#ifndef ENGINE_H
#define ENGINE_H

#include "graphics/Renderer.h"
#include "io/Window.h"
#include "../patterns/events/LoadFileEvent.h"
#include "../patterns/events/MouseDragEvent.h"
#include "../patterns/events/MouseScrollEvent.h"
#include "../patterns/events/ApplyMarchingCubesEvent.h"


namespace n2m {
// forward dec
class UIEvent;

class Engine {
public:
    Engine ()  = default;
    ~Engine () = default;

    bool init ();
    void run ();
    void shutdown ();

private:
    io::Window m_window;           // main window & gl context
    graphics::Renderer m_renderer; // main renderer

    float zoomFactor = 1.0f;

    // Track rotation variables
    float rotationX = 0.0f; // Vertical rotation (pitch)
    float rotationY = 0.0f; // Horizontal rotation (yaw

    void handleLoadFileEvent (const LoadFileEvent& e);
    void handleMouseDragEvent (const MouseDragEvent& e);
    void handleMouseScrollEvent (const MouseScrollEvent& e);
    void handleApplyMarchingCubes (const ApplyMarchingCubesEvent& e);
};
}


#endif //ENGINE_H
