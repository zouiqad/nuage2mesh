//
// Created by zouiqad on 31/12/24.
//

#ifndef ENGINE_H
#define ENGINE_H

#include "graphics/Renderer.h"
#include "io/Window.h"

namespace n2m {
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
};
}


#endif //ENGINE_H
