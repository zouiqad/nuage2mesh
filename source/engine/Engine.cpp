//
// Created by zouiqad on 31/12/24.
//

#include "Engine.h"
#include "io/Window.h"
#include <iostream>
#include <filesystem>
#include <glm/ext/matrix_transform.hpp>

namespace n2m {
bool Engine::init () {
    // Init window
    if (!m_window.init (1280, 720, "My OpenGL App")) {
        std::cerr << "Failed to initialize window.\n";
        return false;
    }

    if (!m_renderer.init ()) {
        std::cerr << "Failed to initialize renderer.\n";
        return false;
    }

    return true;
}

void Engine::run () {
    while (!m_window.shouldClose ()) {
        m_window.render ();

        if (!m_window.pollEvents ()) {
            continue;
        }

        m_renderer.drawFrame (m_window.rotationX, m_window.rotationY,
            m_window.zoomFactor);
    }

    shutdown ();
}

void Engine::shutdown () {
    m_window.shutdown ();
    // m_renderer.cleanup ();
}
}
