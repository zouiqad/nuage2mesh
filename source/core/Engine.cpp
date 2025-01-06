#include "Engine.h"
#include "../patterns/singleton/EventDispatcher.h"
#include "io/FileLoader.h"
#include "io/Window.h"
#include <iostream>


namespace n2m {
bool Engine::init () {
    // Init window
    if (!m_window.init (1280, 720, "Nuage2Mesh Viewer")) {
        std::cerr << "Failed to initialize window.\n";
        return false;
    }

    if (!m_renderer.init ()) {
        std::cerr << "Failed to initialize renderer.\n";
        return false;
    }


    // Subscribe to events
    EventDispatcher::Instance ().subscribe<LoadFileEvent> (
        [this](const LoadFileEvent& e) {
            handleLoadFileEvent (e);
        });

    EventDispatcher::Instance ().subscribe<MouseDragEvent> (
        [this](const MouseDragEvent& e) {
            handleMouseDragEvent (e);
        });

    EventDispatcher::Instance ().subscribe<MouseScrollEvent> (
        [this](const MouseScrollEvent& e) {
            handleMouseScrollEvent (e);
        });

    return true;
}

void Engine::run () {
    while (!m_window.shouldClose ()) {
        m_window.render ();

        if (!m_window.pollEvents ()) {
            continue;
        }

        m_renderer.drawFrame ();
    }

    shutdown ();
}

void Engine::shutdown () {
    m_window.shutdown ();
    m_renderer.cleanup ();
}

void Engine::handleLoadFileEvent (const LoadFileEvent& e) {
    m_renderer.getScene ().setFocusGeometry (
        io::FileLoader::loadOBJ (e.filePath));
}

void Engine::handleMouseDragEvent (const MouseDragEvent& e) {
    // Sensitivity factor for rotation
    const float sensitivity = 0.1f;

    // Update rotation angles based on mouse movement
    rotationY += static_cast<float> (e.x) * sensitivity; // Horizontal rotation
    rotationX += static_cast<float> (e.y) * sensitivity; // Vertical rotation


    // Clamp vertical rotation to avoid flipping
    rotationX = glm::clamp (rotationX, -90.0f, 90.0f);

    // Calculate camera position on a virtual arcball
    m_renderer.getCamera ().setPosition (glm::vec3 (
        sin (glm::radians (rotationY)) * cos (glm::radians (rotationX)) * 2.0f,
        sin (glm::radians (rotationX)) * 2.0f,
        cos (glm::radians (rotationY)) * cos (glm::radians (rotationX)) * 2.0f
        ));
}

void Engine::handleMouseScrollEvent (const MouseScrollEvent& e) {
    zoomFactor -= static_cast<float> (e.yoffset) * 0.5f;
    zoomFactor = glm::clamp (zoomFactor, 1.0f, 20.0f);

    m_renderer.getScene ().getAllNodes ()[0]->setScale (glm::vec3 (zoomFactor));
}
}
