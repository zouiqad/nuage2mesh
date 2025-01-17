#include "Engine.h"

#include "algorithms/FastTriangulation.h"
#include "patterns/events/MarchingCubesEvent.h"
#include "patterns/singleton/EventDispatcher.h"
#include "io/FileIO.h"
#include "io/Window.h"
#include "algorithms/MarchingCubes.h"
#include "algorithms/Poisson.h"
#include "patterns/events/PoissonEvent.h"

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

    EventDispatcher::Instance ().subscribe<RenderModeEvent> (
        [this](const RenderModeEvent& e) {
            handleRenderModeEvent (e);
        });

    EventDispatcher::Instance ().subscribe<ExportFileEvent> (
        [this](const ExportFileEvent& e) {
            handleExportFileEvent (e);
        });

    EventDispatcher::Instance ().subscribe<ResetGeometryEvent> (
        [this](const ResetGeometryEvent& e) {
            handleResetGeometryEvent (e);
        });

    EventDispatcher::Instance ().subscribe<MarchingCubesEvent> (
        [this](const MarchingCubesEvent& e) {
            handleMarchingCubesEvent (e);
        });

    EventDispatcher::Instance ().subscribe<FastTriangulationEvent> (
        [this](const FastTriangulationEvent& e) {
            handleFastTriangulationEvent (e);
        });

    EventDispatcher::Instance ().subscribe<PoissonEvent> (
        [this](const PoissonEvent& e) {
            handlePoissonEvent (e);
        });

    EventDispatcher::Instance ().subscribe<RemoveOutliersEvent> (
        [this](const RemoveOutliersEvent& e) {
            auto geometry = m_renderer.getScene ().getFocusGeometry ();
            auto cloud    = std::dynamic_pointer_cast<graphics::PointCloud> (
                geometry);

            cloud->removeOutliers (e.meanK, e.stddevMul);
        });

    EventDispatcher::Instance ().subscribe<EstimateNormalsEvent> (
        [this](const EstimateNormalsEvent& e) {
            auto geometry = m_renderer.getScene ().getFocusGeometry ();
            auto cloud    = std::dynamic_pointer_cast<graphics::PointCloud> (
                geometry);

            cloud->estimateNormals (e.kSearch);
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
    std::shared_ptr<graphics::PointCloud> cloud = io::FileIO::loadOBJ (
        e.filePath);

    m_renderer.getScene ().setFocusGeometry (cloud);
    m_renderer.getScene ().setLoadedGeometry (cloud);
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

void Engine::handleMarchingCubesEvent (const MarchingCubesEvent& e) {
    m_renderer.getScene ().setFocusGeometry (

        graphics::MarchingCubes::reconstruct (
            m_renderer.getScene ().getFocusGeometry ()->getVertices ()
            , e.gridResolution));
}

void Engine::handleFastTriangulationEvent (const FastTriangulationEvent& e) {
    auto geometry = m_renderer.getScene ().getFocusGeometry ();
    auto cloud    = std::dynamic_pointer_cast<graphics::PointCloud> (geometry);

    m_renderer.getScene ().setFocusGeometry (
        graphics::FastTriangulation::reconstruct (cloud
            , e.parameters));
}

void Engine::handlePoissonEvent (const PoissonEvent& e) {
    auto geometry = m_renderer.getScene ().getFocusGeometry ();
    auto cloud    = std::dynamic_pointer_cast<graphics::PointCloud> (geometry);

    m_renderer.getScene ().setFocusGeometry (
        graphics::Poisson::reconstruct (cloud
            , e.parameters));
}

void Engine::handleRenderModeEvent (const RenderModeEvent& e) {
    switch (e.render_mode) {
    case RenderModeEvent::renderMode::Wireframe:
        glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
        break;
    case RenderModeEvent::renderMode::Solid:
        glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
        break;
    default: break;
    }
}

void Engine::handleExportFileEvent (const ExportFileEvent& e) {
    bool success = false;

    switch (e.format) {
    case ExportFormat::OBJ: success = io::FileIO::exportOBJ (e.filePath,
            m_renderer.getScene ().getFocusGeometry ()->getVertices (),
            m_renderer.getScene ().getFocusGeometry ()->getIndices (),
            m_renderer.getScene ().getFocusGeometry ()->getNormals ());
        break;
    case ExportFormat::STL: success = io::FileIO::exportSTL (e.filePath,
            m_renderer.getScene ().getFocusGeometry ()->getVertices (),
            m_renderer.getScene ().getFocusGeometry ()->getIndices (),
            m_renderer.getScene ().getFocusGeometry ()->getNormals ());
        break;
    // Add cases for new formats here
    default: std::cerr << "Unsupported export format." << std::endl;
        return;
    }

    if (success) {
        std::cout << "Successfully exported file to " << e.filePath <<
            std::endl;
    } else {
        std::cerr << "Failed to export file to " << e.filePath << std::endl;
    }
}

void Engine::handleResetGeometryEvent (const ResetGeometryEvent& e) {
    m_renderer.getScene ().setFocusGeometry (
        m_renderer.getScene ().getLoadedGeometry ());
}
}