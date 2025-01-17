#ifndef ENGINE_H
#define ENGINE_H

#include "graphics/Renderer.h"
#include "io/Window.h"
#include "patterns/events/LoadFileEvent.h"
#include "patterns/events/RenderModeEvent.h"
#include "patterns/events/MouseDragEvent.h"
#include "patterns/events/MouseScrollEvent.h"
#include "patterns/events/MarchingCubesEvent.h"
#include "patterns/events/ExportFileEvent.h"
#include "patterns/events/FastTriangulationEvent.h"
#include "patterns/events/PoissonEvent.h"
#include "patterns/events/ExportFileEvent.h"
#include "patterns/events/RemoveOutliersEvent.h"
#include "patterns/events/EstimateNormalsEvent.h""
#include "patterns/events/FastTriangulationEvent.h"


namespace n2m {
// forward dec
class UIEvent;

class Engine {
public:
    static Engine &Instance() {
        static Engine instance;
        return instance;
    }

    Engine() = default;

    ~Engine() = default;

    bool init();

    void run();

    void shutdown();

private:
    Engine(const Engine &) = delete;

    Engine &operator=(const Engine &) = delete;

    io::Window m_window; // main window & gl context
    graphics::Renderer m_renderer; // main renderer

    float zoomFactor = 1.0f;

    // Track rotation variables
    float rotationX = 0.0f; // Vertical rotation (pitch)
    float rotationY = 0.0f; // Horizontal rotation (yaw

    void handleLoadFileEvent(const LoadFileEvent &e);

    void handleExportFileEvent(const ExportFileEvent &e);

    void handleMouseDragEvent(const MouseDragEvent &e);

    void handleMouseScrollEvent(const MouseScrollEvent &e);

    void handleRenderModeEvent(const RenderModeEvent &e);

    void handleResetGeometryEvent(const ResetGeometryEvent &e);

    void handleMarchingCubesEvent(const MarchingCubesEvent &e);

    void handleFastTriangulationEvent(const FastTriangulationEvent &e);

    void handlePoissonEvent(const PoissonEvent &e);
};
}


#endif //ENGINE_H
