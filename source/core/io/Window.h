#ifndef WINDOW_H
#define WINDOW_H

#include <string>
#include "patterns/events/MarchingCubesEvent.h"
#include "patterns/events/LoadFileEvent.h"
#include "patterns/events/MouseDragEvent.h"
#include "patterns/events/MouseScrollEvent.h"
#include "patterns/events/SceneStateEvent.h"
#include "patterns/singleton/EventDispatcher.h"
#include "patterns/events/ExportFileEvent.h"
#include "patterns/events/FastTriangulationEvent.h"
#include "patterns/events/PoissonEvent.h"
#include "patterns/events/RenderModeEvent.h"
#include "patterns/events/ResetGeometryEvent.h"
#include "patterns/events/TimerEndEvent.h"
#include "patterns/events/EstimateNormalsEvent.h"
#include "patterns/events/RemoveOutliersEvent.h"


// Forward declaration to glfw
struct GLFWwindow;
struct ImGuiIO;

namespace n2m::io {
class Window {
public:
    Window() = default;

    ~Window();

    bool init(int width, int height, const std::string &title);

    bool init_imgui();

    void shutdown();

    // Handle polling events return false if window is asleep
    bool pollEvents() const;

    void render() const;

    // handle imgui
    void render_imgui() const;

    // If you need to check for close
    bool shouldClose() const;

    // Access to raw GLFWwindow pointer (if needed by Renderer, etc.)
    GLFWwindow *getNativeWindow() const { return window; }

private:
    GLFWwindow *window = nullptr;
    ImGuiIO *io = nullptr;

    SceneStateEvent::SceneMetrics sceneMetrics;
    float executionTime;

    double lastMouseX = 0.0;
    double lastMouseY = 0.0;

    // Callbacks
    static void mouse_button_callback(GLFWwindow *window,
                                      int button,
                                      int action,
                                      int mods);

    static void cursorPositionCallback(GLFWwindow *window,
                                       double xpos,
                                       double ypos);

    static void framebuffer_size_callback(GLFWwindow *window,
                                          int width,
                                          int height);

    static void scroll_callback(GLFWwindow *window,
                                double xoffset,
                                double yoffset);
};
}

#endif //WINDOW_H
