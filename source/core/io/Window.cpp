#include "Window.h"

#include "FileIO.h"


#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <glm/common.hpp>
#include <iostream>


namespace n2m::io {
Window::~Window () {
    shutdown ();
}

void Window::framebuffer_size_callback (GLFWwindow* window,
    int width,
    int height) {
    glViewport (0, 0, width, height);
}

void Window::scroll_callback (GLFWwindow* window,
    double xoffset,
    double yoffset) {
    // Retrieve the Window object associated with this GLFWwindow
    auto* win = static_cast<Window*> (glfwGetWindowUserPointer (window));

    if (!win) return;

    MouseScrollEvent mouse_scroll_event (xoffset, yoffset);
    EventDispatcher::Instance ().publish (mouse_scroll_event);
}

// Mouse button callback
void Window::mouse_button_callback (GLFWwindow* window,
    int button,
    int action,
    int mods) {
    // Retrieve the Window object associated with this GLFWwindow
    auto* win = static_cast<Window*> (glfwGetWindowUserPointer (window));

    if (!win) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            glfwGetCursorPos (window, &win->lastMouseX, &win->lastMouseY);

            // MouseDragEvent mouse_drag_event (win->lastMouseX, win->lastMouseY);
            // EventDispatcher::Instance ().publish (mouse_drag_event);
        }
    }
}

// Mouse movement callback
void Window::cursorPositionCallback (GLFWwindow* window,
    double xpos,
    double ypos) {
    // Retrieve the Window object associated with this GLFWwindow
    auto* win = static_cast<Window*> (glfwGetWindowUserPointer (window));
    if (!win) return;

    if (glfwGetMouseButton (window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        // Only drag when holding the left mouse button
        double dx = xpos - win->lastMouseX;
        double dy = ypos - win->lastMouseY;

        MouseDragEvent mouse_drag_event (dx, dy);
        EventDispatcher::Instance ().publish (mouse_drag_event);

        win->lastMouseX = xpos;
        win->lastMouseY = ypos;
    }
}

bool Window::init_imgui () {
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION ();
    ImGui::CreateContext ();
    io = &ImGui::GetIO ();
    (void)io;
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // Enable Keyboard Controls
    io->ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark ();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL (window, true);

    const char* glsl_version = "#version 150";
    ImGui_ImplOpenGL3_Init (glsl_version);
    return true;
}

bool Window::init (int width, int height, const std::string& title) {
    // Initialize GLFW
    if (!glfwInit ()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint (GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint (GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint (GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    // Create window
    window =
        glfwCreateWindow (width, height, title.c_str (), NULL,NULL);

    if (!window) {
        std::cerr << "Failed to create GLFW window.\n";
        glfwTerminate ();
        return false;
    }

    glfwMakeContextCurrent (window);

    glfwSetWindowUserPointer (window, this);

    // Callbacks
    glfwSetFramebufferSizeCallback (window, framebuffer_size_callback);
    glfwSetMouseButtonCallback (window, mouse_button_callback);
    glfwSetCursorPosCallback (window, cursorPositionCallback);
    glfwSetScrollCallback (window, scroll_callback);


    // init imgui
    this->init_imgui ();

    // Load OpenGL via glad
    if (!gladLoadGLLoader ((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD.\n";
        return false;
    }

    // Set a default viewport, enable some GL states if you want
    glViewport (0, 0, width, height);
    glEnable (GL_DEPTH_TEST);

    // Sub to events
    EventDispatcher::Instance ().subscribe<SceneStateEvent> (
        [this](SceneStateEvent event) {
            this->sceneMetrics = event.metrics; // update scene metrics
        });

    EventDispatcher::Instance ().subscribe<TimerEndEvent> (
        [this](TimerEndEvent event) {
            this->executionTime = event.duration; // update scene metrics
        });
    return true;
}

void Window::shutdown () {
    if (window) {
        // Cleanup
        ImGui_ImplOpenGL3_Shutdown ();
        ImGui_ImplGlfw_Shutdown ();
        ImGui::DestroyContext ();

        glfwDestroyWindow (window);
        window = nullptr;
        glfwTerminate ();
    }
}

bool Window::pollEvents () const {
    glfwPollEvents ();

    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    if (glfwGetWindowAttrib (window, GLFW_ICONIFIED) != 0) {
        ImGui_ImplGlfw_Sleep (10);
        return false;
    }

    return true;
}

void Window::render_imgui () const {
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame ();
    ImGui_ImplGlfw_NewFrame ();
    ImGui::NewFrame ();

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
    static float f     = 0.0f;
    static int counter = 0;

    // Create a window and append into it.
    ImGui::Begin ("Settings!");

    if (ImGui::BeginMenu ("File")) {
        if (ImGui::MenuItem ("Open..", "Ctrl+O")) {
            std::string filePath = FileIO::openFileDialog ();

            LoadFileEvent load_file_event (filePath);
            EventDispatcher::Instance ().publish (load_file_event);
        }
        if (ImGui::MenuItem ("Export as OBJ")) {
            ExportFileEvent export_file_event (
                "resources/models/exported_mesh.obj",
                ExportFormat::OBJ);

            EventDispatcher::Instance ().publish (export_file_event);
        }

        if (ImGui::MenuItem ("Export as STL")) {
            ExportFileEvent export_file_event (
                "resources/models/exported_mesh.stl",
                ExportFormat::STL);

            EventDispatcher::Instance ().publish (export_file_event);
        }

        if (ImGui::MenuItem ("Close", "Ctrl+W")) {
        }
        ImGui::EndMenu ();
    }

    // Collapsible section for reconstruction algorithms
    if (ImGui::CollapsingHeader ("Reconstruction Algorithms")) {
        // Marching Cubes section
        static int gridResolution = 64; // Default resolution
        ImGui::Text ("Marching Cubes");

        // Slider for grid resolution
        ImGui::SliderInt ("Grid Resolution", &gridResolution, 2, 512, "%d",
            ImGuiSliderFlags_AlwaysClamp);

        // Ensure grid resolution remains a power of 2
        if (gridResolution < 2) gridResolution = 2;
        gridResolution = std::pow (2, std::floor (std::log2 (gridResolution)));

        // Button to apply marching cubes
        if (ImGui::Button ("Apply Marching Cubes")) {
            MarchingCubesEvent apply_marching_cubes_event (gridResolution);
            EventDispatcher::Instance ().publish (apply_marching_cubes_event);
        }

        ImGui::Separator ();
        static FastTriangulationParameters fastTriangulationParams;
        ImGui::Text ("Fast Triangulation");

        ImGui::SliderFloat ("Search Radius",
            &fastTriangulationParams.searchRadius, 0.01f, 0.1f, "%3f",
            ImGuiSliderFlags_AlwaysClamp);


        ImGui::SliderInt ("Number of nearest neighbors ",
            &fastTriangulationParams.kSearch, 5,
            50,
            "%d",
            ImGuiSliderFlags_AlwaysClamp);

        ImGui::Checkbox ("Enforces consistency in normal directions ",
            &fastTriangulationParams.normalConsistency);


        // Button to apply fast triangulation
        if (ImGui::Button ("Fast Triangulation")) {
            FastTriangulationEvent fast_triangulation_event (
                fastTriangulationParams);
            EventDispatcher::Instance ().publish (fast_triangulation_event);
        }
        ImGui::Separator ();

        // Poisson Reconstruction section
        static PoissonReconstructionParameters poissonParams;
        ImGui::Text ("Poisson Reconstruction");

        ImGui::SliderInt ("k-Nearest Neighbors", &poissonParams.k_search, 5, 50,
            "%d",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderInt ("Octree Depth", &poissonParams.depth, 4, 12, "%d",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderInt ("Solver Divide", &poissonParams.solver_divide, 1, 16,
            "%d",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderInt ("Iso Divide", &poissonParams.iso_divide, 1, 16, "%d",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderFloat ("Samples Per Node", &poissonParams.samples_per_node,
            0.1f, 10.0f, "%.2f",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderFloat ("Bounding Box Scale", &poissonParams.scale, 1.0f,
            2.0f, "%.2f",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::Checkbox ("Confidence in Normals", &poissonParams.confidence);

        if (ImGui::Button ("Apply Poisson Reconstruction")) {
            PoissonEvent poisson_reconstruction_event (
                poissonParams);
            EventDispatcher::Instance ().publish (poisson_reconstruction_event);
        }
        ImGui::Separator ();
    }

    ImGui::Separator ();

    if (ImGui::CollapsingHeader ("Pre-processing operations")) {
        // Normal Estimation section
        static int kSearchNormals = 20;
        ImGui::Text ("Normal Estimation");
        ImGui::SliderInt ("k-Nearest Neighbors", &kSearchNormals, 5, 50, "%d",
            ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::Button ("Estimate Normals")) {
            EstimateNormalsEvent estimate_normals_event (kSearchNormals);
            EventDispatcher::Instance ().publish (estimate_normals_event);
        }

        ImGui::Separator ();

        // Outliers Removal (Cloud Point Cleaning) section
        static int meanK        = 50;
        static float stddevMul = 1.0;
        ImGui::Text ("Outliers Removal (Cloud Point Cleaning)");
        ImGui::SliderInt ("Mean K", &meanK, 10, 200, "%d",
            ImGuiSliderFlags_AlwaysClamp);
        ImGui::SliderFloat ("Std Dev Multiplier", &stddevMul, 0.1f, 3.0f,
            "%.2f",
            ImGuiSliderFlags_AlwaysClamp);
        if (ImGui::Button ("Remove Outliers")) {
            RemoveOutliersEvent remove_outliers_event (meanK, stddevMul);
            EventDispatcher::Instance ().publish (remove_outliers_event);
        }
    }

    ImGui::Separator ();
    ImGui::Text ("Reconstruction algorithm execution time %.3f ms",
        this->executionTime);

    ImGui::Separator ();
    ImGui::SeparatorText ("Vizualization mode");
    const char* items[]             = {"Solid", "Wireframe"};
    static const char* current_item = items[0];

    //RenderModeEvent render_mode_event(RenderModeEvent::renderMode::Solid);
    for (int n = 0; n < IM_ARRAYSIZE (items); n++) {
        bool is_selected = (current_item == items[n]);
        // You can store your selection however you want, outside or inside your objects
        if (ImGui::Selectable (items[n], is_selected)) {
            current_item = items[n];
            RenderModeEvent render_mode_event (current_item);
            EventDispatcher::Instance ().publish (render_mode_event);
        }
    }


    ImGui::SeparatorText ("Statistics");
    ImGui::Text ("Number of vertex %d", this->sceneMetrics.vertexCount);
    ImGui::Text ("Number of triangles %d", this->sceneMetrics.triangleCount);

    ImGui::Text ("Average %.3f ms/frame (%.1f FPS)",
        1000.0f / io->Framerate, io->Framerate);


    ImGui::Separator ();
    if (ImGui::Button ("Reset Geometry")) {
        ResetGeometryEvent reset_geometry_event;
        EventDispatcher::Instance ().publish (reset_geometry_event);
    }
    ImGui::End ();
}

void Window::render () const {
    if (!window) return;

    // Render imgui
    render_imgui ();

    // Rendering
    ImGui::Render ();
    int display_w, display_h;
    glfwGetFramebufferSize (window, &display_w, &display_h);
    glViewport (0, 0, display_w, display_h);
    // glClearColor (clear_color.x * clear_color.w, clear_color.y * clear_color.w,
    //     clear_color.z * clear_color.w, clear_color.w);
    // glClear (GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData (ImGui::GetDrawData ());

    glfwSwapBuffers (window);
}

bool Window::shouldClose () const {
    return (window && glfwWindowShouldClose (window));
}
}

