//
// Created by zouiqad on 31/12/24.
//

#include "Window.h"

#include "FileLoader.h"
#include "../../events/LoadFileEvent.h"
#include "../../events/MouseDragEvent.h"
#include "../../events/MouseScrollEvent.h"
#include "../../patterns/singleton/EventDispatcher.h"

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
            std::string filePath = FileLoader::openFileDialog ();
            LoadFileEvent load_file_event (filePath);

            EventDispatcher::Instance ().publish (load_file_event);

            std::cout
                <<
                "close file dialog" << std::endl;
            /* Do stuff */
        }
        if (ImGui::MenuItem ("Save", "Ctrl+S")) {
            /* Do stuff */
        }
        if (ImGui::MenuItem ("Close", "Ctrl+W")) {
        }
        ImGui::EndMenu ();
    }

    ImGui::Text ("This is some useful text.");
    // Display some text (you can use a format strings too)
    bool checkboxValue = true;
    ImGui::Checkbox ("test", &checkboxValue);
    // Edit bools storing our window open/close state
    ImGui::Checkbox ("Another Window", &checkboxValue);

    ImGui::SliderFloat ("float", &f, 0.0f, 1.0f);

    float clear_color = 0.5f;
    // Edit 1 float using a slider from 0.0f to 1.0f
    ImGui::ColorEdit3 ("clear color", (float*)&clear_color);
    // Edit 3 floats representing a color

    if (ImGui::Button ("Button"))
        // Buttons return true when clicked (most widgets return true when edited/activated)
        counter++;
    ImGui::SameLine ();
    ImGui::Text ("counter = %d", counter);

    ImGui::Text ("Application average %.3f ms/frame (%.1f FPS)",
        1000.0f / io->Framerate, io->Framerate);
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

