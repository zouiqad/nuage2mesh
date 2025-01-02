cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

#==============================================================
#                           Configuration
#==============================================================
set(PROJECT_NAME "nuage2mesh")
set(EXECUTABLE_NAME "nuage2mesh")

set(CMAKE_CXX_STANDARD 20)

#=====================================
#				Options
#=====================================

# Explicitly enable the new policy so OPENGL GLVND libraries are preferred:
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif ()


if (NOT DEFINED ENV{VCPKG_ROOT})
    message(WARNING "VCPKG_ROOT is not set. Please set it to your local vcpkg path.")
else ()
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
endif ()

#==============================================================
#                           Sources
#==============================================================

set(SOURCES ${CMAKE_SOURCE_DIR}/src/)

#==============================================================
#                           Build
#==============================================================
project(${PROJECT_NAME})
set(CURRENT_TARGET ${EXECUTABLE_NAME})

add_executable(nuage2mesh source/main.cpp
        source/engine/Engine.cpp
        source/engine/Engine.h
        source/engine/io/Window.cpp
        source/engine/io/Window.h
        source/engine/graphics/Renderer.cpp
        source/engine/graphics/Renderer.h
        source/engine/graphics/Camera.cpp
        source/engine/graphics/Camera.h
        source/engine/graphics/Geometry.cpp
        source/engine/graphics/Geometry.h
        source/engine/graphics/Program.cpp
        source/engine/graphics/Program.h)

foreach (dir ${external})
    add_subdirectory(${dir})
endforeach ()

#==============================================================
#                           Linking
#==============================================================

target_include_directories(${CURRENT_TARGET} PRIVATE ${EXTERNAL})

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui REQUIRED)
find_package(OpenGL REQUIRED)

target_link_libraries(${CURRENT_TARGET} PRIVATE glad::glad glfw imgui::imgui OpenGL::GL glm::glm)

# Copy the resources directory to the build directory
file(COPY ${CMAKE_SOURCE_DIR}/resources DESTINATION ${CMAKE_BINARY_DIR})