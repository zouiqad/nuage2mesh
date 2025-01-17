cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

#==============================================================
#                           Configuration
#==============================================================
set(PROJECT_NAME "nuage2mesh")
set(EXECUTABLE_NAME "nuage2mesh")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#=====================================
#				Options
#=====================================

if (NOT DEFINED ENV{VCPKG_ROOT})
    message(WARNING "VCPKG_ROOT is not set. Please set it to your local vcpkg path.")
else ()
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
endif ()

# Explicitly enable the new policy so OPENGL GLVND libraries are preferred:
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif ()

#==============================================================
#                           Sources
#==============================================================

set(SOURCES ${CMAKE_SOURCE_DIR}/source/)
set(LIBS ${CMAKE_SOURCE_DIR}/libs/)

#==============================================================
#                           Build
#==============================================================
project(${PROJECT_NAME})
set(CURRENT_TARGET ${EXECUTABLE_NAME})


add_executable(nuage2mesh source/main.cpp
        source/core/Engine.cpp
        source/core/io/Window.cpp
        source/core/graphics/Renderer.cpp
        source/core/graphics/Camera.cpp
        source/core/graphics/Geometry.cpp
        source/core/graphics/Shader.cpp
        source/core/io/FileIO.cpp
        source/core/graphics/Scene.cpp
        source/core/graphics/Node.cpp
        source/core/algorithms/MarchingCubes.cpp
        source/core/algorithms/Poisson.cpp
        source/core/graphics/Mesh.cpp
        source/core/graphics/PointCloud.cpp
        source/core/algorithms/FastTriangulation.cpp
        source/patterns/events/PoissonEvent.h
        source/patterns/events/EstimateNormalsEvent.h
        source/patterns/events/RemoveOutliersEvent.h
)

#==============================================================
#                           Libraries
#==============================================================

target_include_directories(${CURRENT_TARGET} PUBLIC ${SOURCES} PRIVATE ${LIBS})

# Add tinyfiledialogs to your build
set(TFD_DIR ${LIBS}/tinyfiledialogs)

# Include tinyfiledialogs directory
include_directories(${TFD_DIR})

# Add tinyfiledialogs source file
add_library(tinyfiledialogs STATIC
        ${TFD_DIR}/tinyfiledialogs.cpp)

#==============================================================
#                           Linking
#==============================================================

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(flann CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(imgui REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PCL CONFIG REQUIRED)

target_link_libraries(${CURRENT_TARGET} PRIVATE glad::glad glfw imgui::imgui OpenGL::GL glm::glm flann::flann tinyfiledialogs)
target_link_libraries(${CURRENT_TARGET} PRIVATE ${PCL_LIBRARIES})

# Copy the resources directory to the build directory
add_custom_command(TARGET ${CURRENT_TARGET} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/resources
        ${PROJECT_BINARY_DIR}/resources
        COMMENT "Copying resources into binary directory")

