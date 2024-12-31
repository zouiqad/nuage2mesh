cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

#==============================================================
#                           Configuration
#==============================================================
set(PROJECT_NAME "nuage2mesh")
set(EXECUTABLE_NAME  "nuage2mesh")

set(CMAKE_CXX_STANDARD 20)

#=====================================
#				Options
#=====================================

# Explicitly enable the new policy so OPENGL GLVND libraries are preferred:
if (POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW)
endif()



if(NOT DEFINED ENV{VCPKG_ROOT})
    message(WARNING "VCPKG_ROOT is not set. Please set it to your local vcpkg path.")
else()
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
endif()

#==============================================================
#                           Sources
#==============================================================

set(SOURCES ${CMAKE_SOURCE_DIR}/src/)

#==============================================================
#                           Build
#==============================================================
project(${PROJECT_NAME})
set(CURRENT_TARGET ${EXECUTABLE_NAME})

add_executable(nuage2mesh src/main.cpp)

foreach(dir ${external})
    add_subdirectory(${dir})
endforeach()

#==============================================================
#                           Linking
#==============================================================

target_include_directories(${CURRENT_TARGET} PRIVATE ${EXTERNAL})

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

target_link_libraries(${CURRENT_TARGET} PRIVATE OpenGL::GL)
target_link_libraries(${CURRENT_TARGET} PRIVATE glad::glad)
target_link_libraries(${CURRENT_TARGET} PRIVATE glfw)
#target_link_libraries(${CURRENT_TARGET} PRIVATE imgui::imgui)