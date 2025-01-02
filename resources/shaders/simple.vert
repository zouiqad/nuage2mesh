#version 450 core

layout (location = 0) in vec3 inPosition;

uniform mat4 u_view;         // Model-View-Projection matrix
uniform mat4 u_model;       // Model-iew-Projection matrix
uniform mat4 u_proj;        // Model-View-Projection matrix

uniform float u_pointSize;  // Allows controlling point size from CPU

void main()
{
    gl_Position = u_proj * u_view * u_model * vec4(inPosition, 1.0);
    gl_PointSize = u_pointSize; // If you enable GL_PROGRAM_POINT_SIZE
}