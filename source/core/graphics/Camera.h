#ifndef CAMERA_H
#define CAMERA_H
#include <glm/glm.hpp>

namespace n2m::graphics {
class Camera {
public:
    Camera ();
    ~Camera () = default;

    // Set perspective parameters
    void setPerspective (float fovDegrees,
        float aspect,
        float nearPlane,
        float farPlane);

    // Optionally set an orthographic projection
    // void setOrthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);

    // Basic camera transforms
    void setPosition (const glm::vec3& pos);
    void lookAt (const glm::vec3& target);
    void setUp (const glm::vec3& upVec);
    void zoom (const float& zoomFactor);

    glm::mat4 getViewMatrix () const;
    glm::mat4 getProjectionMatrix () const;

    // You can add movement methods (moveForward, moveRight, pitch, yaw, etc.)
    // For simplicity, let's keep it basic.

private:
    glm::vec3 position;
    glm::vec3 forward; // or direction
    glm::vec3 up;

    // For perspective
    float fov;
    float aspectRatio;
    float nearZ;
    float farZ;

    // Zoom
    float baseFov;
    float zoomSensitivity = 1.0f;
    float fovMin          = 30.0f;
    float fovMax          = 90.0f;
};
} // namespace MyApp::Graphics

#endif // CAMERA_H