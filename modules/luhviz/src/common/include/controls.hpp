#pragma once

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include <glm/ext/scalar_constants.hpp>
#include "glm/glm.hpp"
#include <glm/ext/matrix_clip_space.hpp>
#include <glm/ext/matrix_transform.hpp>
#include "include/data_proxy.hpp"
#include "logger/logger.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::luhviz {

class Controls {
   public:
    Controls(float camera_orientation_angle_x, float camera_orientation_angle_y, float camera_orientation_zoom,
             bool perspective_view)
        : horizontal_angle(static_cast<float>(camera_orientation_angle_x)),
          vertical_angle(static_cast<float>(camera_orientation_angle_y)),
          zoom(static_cast<float>(camera_orientation_zoom)),
          perspective_view(perspective_view) {
        if (!perspective_view) {
            horizontal_angle = 0;
            vertical_angle = STAND_TOPDOWN_ANGLEY;
        }
    };
    /**
     * @brief computes the camera position and rotation from user input
     *
     * @param window
     * @param window_size
     * @param mouse_pos
     * @param mouse_active
     * @param window_size_changed
     * @param scroll_value
     */
    void computeMatricesFromInputs(GLFWwindow* window, glm::vec2 window_size, glm::vec2 mouse_pos, bool mouse_active,
                                   bool window_size_changed, float scroll_value, bool is_hovered,
                                   bool update_mouse_pose);

    /**
     * @brief Get the View Matrix object
     *
     * @return glm::mat4
     */
    [[nodiscard]] glm::mat4 getViewMatrix() const { return view_matrix; }

    /**
     * @brief Get the Projection Matrix object
     *
     * @return glm::mat4
     */
    [[nodiscard]] glm::mat4 getProjectionMatrix() const { return projection_matrix; }

    /**
     * @brief Get the Camera Position in world space
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getCameraPosition() const { return this->cam_position; }

    /**
     * @brief Get the Camera target Position in world space
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getCameraTargetPosition() const { return this->cam_target; }

    /**
     * @brief Get the Camera Right vector
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getCameraRight() const { return this->cam_right; }

    /**
     * @brief Get the Camera Up vector
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getCameraUp() const { return this->cam_up; }

    /**
     * @brief Get the Camera Direction vector
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getCameraDirection() const { return this->direction; }

    /**
     * @brief getter for perspective view
     *
     * @return true
     * @return false
     */
    bool isPerspectiveView() { return this->perspective_view; }

    /**
     * @brief Set the View3d object
     *
     * @param view3d
     */
    void setView3d(bool view3d);

    /**
     * @brief resets the view
     *
     */
    void resetView();

    /**
     * @brief saves the current view settings (camera orientation, zoom and perspective)
     *
     */
    void saveParams(DataProxy& proxy) {
        proxy.setConfigDouble("camera_angle_x", this->horizontal_angle);
        proxy.setConfigDouble("camera_angle_y", this->vertical_angle);
        proxy.setConfigDouble("camera_zoom", this->zoom);
        proxy.setConfigBool("perspective_view", this->perspective_view);
    }

   private:
    // PARAMS for camera
    constexpr static float PI = glm::pi<float>();
    constexpr static float ROTATE_SPEED_PERS = 2.0f;
    constexpr static float MOVE_SPEED_PERSP = 108.0f;
    constexpr static float MOVE_SPEED_ORTH = 200.0f;
    constexpr static float RADIUS = 15.0f;
    constexpr static float CAMERA_NEAR = 0.1f, CAMERA_FAR = 100.0f;
    constexpr static float STAND_PERSP_ANGLEX = PI;
    constexpr static float STAND_PERSP_ANGLEY = PI / 4;
    constexpr static float STAND_TOPDOWN_ANGLEY = PI / 2;
    constexpr static float ORBIT_Y_MIN = 0, ORBIT_Y_MAX = PI / 2;
    constexpr static float STANDARD_ZOOM = 15.0f;
    constexpr static float ZOOM_MAX = 35.0f, ZOOM_MIN = 1.0f;
    constexpr static float STANDARD_MOUSE_SPEED = 0.001f;

    glm::mat4 view_matrix = glm::mat4(1);
    glm::mat4 projection_matrix = glm::mat4(1);

    // cam properties
    const float default_y_pos = 10;
    glm::vec3 cam_position = glm::vec3(0, default_y_pos, 0);
    glm::vec3 cam_target = glm::vec3(0, 0, 0);
    glm::vec3 cam_right = glm::vec3(0, 0, 0);
    glm::vec3 cam_up = glm::vec3(0, 0, 0);
    glm::vec3 direction = glm::vec3(0, 0, 0);
    // Initial horizontal angle : toward -Z
    float horizontal_angle = STAND_PERSP_ANGLEX;
    // Initial vertical angle : none
    float vertical_angle = STAND_PERSP_ANGLEY;
    float mouse_speed = STANDARD_MOUSE_SPEED;

    glm::vec3 cam_offset{};

    double last_x_pos = 0, last_y_pos = 0;
    bool initialized = false;
    float zoom = STANDARD_ZOOM;
    bool perspective_view = true;
    bool perspective_view_before = true;
    bool view_resetted = false;

    logger::Logger logger{"luhviz/controls"};
};

}  // namespace luhsoccer::luhviz
