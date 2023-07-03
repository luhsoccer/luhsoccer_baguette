#include "include/controls.hpp"
#include <cmath>
#include <iostream>

namespace luhsoccer::luhviz {

void Controls::setView3d(bool perspective) {
    if (perspective_view == perspective) return;
    perspective_view = perspective;
    resetView();
}

void Controls::resetView() {
    this->cam_target = {0, 0, 0};
    this->cam_offset = {0, 0, 0};
    this->view_resetted = true;
    this->zoom = this->STANDARD_ZOOM;
    if (perspective_view) {
        horizontal_angle = STAND_PERSP_ANGLEX;
        vertical_angle = STAND_PERSP_ANGLEY;
    } else {
        horizontal_angle = 0;
        vertical_angle = STAND_TOPDOWN_ANGLEY;
    }
    initialized = false;
}

void Controls::computeMatricesFromInputs(GLFWwindow* window, glm::vec2 window_size, glm::vec2 mouse_pos,
                                         bool mouse_active, bool window_size_changed, float scroll_value,
                                         bool is_hovered, bool update_mouse_pos) {
    zoom = std::clamp(zoom - scroll_value, ZOOM_MIN, ZOOM_MAX);
    if (!initialized || window_size_changed || !is_hovered || update_mouse_pos) {
        initialized = true;
        last_x_pos = mouse_pos.x;
        last_y_pos = mouse_pos.y;

        cam_right = glm::vec3(std::sin(horizontal_angle - PI / 2), 0, std::cos(horizontal_angle - PI / 2));
        cam_right = glm::normalize(cam_right);
        direction = glm::normalize(cam_target - cam_position);
        cam_up = -glm::cross(cam_right, direction);
    }
    if (!mouse_active && perspective_view == perspective_view_before && !view_resetted) {
        perspective_view_before = perspective_view;
        return;  // dont compute if nothing changed
    }
    view_resetted = false;
    perspective_view_before = perspective_view;

    // glfwGetTime is called only once, the first time this function is called
    static double last_time = glfwGetTime();

    // Compute time difference between current and last frame
    double current_time = glfwGetTime();
    auto delta_time = static_cast<float>(current_time - last_time);
    auto x_diff = static_cast<float>(last_x_pos - mouse_pos.x);
    auto y_diff = static_cast<float>(last_y_pos - mouse_pos.y);

    if (!window_size_changed && is_hovered && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
        if (perspective_view) {
            bool rotation_mode =
                !(glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)) &&
                !glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);
            // Rotate(Leftmouse) view
            if (rotation_mode) {
                // rotate viewport in perspective mode
                horizontal_angle += x_diff * ROTATE_SPEED_PERS;
                vertical_angle =
                    std::clamp(vertical_angle - y_diff * ROTATE_SPEED_PERS * (window_size.y / window_size.x),
                               ORBIT_Y_MIN, ORBIT_Y_MAX);
                cam_right = glm::vec3(std::sin(horizontal_angle - PI / 2), 0, std::cos(horizontal_angle - PI / 2));
                cam_right = glm::normalize(cam_right);
                direction = glm::normalize(cam_target - cam_position);
                cam_up = -glm::cross(cam_right, direction);
            }
            // move(LeftMouse+Shift) view
            else {
                cam_right = glm::vec3(std::sin(horizontal_angle - PI / 2), 0, std::cos(horizontal_angle - PI / 2));
                cam_right = glm::normalize(cam_right);
                direction = glm::normalize(cam_target - cam_position);
                cam_up = -glm::cross(cam_right, direction);

                // move in horizontal camera space
                glm::vec3 add = cam_right * x_diff * delta_time * MOVE_SPEED_PERSP * zoom / STANDARD_ZOOM;
                cam_target -= add;
                cam_offset -= add;
                // move in vertical camera space
                glm::vec3 substract = cam_up * y_diff * delta_time * (window_size.y / window_size.x) *
                                      MOVE_SPEED_PERSP * zoom / STANDARD_ZOOM;
                cam_target -= substract;
                cam_offset -= substract;
            }

        } else {
            bool rotation_mode = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT);
            if (rotation_mode) {
                // rotate only around z in topdown view
                horizontal_angle += x_diff * ROTATE_SPEED_PERS;
                cam_right = glm::vec3(std::sin(horizontal_angle - PI / 2), 0, std::cos(horizontal_angle - PI / 2));
                cam_right = glm::normalize(cam_right);
                direction = glm::normalize(cam_target - cam_position);
                cam_up = -glm::cross(cam_right, direction);
            } else {
                cam_right = glm::vec3(std::sin(horizontal_angle - PI / 2), 0, std::cos(horizontal_angle - PI / 2));
                cam_right = glm::normalize(cam_right);
                direction = glm::normalize(cam_target - cam_position);
                cam_up = -glm::cross(cam_right, direction);

                // move in horizontal camera space
                glm::vec3 add = cam_right * x_diff * delta_time * MOVE_SPEED_ORTH * zoom / STANDARD_ZOOM;
                cam_target -= add;
                cam_offset -= add;
                // move in vertical camera space
                glm::vec3 substract = cam_up * y_diff * delta_time * (window_size.y / window_size.x) * MOVE_SPEED_ORTH *
                                      zoom / STANDARD_ZOOM;
                cam_target -= substract;
                cam_offset -= substract;
            }
        }
    }

    // ensure we are always in 0-2PI range
    if (horizontal_angle < 0) horizontal_angle += 2 * PI;
    if (horizontal_angle > 2 * PI) horizontal_angle -= 2 * PI;

    // Direction : Spherical coordinates to Cartesian coordinates conversion
    cam_position = {std::cos(vertical_angle) * std::sin(horizontal_angle) * RADIUS, std::sin(vertical_angle) * RADIUS,
                    std::cos(vertical_angle) * std::cos(horizontal_angle) * RADIUS};
    cam_position += cam_offset;

    // Projection matrix
    float aspect = window_size.x / window_size.y;
    if (perspective_view)
        projection_matrix = glm::perspective(glm::radians(zoom), aspect, CAMERA_NEAR, CAMERA_FAR);
    else {
        const float ortho_zoom_multiplier = 0.5f;
        glm::vec2 center{ortho_zoom_multiplier * aspect * this->zoom, ortho_zoom_multiplier * this->zoom};
        projection_matrix =
            glm::ortho(-center.x / 2, center.x / 2, -center.y / 2, center.y / 2, CAMERA_NEAR, CAMERA_FAR);
    }

    // Camera matrix
    view_matrix = glm::lookAt(cam_position,  // Camera is here
                              cam_target,    // and looks to cam target
                              cam_up         // Head up
    );

    // For the next frame, the "last time" will be "now"
    last_time = current_time;
    last_x_pos = mouse_pos.x;
    last_y_pos = mouse_pos.y;
}

}  // namespace luhsoccer::luhviz