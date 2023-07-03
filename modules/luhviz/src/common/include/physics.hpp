#pragma once

#include <glm/glm.hpp>
#include "glm/gtx/intersect.hpp"
#include <optional>
#include <vector>
#include "marker_service/marker_impl.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::luhviz {

class Physics {
   public:
    /**
     * @brief casts a ray from the mouse cursor position and returns the first hit position
     *
     * @param pos the mouse position in screen coordinates
     * @param mvp the model-view-projection matrix to transform the mouse position in screen coordinates to the world
     * @param ortho_view orhtographic view enabled
     * position
     * @return glm::dvec3
     */
    static glm::dvec3 raycast(const glm::dvec2& pos, const glm::mat4& view, const glm::mat4& projection,
                              bool ortho_view);

    /**
     * @brief calculates the nearest hit of the given ray with the robot sphere colliders
     *
     * @param robots the robots to check the hit
     * @param cam_pos position of the camera in world coordinates
     * @param ray_world the ray (or line) in world coordinates
     * @param robot_collider_radius the radius of the robots sphere colliders
     * @param robot_collider_center the center offset of the colliders from the robot origin
     * @param hit_robot reference to the robot which was hit (if one was hit)
     * @return true hit
     * @return false no hit
     */
    static bool rayRobotIntersection(const std::unordered_map<size_t, marker::MarkerImpl>& robots,
                                     const glm::dvec3& cam_pos, const glm::dvec3& ray_world,
                                     const double& robot_collider_radius, const glm::dvec3 robot_collider_center,
                                     marker::MarkerImpl& hit_robot, bool ortho_view);

    /**
     * @brief calculates the intersection point of the given ray with the ground plane
     *
     * @param ray_world the ray in world coordinates
     * @param cam_pos the camera position
     * @return std::optional<glm::dvec3> the intersection position (can be nullopt if the ray is parrallel to plane so
     * it never hits it)
     */
    static std::optional<glm::dvec3> rayGroundIntersection(const glm::dvec3& ray_world, const glm::dvec3& cam_pos,
                                                           bool ortho_view);

   private:
    const static logger::Logger LOGGER;
};
}  // namespace luhsoccer::luhviz