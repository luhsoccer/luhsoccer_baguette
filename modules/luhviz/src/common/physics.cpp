#include "include/physics.hpp"

namespace luhsoccer::luhviz {

glm::dvec3 Physics::raycast(const glm::dvec2& pos, const glm::mat4& view, const glm::mat4& projection,
                            bool ortho_view) {
    // get inverse transformation matrix
    glm::mat4 m = glm::inverse(projection * view);

    // define start and end point of ray in world space
    glm::vec4 ray_start_world = m * glm::vec4{pos.x, pos.y, -1.0, 1.0f};
    ray_start_world /= ray_start_world.w;

    if (ortho_view) {
        // top down view is easy
        return {ray_start_world.x, 0, ray_start_world.z};
    } else {
        // Transformation into world space
        // for more info see
        // https://antongerdelan.net/opengl/raycasting.html and
        // http://www.opengl-tutorial.org/miscellaneous/clicking-on-objects/picking-with-a-physics-library/

        glm::vec4 ray_end_world = m * glm::vec4{pos.x, pos.y, 0.0, 1.0f};
        ray_end_world /= ray_end_world.w;
        // create normalized direction vector
        glm::vec3 ray_dir_world = glm::normalize(ray_end_world - ray_start_world);

        // save it as vec3 (w is not needed)
        return glm::dvec3{ray_dir_world};
    }
}

bool Physics::rayRobotIntersection(const std::unordered_map<size_t, marker::MarkerImpl>& robots,
                                   const glm::dvec3& cam_pos, const glm::dvec3& ray_world,
                                   const double& robot_collider_radius, const glm::dvec3 robot_collider_center,
                                   marker::MarkerImpl& hit_robot, bool ortho_view) {
    // calculate intersection with all robot sphere colliders
    bool hit = false;
    double min_distance = DBL_MAX;
    marker::MarkerImpl r{};
    for (const auto& [id, robot] : robots) {
        double distance = DBL_MAX;
        if (ortho_view) {
            // just calculate distance between collider center and ray_world smaller than radius
            glm::dvec3 coll_pos2d = robot.getPosition() + robot_collider_center;
            distance = glm::distance(coll_pos2d, ray_world);
            if (distance < robot_collider_radius) {
                hit = true;
                hit_robot = robot;
                min_distance = distance;
                return hit;
            }
        } else {
            // check collision with sphere with coll_pos and radius
            bool hit_sphere = glm::intersectRaySphere(cam_pos, ray_world, robot.getPosition() + robot_collider_center,
                                                      robot_collider_radius * robot_collider_radius, distance);
            if (hit_sphere && distance < min_distance) {
                hit = true;
                r = robot;
                min_distance = distance;
            }
        }
    }

    hit_robot = r;
    return hit;
}

std::optional<glm::dvec3> Physics::rayGroundIntersection(const glm::dvec3& ray_world, const glm::dvec3& cam_pos,
                                                         bool ortho_view) {
    // calculate intersection of ray with ground plane
    // our plane equation is y = 0, because y axis is up
    // our ray lines equation is (x,y,z) = cam_pos + t * ray_world
    // first we calc t with y = 0 = cam_pos.y + t * ray_world.y
    // than we use t to calc x and z
    if (ray_world.y == 0) {
        // no ground hit
        if (ortho_view) {
            return ray_world;
        } else {
            return std::nullopt;
        }
    } else {
        return cam_pos + ray_world * glm::dvec3{-cam_pos.y / ray_world.y};
    }
}

}  // namespace luhsoccer::luhviz