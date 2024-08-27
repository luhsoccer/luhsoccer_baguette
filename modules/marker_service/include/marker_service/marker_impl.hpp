#pragma once

#include <utility>
#include "marker.hpp"

#include "logger/logger.hpp"
#include "time/time.hpp"
#include "transform/transform.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace luhsoccer::marker {
class MarkerImpl {
   public:
    MarkerImpl() = default;

    explicit MarkerImpl(const Type3D type_3d, glm::dvec3 pos = {0, 0, 0}, glm::dquat rotation = {},
                        glm::dvec3 scale = {1, 1, 1}, glm::dvec4 color = {1, 0.0784f, 0.5765f, 1})
        : type_3d(type_3d), position(pos), rotation(rotation), scale(scale), color(color) {}

    /**
     * @brief Set the Type object
     *
     * @param type
     */
    void setType(MType type) { this->type = type; }

    /**
     * @brief Get the Type object
     *
     * @return Type
     */
    [[nodiscard]] MType getType() const { return this->type; }

    /**
     * @brief Get the 3D Type object
     *
     * @return Type
     */
    [[nodiscard]] Type3D getType3D() const { return this->type_3d; }

    /**
     * @brief Set the 3D Type object
     *
     * @param type
     */
    void setType3D(Type3D type) { this->type_3d = type; }

    /**
     * @brief Set the Position object
     *
     * @param position
     */
    void setPosition(glm::dvec3 position) { this->position = position; }

    /**
     * @brief Set the Position object
     *
     * @param position
     */
    void setPosition(glm::dvec2 position) { this->position = glm::dvec3(position.x, 0, position.y); }

    /**
     * @brief Get the Position object
     *
     * @return glm::dvec3
     */
    [[nodiscard]] glm::dvec3 getPosition() const { return this->position; }

    /**
     * @brief Set the Rotation object
     *
     * @param quat
     */
    void setRotation(glm::dquat quat) { this->rotation = quat; }

    /**
     * @brief Set the Rotation object
     *
     * @param rot
     */
    void setRotation(double rot) { setRotation(0, rot, 0); }

    /**
     * @brief Set the Rotation object
     *
     * @param x_rot
     * @param y_rot
     * @param z_rot
     */
    void setRotation(double x_rot, double y_rot, double z_rot) {
        this->rotation = glm::angleAxis(x_rot, glm::dvec3{1, 0, 0}) * glm::angleAxis(y_rot, glm::dvec3{0, 1, 0}) *
                         glm::angleAxis(z_rot, glm::dvec3{0, 0, 1});
    }

    /**
     * @brief Get the Rotation object
     *
     * @return glm::dquat
     */
    [[nodiscard]] glm::dquat getRotation() const { return this->rotation; }

    /**
     * @brief Get the Rotation Euler object
     *
     * @return glm::dvec3
     */
    [[nodiscard]] glm::dvec3 getRotationEuler() const { return glm::eulerAngles(this->rotation); }

    /**
     * @brief Get the Rotation2 D object
     *
     * @return double
     */
    [[nodiscard]] double getRotation2D() const { return glm::eulerAngles(this->rotation).y; }

    /**
     * @brief Set the Velocity object
     *
     * @param x
     * @param y
     * @param rot
     */
    void setVelocity(double x, double y, double rot) { this->velocity = {x, y, rot}; }

    /**
     * @brief Get the Velocity object
     *
     * @return glm::dvec3
     */
    glm::dvec3 getVelocity() { return this->velocity; }

    /**
     * @brief Set the Scale object
     *
     * @param scale
     */
    void setScale(glm::dvec3 scale) { this->scale = scale; }

    /**
     * @brief Set the Scale object
     *
     * @param uniform_scale
     */
    void setScale(double uniform_scale) { this->scale = glm::dvec3(uniform_scale, uniform_scale, uniform_scale); }

    /**
     * @brief Get the Scale object
     *
     * @return glm::dvec3
     */
    [[nodiscard]] glm::dvec3 getScale() const { return this->scale; }

    /**
     * @brief Set the Color object
     *
     * @param color
     */
    void setColor(glm::dvec4 color) { this->color = color; }

    /**
     * @brief Get the Color object
     *
     * @return glm::dvec4
     */
    [[nodiscard]] glm::dvec4 getColor() const { return this->color; }

    /**
     * @brief Set the Text object
     *
     * @param text
     */
    void setText(std::string text) { this->text = std::move(text); }

    /**
     * @brief Get the Text object
     *
     * @return std::string
     */
    [[nodiscard]] std::string getText() const { return this->text; }

    /**
     * @brief Set the Ns object
     *
     * @param ns
     */
    void setNs(std::string ns) { this->ns = std::move(ns); }

    /**
     * @brief Get the Ns object
     *
     * @return std::string
     */
    [[nodiscard]] std::string getNs() const { return this->ns; }

    /**
     * @brief Set the Id object
     *
     * @param id
     */
    void setId(size_t id) { this->id = id; }

    /**
     * @brief Get the Id object
     *
     * @return size_t
     */
    [[nodiscard]] size_t getId() const { return this->id; }

    /**
     * @brief Set the Robot Identifier object
     *
     * @param robot_handle
     */
    void setRobotIdentifier(const RobotIdentifier& robot_handle) { this->robot_handle = robot_handle; }

    /**
     * @brief Get the Robot Identifier object
     *
     * @return std::optional<RobotIdentifier>
     */
    [[nodiscard]] std::optional<RobotIdentifier> getRobotIdentifier() const { return this->robot_handle; }

   private:
    constexpr static glm::dvec4 DEFAULT_COLOR{1, 0.0784f, 0.5765f, 1};
    // marker parameter
    MType type{MType::LAST_MARKER_TYPE};
    Type3D type_3d{Type3D::CYLINDER3D};
    glm::dvec3 position{0, 0, 0};
    glm::dvec3 velocity{0, 0, 0};
    glm::dquat rotation{};
    glm::dvec3 scale{1, 1, 1};
    glm::dvec4 color{DEFAULT_COLOR};
    std::string text{""};
    std::string ns{"default"};
    size_t id{0};
    bool limited_lifetime{false};
    std::optional<RobotIdentifier> robot_handle{std::nullopt};

    inline const static logger::Logger LOGGER = logger::Logger("Marker_Impl");  // TODO wait for logger fix
};
}  // namespace luhsoccer::marker
