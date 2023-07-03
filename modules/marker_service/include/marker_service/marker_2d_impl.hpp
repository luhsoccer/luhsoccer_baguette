#pragma once

#include <utility>
#include <numeric>
#include "logger/logger.hpp"
#include "time/time.hpp"
#include "marker.hpp"
#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace luhsoccer::marker {

inline glm::dvec4 toVec(Color color) { return {color.red, color.green, color.blue, color.alpha}; }

class Marker2DImpl {
   public:
    Marker2DImpl(float thickness = DEFAULT_THICKNESS, const Color& color = {})
        : thickness(thickness), color({color.red, color.green, color.blue, color.alpha}){};

    /**
     * @brief Set the Position object
     *
     * @param pos
     */
    void setPosition(const Point& pos) { this->position = glm::vec3(pos.x, pos.y, pos.z); }

    /**
     * @brief Set the Rotation object
     *
     * @param rot
     */
    void setRotation(float rot) {
        glm::vec3 r = glm::eulerAngles(this->rotation);
        this->rotation = glm::angleAxis(r.x, glm::vec3(1, 0, 0)) * glm::angleAxis(rot, glm::vec3(0, 1, 0)) *
                         glm::angleAxis(r.z, glm::vec3(0, 0, 1));
    }

    /**
     * @brief Set the Scale object
     *
     * @param scale
     */
    void setScale(ScaleVec3 scale) { this->scale = {scale.x, scale.y, scale.z}; }

    /**
     * @brief Set the Line Thickness object
     *
     * @param thickness
     */
    void setLineThickness(float thickness) { this->thickness = thickness; }

    /**
     * @brief Set the Color object
     *
     * @param c
     */
    void setColor(Color c) { this->color = {c.red, c.green, c.blue, c.alpha}; }

    /**
     * @brief Set the Point Colors object
     *
     * @param c
     */
    void setPointColors(const std::vector<Color>& c) {
        this->input_colors.clear();
        if (c.empty()) return;
        for (auto col : c) {
            this->input_colors.emplace_back(toVec(col));
        }
    }

    /**
     * @brief Set the Closed Path object
     *
     * @param closed
     */
    void setClosedPath(bool closed) { this->closed_path = closed; }

    /**
     * @brief Set the Filled object
     *
     * @param filled
     */
    void setFilled(bool filled) { this->filled = filled; }

    /**
     * @brief Set the Radius object
     *
     * @param radius
     */
    void setRadius(float radius) { this->radius = radius; }

    /**
     * @brief Set the Resolution object
     *
     * @param resolution
     */
    void setResolution(unsigned int resolution) { this->resolution = resolution; }

    /**
     * @brief Set the Size object
     *
     * @param size
     */
    void setSize(SizeVec2 size) { this->size = size; }

    /**
     * @brief Set the Points object
     *
     * @param p1
     * @param p2
     */
    void setPoints(Point p1, Point p2) {
        // luhviz coordinate system is different, x,y,height is mapped to x=x, y=height and z = -y
        this->p1 = {p1.x, p1.z, -p1.y};
        this->p2 = {p2.x, p2.z, -p2.y};
    }

    /**
     * @brief Set the Points object
     *
     * @param points
     */
    void setPoints(const std::vector<Point>& points) {
        this->points.clear();
        for (const auto& p : points) {
            // luhviz coordinate system is different, x,y,height is mapped to x=x, y=height and z = -y
            this->points.emplace_back(p.x, p.z, -p.y);
        }
    }

    /**
     * @brief Set the Type object
     *
     * @param type
     */
    void setType(MType type) { this->type = type; }

    /**
     * @brief Set the Type object
     *
     * @param type
     */
    void setType2D(Type2D type) { this->type_2d = type; }

    /**
     * @brief Set the Ns object
     *
     * @param ns
     */
    void setNs(std::string ns) { this->ns = std::move(ns); }

    /**
     * @brief Set the Id object
     *
     * @param id
     */
    void setId(size_t id) { this->id = id; }

    /**
     * @brief Set the Limited Lifetime object
     *
     * @param limited_lifetime
     */
    void setLimitedLifetime(bool limited_lifetime) { this->limited_lifetime = limited_lifetime; }

    /**
     * @brief Set the Strip Type object
     *
     * @param strip_type
     */
    void setStripType(Type2D strip_type) { this->strip_type = strip_type; }

    /**
     * @brief Set the Heatmap Size object
     *
     * @param heatmap_size
     * @param heatmap_data_size
     */
    void setHeatmapSize(SizeVec2 heatmap_size, SizeVec2 heatmap_data_size) {
        this->heatmap_size = heatmap_size;
        this->heatmap_data_size = heatmap_data_size;
    }

    /**
     * @brief Set the Rotations Y object
     *
     * @param rotations
     */
    void setRotationsY(const std::vector<double>& rotations) {
        this->rotations_y.clear();
        for (double rot : rotations) {
            glm::vec3 r = glm::eulerAngles(this->rotation);
            this->rotations_y.emplace_back(glm::angleAxis(r.x, glm::vec3(1, 0, 0)) *
                                           glm::angleAxis(static_cast<float>(rot), glm::vec3(0, 1, 0)) *
                                           glm::angleAxis(r.z, glm::vec3(0, 0, 1)));
        }
    }

    /**
     * @brief Get the Position object
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getPosition() const { return this->position; }

    /**
     * @brief Get the Rotation object
     *
     * @return glm::dquat
     */
    [[nodiscard]] glm::dquat getRotation() const { return this->rotation; }

    /**
     * @brief Get the Scale object
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getScale() const { return this->scale; }

    /**
     * @brief Get the Vertices object
     *
     * @return std::vector<glm::vec3>
     */
    [[nodiscard]] const std::vector<glm::vec3>& getVertices() const { return this->vertices; }

    /**
     * @brief Get the Thickness object
     *
     * @return float
     */
    [[nodiscard]] float getThickness() const { return this->thickness; }

    /**
     * @brief Get the Color object
     *
     * @return glm::dvec4
     */
    [[nodiscard]] glm::dvec4 getColor() const { return this->color; }

    /**
     * @brief Get the Colors object
     *
     * @return std::vector<glm::dvec4>
     */
    [[nodiscard]] const std::vector<glm::vec4>& getColors() const { return this->colors; }

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool isClosedPath() const { return this->closed_path; }

    /**
     * @brief
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool isFilled() const { return this->filled; }

    /**
     * @brief Get the Resolution object
     *
     * @return unsigned int
     */
    [[nodiscard]] unsigned int getResolution() const { return this->resolution; }

    /**
     * @brief Get the Radius object
     *
     * @return float
     */
    [[nodiscard]] float getRadius() const { return this->radius; }

    /**
     * @brief Get the Size object
     *
     * @return SizeVec2
     */
    [[nodiscard]] SizeVec2 getSize() const { return this->size; }

    /**
     * @brief
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getP1() const { return this->p1; }

    /**
     * @brief
     *
     * @return glm::vec3
     */
    [[nodiscard]] glm::vec3 getP2() const { return this->p2; }

    /**
     * @brief Get the Type object
     *
     * @return MarkerImpl::Type
     */
    [[nodiscard]] MType getType() const { return this->type; }

    /**
     * @brief Get the Type 2D object
     *
     * @return MarkerImpl::Type
     */
    [[nodiscard]] Type2D getType2D() const { return this->type_2d; }

    /**
     * @brief Get the Ns object
     *
     * @return std::string
     */
    [[nodiscard]] std::string getNs() const { return this->ns; }

    /**
     * @brief Get the Id object
     *
     * @return size_t
     */
    [[nodiscard]] size_t getId() const { return this->id; }

    /**
     * @brief Get the Limited Lifetime object
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool getLimitedLifetime() const { return this->limited_lifetime; }

    /**
     * @brief Get the Strip Type object
     *
     * @return MarkerImpl
     */
    [[nodiscard]] Type2D getStripType() const { return this->strip_type; }

    /**
     * @brief Get the Heatmap Size object
     *
     * @return SizeVec2
     */
    [[nodiscard]] SizeVec2 getHeatmapSize() const { return this->heatmap_size; }

    /**
     * @brief returns true if the vertices were updated
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool isVerticesChanged() const { return this->vertices_changed; }

    /**
     * @brief returns true if colors were updated
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool isColorsChanged() const { return this->colors_changed; }

    /**
     * @brief Set the Vertices Changed object
     *
     * @param changed
     */
    void setVerticesChanged(bool changed) { this->vertices_changed = changed; }

    /**
     * @brief Set the Colors Changed object
     *
     * @param changed
     */
    void setColorsChanged(bool changed) { this->colors_changed = changed; }

    /**
     * @brief gets called when parameters have changed to recalculate the vertix positions and colors
     *
     */
    void recalculateVertices();

   private:
    static constexpr float DEFAULT_THICKNESS = 0.01;
    static constexpr float DEFAULT_SIZE = 0.01;
    static constexpr size_t DEFAULT_RESOLUTION = 20;

    const static logger::Logger LOGGER;

    // marker parameter
    float thickness{DEFAULT_THICKNESS};
    glm::dvec4 color{};  // rgb with alpha channel
    bool filled{false};

    glm::vec3 position{0, 0, 0};
    glm::dquat rotation = glm::dquat(1, 0, 0, 0);
    glm::vec3 scale{1, 1, 1};
    std::vector<glm::vec3> points{};
    std::vector<glm::vec3> vertices{};
    std::vector<glm::vec4> colors{};
    std::vector<glm::dvec4> input_colors{};
    bool closed_path{false};
    size_t id{0};
    std::string ns{"default"};
    bool limited_lifetime{false};

    bool vertices_changed{false};
    bool colors_changed{false};

    // circle
    unsigned int resolution{DEFAULT_RESOLUTION};
    float radius{1.0};
    // rect, arrow
    SizeVec2 size{DEFAULT_SIZE, DEFAULT_SIZE};
    // line
    glm::vec3 p1{};
    glm::vec3 p2{};
    // Custom Marker Strip Type
    Type2D strip_type{Type2D::ARROW2D};
    std::vector<glm::quat> rotations_y{};
    // heat map
    SizeVec2 heatmap_size{9, 6};
    SizeVec2 heatmap_data_size{1, 1};

    MType type{MType::LAST_MARKER_TYPE};
    Type2D type_2d{Type2D::LAST_MARKER_TYPE2D};

    /**
     * @brief calculates the vertex positions and colors for the circle
     *
     * @param radius
     * @param resolution
     * @param filled
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateCircle(float radius, unsigned int resolution, bool filled);

    /**
     * @brief calculates the vertex positions and colors for the rectangle
     *
     * @param size
     * @param filled
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateRect(SizeVec2 size, bool filled);

    /**
     * @brief calculates the vertex positions and colors for the line
     *
     * @param p1
     * @param p2
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateLine(const glm::vec3& p1, const glm::vec3& p2);

    /**
     * @brief calculates the vertex positions and colors for the 2d arrow
     *
     * @param size
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateArrow(SizeVec2 size);

    /**
     * @brief calculates the vertex positions and colors for the line strip
     *
     * @param points
     * @param closed
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateLineStrip(const std::vector<glm::vec3>& points, bool closed_path);

    /**
     * @brief calculates the vertex positions and colors for the custom strip
     *
     * @param points
     * @param type
     * @param size
     * @param circle_radius
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateCustomStrip(const std::vector<glm::vec3>& points, const Type2D& type, SizeVec2 size,
                                                float circle_radius);

    /**
     * @brief calculates the vertex positions and colors for the heatmap
     *
     * @param size
     * @param data_size
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateHeatmap(SizeVec2 size, SizeVec2 data_size);

    /**
     * @brief calculates the vertex positions and colors for the circular heatmap
     *
     * @param radius
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calculateCircularHeatmap(float radius);

    /**
     * @brief calculates extra vertices to give a line thickness
     *
     * @param points
     * @param thickness
     * @return std::vector<glm::vec3>
     */
    std::vector<glm::vec3> calcThickLines(std::vector<glm::vec3> points, float thickness);
};

}  // namespace luhsoccer::marker
