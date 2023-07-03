#pragma once

#include <map>
#include <queue>
#include <utility>
#include <variant>

#include "transform/transform.hpp"

namespace luhsoccer::marker {

enum class MType {
    GOAL_BORDERS,
    ROBOT,
    BALL,
    FRAME,
    CONE,
    CUBE,
    CYLINDER,
    SPHERE,
    TORUS,
    ARROW,
    SUZANNE,
    TEXT,
    LINE,
    RECT,
    ARROW_2D,
    CIRCLE,
    LINE_STRIP,
    CUSTOM_STRIP,
    HEATMAP,
    CIRCULAR_HEATMAP,
    INFO,
    ROBOT_INFO,
    LAST_MARKER_TYPE,  // this needs to stay the last enum type!
};

// Marker Types
enum class Type3D {
    GOAL_BORDERS3D,
    ROBOT3D,
    BALL3D,
    FRAME3D,
    CONE3D,
    CUBE3D,
    CYLINDER3D,
    SPHERE3D,
    TORUS3D,
    ARROW3D,
    SUZANNE3D,
    TEXT3D,
    LAST_MARKER_TYPE3D,  // this needs to stay the last enum type!
};

enum class Type2D {
    LINE2D,
    RECT2D,
    ARROW2D,
    CIRCLE2D,
    LINE_STRIP2D,
    CUSTOM_STRIP2D,
    HEATMAP2D,
    CIRCULAR_HEATMAP2D,
    LAST_MARKER_TYPE2D,  // this needs to stay the last enum type!
};

/**
 * @brief represents color as rgb but can also be created with hsv values
 *
 */
class Color {
   public:
    /**
     * @brief Construct a new Color object from rgb values
     *
     * @param red red component range: [0,1] or [0,255]
     * @param green green component range: [0,1] or [0,255]
     * @param blue blue component range: [0,1] or [0,255]
     * @param alpha alpha componen range: [0,1]
     * @param normalized true defines rgb values in range [0,1]
     */
    constexpr Color(double red = DEFAULT_RED, double green = DEFAULT_GREEN, double blue = DEFAULT_BLUE,
                    double alpha = DEFAULT_ALPHA)
        : red(red), green(green), blue(blue), alpha(alpha) {
        if (red > 1.0) this->red /= COLOR_MAX;
        if (green > 1.0) this->green /= COLOR_MAX;
        if (blue > 1.0) this->blue /= COLOR_MAX;
        if (red < 0) this->red = 0;
        if (green < 0) this->green = 0;
        if (blue < 0) this->blue = 0;
    }

    [[nodiscard]] bool equals(const Color& c2) const;

    bool operator==(const Color& c) const;
    bool operator!=(const Color& c) const;

    double red;
    double green;
    double blue;
    double alpha;

    // NOLINTBEGIN(readability-identifier-naming)
    static constexpr Color RED(double alpha = 1) { return {1, 0, 0, alpha}; }
    static constexpr Color GREEN(double alpha = 1) { return {0, 1, 0, alpha}; }
    static constexpr Color BLUE(double alpha = 1) { return {0, 0, 1, alpha}; }
    static constexpr Color YELLOW(double alpha = 1) { return {1, 1, 0, alpha}; }
    static constexpr Color ORANGE(double alpha = 1) { return {1, 0.6, 0.1, alpha}; }
    static constexpr Color LIGHT_GREEN(double alpha = 1) { return {0.5, 1, 0.1, alpha}; }
    static constexpr Color LIGHT_BLUE(double alpha = 1) { return {0.3, 0.6, 1, alpha}; }
    static constexpr Color PURPLE(double alpha = 1) { return {0.6, 0.2, 0.8, alpha}; }
    static constexpr Color PINK(double alpha = 1) { return {1, 0.4, 1, alpha}; }
    static constexpr Color GREY(double alpha = 1) { return {0.4, 0.4, 0.4, alpha}; }
    static constexpr Color LIGHT_GREY(double alpha = 1) { return {0.7, 0.7, 0.7, alpha}; }
    static constexpr Color WHITE(double alpha = 1) { return {1, 1, 1, alpha}; }
    static constexpr Color BLACK(double alpha = 1) { return {0, 0, 0, alpha}; }
    // NOLINTEND

    /**
     * @brief converts hsv colors to rgb
     *
     * @param hue hue component
     * @param saturation saturation component
     * @param value value component
     * @param alpha alpha component
     * @return a Color object
     */
    static Color hsv2Rgb(double hue, double saturation, double value, double alpha);

    static Color random();

    static Color random(unsigned int seed);

    static Color interpolate(Color c1, Color c2, double fraction);

    static Color interpolateGradient(double fraction, std::vector<Color> gradient = {PINK(), BLUE(), LIGHT_BLUE(),
                                                                                     GREEN(), YELLOW(), RED()});

   private:
    constexpr static double DEFAULT_RED = 1;
    constexpr static double DEFAULT_GREEN = 0.0784;
    constexpr static double DEFAULT_BLUE = 0.5765;
    constexpr static double DEFAULT_ALPHA = 1;
    constexpr static double COLOR_MAX = 255;
    constexpr static double DEGREE_MAX = 360;
};

/**
 * @brief represents a 3d scale
 *
 */
struct ScaleVec3 {
    ScaleVec3() = default;
    ScaleVec3(double x, double y, double z) : x(x), y(y), z(z) {}
    ScaleVec3(double uniform_scale) : x(uniform_scale), y(uniform_scale), z(uniform_scale) {}
    double x{1};
    double y{1};
    double z{1};
};

/**
 * @brief represents a 2d size
 *
 */
struct SizeVec2 {
    SizeVec2(double size_uniform) : size_x(size_uniform), size_y(size_uniform) {}
    SizeVec2(double size_x, double size_y) : size_x(size_x), size_y(size_y) {}

    bool operator==(const SizeVec2& s) const { return this->size_x == s.size_x && this->size_y == s.size_y; }
    bool operator!=(const SizeVec2& s) const { return !(s == *this); }

    double size_x{0};
    double size_y{0};
    std::string toString() { return "{size_x=" + std::to_string(size_x) + ", size_y=" + std::to_string(size_y) + "}"; }
};

/**
 * @brief represents a 3d point
 *
 */
struct Point {
    Point() = default;
    Point(double x, double y, double z = 0) : x(x), y(y), z(z) {}

    bool operator==(const Point& p1) const { return p1.x == this->x && p1.y == this->y && p1.z == this->z; }
    bool operator!=(const Point& p1) const { return !(p1 == *this); }

    double x{0};
    double y{0};
    double z{0};
};

struct LinePoint {
    LinePoint(double x, double y) : x(x), y(y) {}
    double x{0};
    double y{0};
};

// --------------3d MARKERS -------------
class Marker {
   public:
    virtual ~Marker() = default;

    operator std::string() const;
    [[nodiscard]] MType getType() const;

    // setter
    void setNs(std::string ns);
    void setId(size_t id);
    void setColor(Color color);
    void setScale(ScaleVec3 scale);
    void setLifetime(double lifetime);
    void setFrameLocked(bool frame_locked);
    void setHeight(double height);

    // getter
    transform::Position getPosition();
    double getHeight();
    std::string getNs();
    size_t getId();
    Color getColor();
    ScaleVec3 getScale();
    double getLifetime();
    bool isFrameLocked();
    std::optional<RobotIdentifier> getRobotIdentifier();

    // marker 2d
    double getThickness();
    std::vector<Color> getColors();
    std::vector<Point> getPoints();
    // rect
    SizeVec2 getRectSize();
    bool isRectFilled();
    // line
    Point getLineStart();
    Point getLineEnd();
    // circle
    double getCircleRadius();
    bool isCircleFilled();
    // arrow
    SizeVec2 getArrowSize();
    // linestrip
    bool isLinestripClosed();
    // text
    std::string getText();
    // custom strip type
    [[nodiscard]] MType getMarkerStripType() const;
    std::vector<double> getRotations();
    // heatmap
    std::array<Color, 2> getHeatmapColors();
    Eigen::MatrixXd getHeatmapData();
    SizeVec2 getHeatmapSize();
    const std::array<double, 2> getHeatmapMinMax();
    void setHeatmapGradient(std::vector<Color> gradient);
    std::optional<std::vector<Color>> getHeatmapGradient();
    // circular heatmap
    std::vector<double> getCircularHeatmapData();
    std::array<Color, 2> getCircularHeatmapColors();

   protected:
    constexpr static double DEFAULT_THICKNESS{0.01};
    constexpr static double DEFAULT_SIZE{0.01};

    /**
     * @brief Construct a new Marker object
     *
     * @param header header of the transform
     * @param ns the namespace
     * @param id the unique identifier inside the namespace
     */
    Marker(transform::Position position, std::string ns, size_t id)
        : position(std::move(position)), ns(std::move(ns)), id(id) {}

    /** defines the type of mesh */
    MType type{MType::LAST_MARKER_TYPE};
    double luhviz_exp_time{0};

    transform::Position position;
    double height{0};
    std::string ns{"default_ns"};
    size_t id{0};
    Color color{};
    ScaleVec3 scale{};
    double lifetime{0};
    bool frame_locked{false};

    // to track the robot (if its a robot marker)
    std::optional<RobotIdentifier> robot_handle{std::nullopt};

    // 2d Marker
    double thickness{DEFAULT_THICKNESS};
    std::vector<Color> colors{};
    std::vector<Point> points{};
    // text
    std::string text{"no_text"};
    // rect
    SizeVec2 rect_size{DEFAULT_SIZE, DEFAULT_SIZE};
    bool fill_rect{false};
    // line
    Point start{};
    Point end{};
    // circle
    double radius{DEFAULT_SIZE / 2};
    bool fill_circle{false};
    // arrow
    SizeVec2 arrow_size{DEFAULT_SIZE, DEFAULT_SIZE};
    // linestrip
    bool close_line_strip{false};
    // customStrip
    MType strip_type{};
    std::vector<double> phi_s{};
    // heatmap
    std::array<Color, 2> heatmap_colors{Color::WHITE(), Color::WHITE()};
    std::optional<std::vector<Color>> heatmap_gradient{std::nullopt};
    Eigen::MatrixXd heatmap_data{Eigen::MatrixXd::Random(1, 1)};
    SizeVec2 heatmap_size{1, 1};
    std::array<double, 2> min_max_values{0, 0};
    // circular heatmap
    std::vector<double> circular_heatmap_data{1, 1};
    // infomarker
    std::string info_text{"value"};
    std::string info_value{"empty"};

    // track if vector data changes to save performance
    bool vertices_changed{false};
    bool colors_changed{false};

   private:
    void setLuhvizTimer(double expiration_time);
    double getLuhvizExpirationTime();
    friend class MarkerService;
};

/**
 * @brief displays the robot mesh
 *
 */
class GoalBorder : public Marker {
   public:
    GoalBorder(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays the robot mesh
 *
 */
class Robot : public Marker {
   public:
    Robot(transform::Position position, const RobotIdentifier& robot_handle, std::string ns = "default_ns",
          size_t id = 0);
};
/**
 * @brief displays the ball mesh
 *
 */
class Ball : public Marker {
   public:
    Ball(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d cone
 *
 */
class Cone : public Marker {
   public:
    Cone(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d cube
 *
 */
class Cube : public Marker {
   public:
    Cube(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d cylinder
 *
 */
class Cylinder : public Marker {
   public:
    Cylinder(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d sphere
 *
 */
class Sphere : public Marker {
   public:
    Sphere(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d torus
 *
 */
class Torus : public Marker {
   public:
    Torus(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a 3d arrow
 *
 */
class Arrow : public Marker {
   public:
    Arrow(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays the suzanne mesh
 *
 */
class Suzanne : public Marker {
   public:
    Suzanne(transform::Position position, std::string ns = "default_ns", size_t id = 0);
};
/**
 * @brief displays a text mesh
 *
 */
class Text : public Marker {
   public:
    Text(transform::Position position, std::string ns = "default_ns", size_t id = 0, std::string text = "no_text");

    void setText(std::string text);
};
// ---------------------------------------

class Marker2D : public Marker {
   protected:
    Marker2D(transform::Position position, std::string ns = "default_ns", size_t id = 0);

   public:
    void setThickness(double thickness);
    void setColors(std::vector<Color> colors);
};

/**
 * @brief displays a 2d circle
 *
 */
class Circle : public Marker2D {
   public:
    Circle(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setRadius(double radius);

    void setFilled(bool filled);
};

/**
 * @brief displays a 2d rectangle
 *
 */
class Rect : public Marker2D {
   public:
    Rect(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setSize(SizeVec2 size);

    void setFilled(bool filled);
};

/**
 * @brief displays a 2d line
 *
 */
class Line : public Marker2D {
   public:
    Line(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setLinePoints(Point p1, Point p2);
};

/**
 * @brief displays a 2d arrow
 *
 */
class Arrow2d : public Marker2D {
   public:
    Arrow2d(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setSize(SizeVec2 size);
};

/**
 * @brief displays a 2d strip of lines
 *
 */
class LineStrip : public Marker2D {
   public:
    LineStrip(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setPathClosed(bool closed);

    void setPoints(std::vector<Point> points);
};

/**
 * @brief displays a heatmap from a 2d matrix of data
 *
 */
class Heatmap : public Marker2D {
   public:
    Heatmap(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setHeatmapColors(std::array<Color, 2> colors);

    void setHeatmapMinMax(double min, double max);

    void setHeatmapSize(SizeVec2 heatmap_size);

    void setHeatmapPoints(Eigen::MatrixXd points);
};

/**
 * @brief displays a circular heatmap from a vector of data
 *
 */
class CircularHeatmap : public Marker2D {
   public:
    CircularHeatmap(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setCircularHeatmapColors(std::array<Color, 2> colors);

    void setHeatmapMinMax(double min, double max);

    void setRadius(double radius);

    void setCircularHeatmapData(std::vector<double> data);
};

/**
 * @brief displays a sequence of Markers of a chosen type at provided points
 *
 */
class CustomStrip : public Marker {
   public:
    CustomStrip(transform::Position position, std::string ns = "default_ns", size_t id = 0);

    void setMarkerStripType(MType type);
    void setRotations(std::vector<double> phi_s);
    void setPoints(std::vector<Point> points);
    void setComponentParameter(std::variant<double, SizeVec2> radius_or_size);
};

/**
 * @brief holds text information to be used for debugging and beeing later displayed in the game info section
 *
 */
class Info : public Marker {
   public:
    Info(std::string ns, size_t id) : Marker(transform::Position{"none"}, std::move(ns), id) {
        this->type = MType::INFO;
    }

    void set(const std::string& name, std::string& value) { this->setValue(name, value); }

    void set(const std::string& name, int value) { this->setValue(name, std::to_string(value)); }

    void set(const std::string& name, double value) { this->setValue(name, std::to_string(value)); }

    void set(const std::string& name, bool value) { this->setValue(name, value ? "true" : "false"); }

    [[nodiscard]] std::string getInfoText() const { return this->info_text; }
    [[nodiscard]] std::string getInfoValue() const { return this->info_value; }

   private:
    void setValue(std::string name, std::string value) {
        this->info_text = std::move(name);
        this->info_value = std::move(value);
    }
};

/**
 * @brief The RobotInfo Marker can contain different usefull information for displaying robot status (hardware)
 *
 */
class RobotInfo : public Marker {
   public:
    RobotInfo(const RobotIdentifier& handle);

    /**
     * @brief adds or asigns a param (converted with std::to_string) to the map
     *
     * @param key
     * @param param
     */
    void addParam(std::string key, int param);

    /**
     * @brief adds or asigns a param (converted with std::to_string) to the map
     *
     * @param key
     * @param param
     */
    void addParam(std::string key, double param);

    /**
     * @brief adds or asigns a param (converted with std::to_string) to the map
     *
     * @param key
     * @param param
     */
    void addParam(std::string key, bool param);

    /**
     * @brief adds or assigns a string param to the map
     *
     * @param key
     * @param param
     */
    void addParam(std::string key, std::string param);

    /**
     * @brief Set the status of the robot
     *
     * @param new_status
     */
    void setStatus(std::string new_status, Color color);

    /**
     * @brief Set the text color
     *
     * @param text_color
     */
    void setStatusTextColor(Color text_color);

    /**
     * @brief Get the Status of the robot
     *
     * @return RobotStatus
     */
    [[nodiscard]] std::string getStatus() const;

    /**
     * @brief returns the status background color
     *
     * @return marker::Color
     */
    [[nodiscard]] marker::Color getStatusColor() const;

    /**
     * @brief return the text color
     *
     * @return marker::Color
     */
    [[nodiscard]] marker::Color getStatusTextColor() const;

    /**
     * @brief Get the Robot Id
     *
     * @return RobotIdentifier&
     */
    [[nodiscard]] RobotIdentifier getRobotId() const;

    /**
     * @brief Get the Params
     *
     * @return std::unordered_map<std::string, std::string> const
     */
    [[nodiscard]] std::map<std::string, std::string> getParams() const;

   private:
    RobotIdentifier handle;
    std::map<std::string, std::string> params{};
    std::string status{};
    Color status_color{Color::RED()};
    Color status_text_color{Color::WHITE()};
};

// utility structure for realtime plot
class LinePlot : public Marker {
   public:
    using LineHandle = size_t;
    LinePlot(std::string id, float time_window = 15);

    void addPoint(float x, float y);


    void swap(LinePlot& other);

    class LineData {
       public:
           LineData(const std::string& label);
           const std::string& getLabel() const;
            
           const std::vector<float>& getDataX() const;
           const std::vector<float>& getDataY() const;

           void clear();
       private:
        void addPoint(float x, float y);
        std::string label;
        std::vector<float> data_x;
        std::vector<float> data_y;
        friend LinePlot;
    };

    LineHandle getLine(const std::string& name);

    size_t getLineCount() const;
    LineData& operator[](size_t idx);
    const LineData& operator[](size_t idx) const;

    void addPoint(LineHandle handle, float x, float y);


    float getLeftLimit() const;
    float getRightLimit() const;
   private:
   
    float time_window;
    float newest_sample;
    std::vector<LineData> lines;
};

}  // namespace luhsoccer::marker
