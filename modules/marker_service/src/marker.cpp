#include <utility>

#include "marker_service/marker.hpp"

namespace luhsoccer::marker {

bool Color::equals(const Color& c2) const {
    return (red == c2.red && green == c2.green && blue == c2.blue && alpha == c2.alpha);
}

bool Color::operator==(const Color& c) const {
    return this->red == c.red && this->green == c.green && this->blue == c.blue && this->alpha == c.alpha;
}

bool Color::operator!=(const Color& c) const { return !(c == *this); }

Color Color::hsv2Rgb(double hue, double saturation, double value, double alpha) {
    const double max_s = 100;
    const double max_v = 100;
    const double sixty = 60;
    const double sixty_x2 = sixty * 2;
    const double sixty_x3 = sixty * 3;
    const double sixty_x4 = sixty * 4;
    const double sixty_x5 = sixty * 5;

    saturation /= max_s;
    value /= max_v;
    double c = saturation * value;
    double x = c * (1 - abs(fmod(hue / sixty, 2) - 1));
    double m = value - c;
    double r = 0, g = 0, b = 0;
    if (hue >= 0 && hue < sixty) {
        r = c, g = x, b = 0;
    } else if (hue >= sixty && hue < sixty_x2) {
        r = x, g = c, b = 0;
    } else if (hue >= sixty_x2 && hue < sixty_x3) {
        r = 0, g = c, b = x;
    } else if (hue >= sixty_x3 && hue < sixty_x4) {
        r = 0, g = x, b = c;
    } else if (hue >= sixty_x4 && hue < sixty_x5) {
        r = x, g = 0, b = c;
    } else {
        r = c, g = 0, b = x;
    }

    return {r + m, g + m, b + m, alpha};
}

Color Color::random() {
    return Color{rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, 1};
}

Color Color::random(unsigned int seed) {
    std::srand(seed);
    return Color{rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, rand() / (double)RAND_MAX, 1};
}

Color Color::interpolate(Color c1, Color c2, double fraction) {
    // constraint value in [0,1]
    fraction = std::clamp(fraction, 0.0, 1.0);
    double red_interpolated = c1.red + (c2.red - c1.red) * fraction;
    double green_interpolated = c1.green + (c2.green - c1.green) * fraction;
    double blue_interpolated = c1.blue + (c2.blue - c1.blue) * fraction;

    return {red_interpolated, green_interpolated, blue_interpolated, c1.alpha};
}

Color Color::interpolateGradient(double fraction, std::vector<Color> gradient) {
    fraction = std::clamp(fraction, 0.0, 1.0);
    const double distance = 1.0 / (gradient.size() - 1);  // sampled equally distributed
    Color c1;
    Color c2;
    size_t i = 1;
    for (; i < gradient.size(); ++i) {
        if (fraction <= distance * i) {
            c1 = gradient[i - 1];
            c2 = gradient[i];
            break;
        }
    }
    // map fraction again from [old_min, old_max] to [0,1]
    double old_min = (i - 1) * distance;
    double old_max = i * distance;
    double mapped_fraction = (fraction - old_min) / (old_max - old_min);
    return interpolate(c1, c2, mapped_fraction);
}

Marker::operator std::string() const { return "/" + this->ns + "/" + std::to_string(this->id); }
[[nodiscard]] MType Marker::getType() const { return this->type; }

void Marker::setNs(std::string ns) { this->ns = std::move(ns); }
void Marker::setId(size_t id) { this->id = id; }
void Marker::setColor(Color color) { this->color = color; }
void Marker::setScale(ScaleVec3 scale) { this->scale = scale; }
void Marker::setLifetime(double lifetime) { this->lifetime = lifetime; }
void Marker::setFrameLocked(bool frame_locked) { this->frame_locked = frame_locked; }
void Marker::setHeight(double height) { this->height = height; }

transform::Position Marker::getPosition() { return this->position; }
double Marker::getHeight() { return this->height; }
std::string Marker::getNs() { return this->ns; }
size_t Marker::getId() { return id; }
Color Marker::getColor() { return this->color; }
ScaleVec3 Marker::getScale() { return this->scale; }
double Marker::getLifetime() { return this->lifetime; }
bool Marker::isFrameLocked() { return this->frame_locked; }
std::optional<RobotIdentifier> Marker::getRobotIdentifier() { return this->robot_handle; }

// marker 2d
double Marker::getThickness() { return this->thickness; }
std::vector<Color> Marker::getColors() { return this->colors; }
std::vector<Point> Marker::getPoints() { return this->points; }
// rect
SizeVec2 Marker::getRectSize() { return this->rect_size; }
bool Marker::isRectFilled() { return this->fill_rect; }
// line
Point Marker::getLineStart() { return this->start; }
Point Marker::getLineEnd() { return this->end; }
// circle
double Marker::getCircleRadius() { return this->radius; }
bool Marker::isCircleFilled() { return this->fill_circle; }
// arrow
SizeVec2 Marker::getArrowSize() { return this->arrow_size; }
// linestrip
bool Marker::isLinestripClosed() { return this->close_line_strip; }
// text
std::string Marker::getText() { return this->text; }
// custom strip type
[[nodiscard]] MType Marker::getMarkerStripType() const { return this->strip_type; }
std::vector<double> Marker::getRotations() { return this->phi_s; }
// heatmap
std::array<Color, 2> Marker::getHeatmapColors() { return heatmap_colors; }
Eigen::MatrixXd Marker::getHeatmapData() { return this->heatmap_data; }
SizeVec2 Marker::getHeatmapSize() { return this->heatmap_size; }
const std::array<double, 2> Marker::getHeatmapMinMax() { return this->min_max_values; }
void Marker::setHeatmapGradient(std::vector<Color> gradient) { this->heatmap_gradient = std::move(gradient); }
std::optional<std::vector<Color>> Marker::getHeatmapGradient() { return this->heatmap_gradient; }
// circular heatmap
std::vector<double> Marker::getCircularHeatmapData() { return this->circular_heatmap_data; }
std::array<Color, 2> Marker::getCircularHeatmapColors() { return heatmap_colors; }

void Marker::setLuhvizTimer(double expiration_time) { this->luhviz_exp_time = expiration_time; }
double Marker::getLuhvizExpirationTime() { return luhviz_exp_time; }

GoalBorder::GoalBorder(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::GOAL_BORDERS;
}

Robot::Robot(transform::Position position, const RobotIdentifier& robot_handle, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::ROBOT;
    this->robot_handle = robot_handle;
}

Ball::Ball(transform::Position position, std::string ns, size_t id) : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::BALL;
}

Cone::Cone(transform::Position position, std::string ns, size_t id) : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::CONE;
}

Cube::Cube(transform::Position position, std::string ns, size_t id) : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::CUBE;
}

Cylinder::Cylinder(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::CYLINDER;
}

Sphere::Sphere(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::SPHERE;
}

Torus::Torus(transform::Position position, std::string ns, size_t id) : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::TORUS;
}

Arrow::Arrow(transform::Position position, std::string ns, size_t id) : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::ARROW;
}

Suzanne::Suzanne(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::SUZANNE;
}

Text::Text(transform::Position position, std::string ns, size_t id, std::string text)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::TEXT;
    this->text = std::move(text);
}

void Text::setText(std::string text) { this->text = std::move(text); }

Marker2D::Marker2D(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {}

void Marker2D::setThickness(double thickness) { this->thickness = thickness; }

void Marker2D::setColors(std::vector<Color> colors) { this->colors = std::move(colors); }

Circle::Circle(transform::Position position, std::string ns, size_t id)
    : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::CIRCLE;
}

void Circle::setRadius(double radius) {
    if (this->radius != radius) {
        this->radius = radius;
        vertices_changed = true;
    }
}
void Circle::setFilled(bool filled) {
    if (this->fill_circle != filled) {
        this->fill_circle = filled;
        vertices_changed = true;
    }
}

Rect::Rect(transform::Position position, std::string ns, size_t id) : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::RECT;
}

void Rect::setSize(SizeVec2 size) {
    if (this->rect_size != size) {
        this->rect_size = size;
        vertices_changed = true;
    }
}
void Rect::setFilled(bool filled) {
    if (this->fill_rect != filled) {
        this->fill_rect = filled;
        vertices_changed = true;
    }
}

Line::Line(transform::Position position, std::string ns, size_t id) : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::LINE;
}

void Line::setLinePoints(Point p1, Point p2) {
    if (this->start != p1 || this->end != p2) {
        this->start = p1;
        this->end = p2;
        this->vertices_changed = true;
    }
}

Arrow2d::Arrow2d(transform::Position position, std::string ns, size_t id)
    : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::ARROW_2D;
}

void Arrow2d::setSize(SizeVec2 size) {
    if (this->arrow_size != size) {
        this->arrow_size = size;
        vertices_changed = true;
    }
}

LineStrip::LineStrip(transform::Position position, std::string ns, size_t id)
    : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::LINE_STRIP;
}

void LineStrip::setPathClosed(bool closed) {
    if (this->close_line_strip != closed) {
        this->close_line_strip = closed;
        vertices_changed = true;
    }
}

void LineStrip::setPoints(std::vector<Point> points) {
    if (this->points != points) {
        this->points = std::move(points);
        vertices_changed = true;
    }
}

Heatmap::Heatmap(transform::Position position, std::string ns, size_t id)
    : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::HEATMAP;
}

void Heatmap::setHeatmapColors(std::array<Color, 2> colors) {
    if (this->heatmap_colors != colors) {
        this->heatmap_colors = colors;
        colors_changed = true;
    }
}

void Heatmap::setHeatmapMinMax(double min, double max) {
    if (min != this->min_max_values[0] || max != this->min_max_values[1]) {
        this->min_max_values = {min, max};
        colors_changed = true;
    }
}

void Heatmap::setHeatmapSize(SizeVec2 heatmap_size) {
    if (heatmap_size != this->heatmap_size) {
        this->heatmap_size = heatmap_size;
        vertices_changed = true;
    }
}

void Heatmap::setHeatmapPoints(Eigen::MatrixXd points) {
    if (this->heatmap_data.rows() != points.rows() || this->heatmap_data.cols() != points.rows() ||
        (points - this->heatmap_data).norm() != 0) {
        this->heatmap_data = std::move(points);
        colors_changed = true;
    }
}

CircularHeatmap::CircularHeatmap(transform::Position position, std::string ns, size_t id)
    : Marker2D(std::move(position), std::move(ns), id) {
    this->type = MType::CIRCULAR_HEATMAP;
}

void CircularHeatmap::setCircularHeatmapColors(std::array<Color, 2> colors) {
    if (this->heatmap_colors != colors) {
        this->heatmap_colors = colors;
        colors_changed = true;
    }
}

void CircularHeatmap::setHeatmapMinMax(double min, double max) {
    if (min != this->min_max_values[0] || max != this->min_max_values[1]) {
        this->min_max_values = {min, max};
        colors_changed = true;
    }
}

void CircularHeatmap::setRadius(double radius) {
    if (this->radius != radius) {
        this->radius = radius;
        vertices_changed = true;
    }
}

void CircularHeatmap::setCircularHeatmapData(std::vector<double> data) {
    if (data != this->circular_heatmap_data) {
        this->circular_heatmap_data = std::move(data);
        colors_changed = true;
    }
}

CustomStrip::CustomStrip(transform::Position position, std::string ns, size_t id)
    : Marker(std::move(position), std::move(ns), id) {
    this->type = MType::CUSTOM_STRIP;
}

void CustomStrip::setMarkerStripType(MType type) { this->strip_type = type; }
void CustomStrip::setRotations(std::vector<double> phi_s) { this->phi_s = std::move(phi_s); }
void CustomStrip::setPoints(std::vector<Point> points) { this->points = std::move(points); }
void CustomStrip::setComponentParameter(std::variant<double, SizeVec2> radius_or_size) {
    // assign the given parameter
    if (std::holds_alternative<SizeVec2>(radius_or_size)) {
        this->arrow_size = std::get<SizeVec2>(radius_or_size);
        this->rect_size = std::get<SizeVec2>(radius_or_size);
    } else if (std::holds_alternative<double>(radius_or_size)) {
        this->radius = std::get<double>(radius_or_size);
    }
}

RobotInfo::RobotInfo(const RobotIdentifier& handle)
    : Marker(transform::Position{"none"}, "robot_info", handle.id), handle(handle) {
    this->type = MType::ROBOT_INFO;
}

void RobotInfo::addParam(std::string key, int param) { this->addParam(std::move(key), std::to_string(param)); }

void RobotInfo::addParam(std::string key, double param) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << param;
    this->addParam(std::move(key), stream.str());
}

void RobotInfo::addParam(std::string key, bool param) {
    std::string str = param ? "true" : "false";
    this->addParam(std::move(key), str);
}

void RobotInfo::addParam(std::string key, std::string param) {
    this->params.insert_or_assign(std::move(key), std::move(param));
}

void RobotInfo::setStatus(std::string status, Color color) {
    this->status = std::move(status);
    this->status_color = color;
}

void RobotInfo::setStatusTextColor(Color text_color) { this->status_text_color = text_color; }

[[nodiscard]] std::string RobotInfo::getStatus() const { return this->status; }

[[nodiscard]] marker::Color RobotInfo::getStatusColor() const { return this->status_color; }

[[nodiscard]] marker::Color RobotInfo::getStatusTextColor() const { return this->status_text_color; }

[[nodiscard]] RobotIdentifier RobotInfo::getRobotId() const { return this->handle; }

[[nodiscard]] std::map<std::string, std::string> RobotInfo::getParams() const { return this->params; }

LinePlot::LinePlot(std::string id, float time_window) : Marker(transform::Position{"none"}, std::move(id), 0), time_window(time_window), newest_sample(time_window) {}

LinePlot::LineHandle LinePlot::getLine(const std::string& name) {
    for(size_t i = 0; i < lines.size(); i++) {
        if(lines[i].getLabel() == name)
            return i;
    }
    this->lines.emplace_back(name);
    return this->lines.size() - 1;
}

void LinePlot::addPoint(LineHandle handle, float x, float y) {
    const float threshold = this->newest_sample - this->time_window;
    if(x < threshold)
        return;

    this->lines[handle].addPoint(x, y);
    if(x > this->newest_sample) {
        this->newest_sample = x;

        // erase outdated samples
        for(LineData& line : this->lines) {
            // remove all samples that were added before newest_sample - time_window
            size_t insert_idx = 0;
            for(size_t i = 0; i < line.data_x.size(); i++) {
                if(line.data_x[i] >= threshold) {
                    line.data_x[insert_idx] = line.data_x[i];
                    line.data_y[insert_idx] = line.data_y[i];
                    insert_idx++;
                }
            }
            line.data_x.resize(insert_idx);
            line.data_y.resize(insert_idx);
        }
    }
}

void LinePlot::LineData::addPoint(float x, float y) {
    size_t left = 0;
    size_t right = this->data_x.size();

    while(left < right) {
        const size_t mid = (left + right) / 2;
        const float mid_x = this->data_x[mid];
        if(mid_x < x) {
            left = mid + 1;
        }else{ // mid >= x
            right = mid;
        }
    }

    const size_t insert_idx = right;

    this->data_x.insert(this->data_x.begin() + insert_idx, x);
    this->data_y.insert(this->data_y.begin() + insert_idx, y);
    assert(this->data_x.size() == this->data_y.size());
}

const std::vector<float>& LinePlot::LineData::getDataX() const { return this->data_x; }
const std::vector<float>& LinePlot::LineData::getDataY() const { return this->data_y; }
const std::string& LinePlot::LineData::getLabel() const { return this->label; }
void LinePlot::LineData::clear() {
    data_x.clear();
    data_y.clear();
}
LinePlot::LineData::LineData(const std::string& label) : label(label) {}

size_t LinePlot::getLineCount() const { return this->lines.size(); }
LinePlot::LineData& LinePlot::operator[](size_t idx) { return this->lines[idx]; }
const LinePlot::LineData& LinePlot::operator[](size_t idx) const { return this->lines[idx]; }

float LinePlot::getLeftLimit() const {
    float left = std::numeric_limits<float>::max();
    for(const LineData& line : this->lines) {
        if(line.data_x.size() > 0 && line.data_x.front() < left)
            left = line.data_x.front();
    }

    if(left == std::numeric_limits<float>::max())
        left = 0;
    return left;
}
float LinePlot::getRightLimit() const {
    return this->newest_sample;
}

void LinePlot::swap(LinePlot& other) {
    std::swap(lines, other.lines);
    this->newest_sample = other.newest_sample;
    this->time_window = other.time_window;
}


}  // namespace luhsoccer::marker
