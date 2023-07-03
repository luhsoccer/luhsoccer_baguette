#include "marker_service/marker_2d_impl.hpp"

namespace luhsoccer::marker {

const logger::Logger Marker2DImpl::LOGGER = logger::Logger("marker_2d");

std::vector<glm::vec3> Marker2DImpl::calcThickLines(std::vector<glm::vec3> points, float thickness) {
    // ensure that there are enough colors
    if (!this->input_colors.empty()) {
        for (size_t i = this->input_colors.size(); i < points.size(); i++) {
            this->input_colors.emplace_back(1, 1, 1, 1);
        }
    }

    std::vector<glm::vec3> vertices;
    this->colors.clear();
    for (size_t i = 0; i < points.size(); ++i) {
        size_t last_index = i == 0 ? points.size() - 1 : i - 1;
        size_t next_index = i + 1 < points.size() ? i + 1 : 0;
        size_t next_next_index = i + 2 < points.size() ? i + 2 : (i + 2) - points.size();

        glm::vec3 p0 = points[last_index];
        glm::vec3 p1 = points[i];
        glm::vec3 p2 = points[next_index];
        glm::vec3 p3 = points[next_next_index];

        // normalize direction
        glm::vec3 dir0 = glm::normalize(p1 - p0);
        glm::vec3 dir1 = glm::normalize(p2 - p1);
        glm::vec3 dir2 = glm::normalize(p3 - p2);

        // get normal
        glm::vec3 normal0{-dir0.z, 0, dir0.x};
        glm::vec3 normal1{-dir1.z, 0, dir1.x};
        glm::vec3 normal2{-dir2.z, 0, dir2.x};

        // MITER
        glm::vec3 miter1 = glm::normalize(normal0 + normal1);
        glm::vec3 miter2 = glm::normalize(normal1 + normal2);

        float an1 = glm::dot(miter1, normal1);
        float an2 = glm::dot(miter2, normal2);
        if (an1 == 0) an1 = 1;
        if (an2 == 0) an2 = 1;
        float length_1 = thickness * an1;

        // triangle 1
        vertices.emplace_back(p1 - miter1 * length_1);
        vertices.emplace_back(p1 + miter1 * length_1);
        vertices.emplace_back(p2 - miter2 * length_1);

        // triangle 2
        vertices.emplace_back(p2 - miter2 * length_1);
        vertices.emplace_back(p1 + miter1 * length_1);
        vertices.emplace_back(p2 + miter2 * length_1);

        // add color per vertice
        if (!this->input_colors.empty()) {
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
        }
    }

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateCircle(float radius, unsigned int resolution, bool filled) {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> points;
    for (unsigned int i = 0; i < resolution; i++) {
        // Angle between each side in radians
        float ang = glm::pi<float>() * 2 / resolution * i;

        // Offset from center of point
        glm::vec3 offset = glm::vec3(cos(ang), 0.0, -sin(ang)) * radius;
        points.emplace_back(offset);
    }

    // create vertices from points (add thickness)
    if (filled) {
        glm::vec3 center = std::reduce(points.begin(), points.end());
        center /= points.size();

        // ensure that a filled shape is not multicolored
        if (!this->input_colors.empty()) {
            this->color = this->colors[0];
            this->colors.clear();
        }

        // for every circle point add a triangle to fill the circle
        for (size_t i = 0; i < points.size() - 1; i++) {
            vertices.emplace_back(points[i]);
            vertices.emplace_back(points[i + 1]);
            vertices.emplace_back(center);
        }
        // last triangle
        if (points.size() > 0) {
            vertices.emplace_back(points[points.size() - 1]);
            vertices.emplace_back(points[0]);
            vertices.emplace_back(center);
        }
    } else {
        vertices = calcThickLines(points, this->thickness / 2);
    }
    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateRect(SizeVec2 size, bool filled) {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> points;

    float w2 = size.size_x / 2, h2 = size.size_y / 2;
    points.clear();
    points.emplace_back(-w2, 0, -h2);
    points.emplace_back(-w2, 0, h2);
    points.emplace_back(w2, 0, h2);
    points.emplace_back(w2, 0, -h2);

    // create vertices from points (add thickness)
    this->colors.clear();
    if (filled) {
        if (!this->input_colors.empty()) {
            this->color = this->input_colors[0];
        }
        vertices.emplace_back(points[0]);
        vertices.emplace_back(points[1]);
        vertices.emplace_back(points[2]);
        vertices.emplace_back(points[0]);
        vertices.emplace_back(points[2]);
        vertices.emplace_back(points[3]);
    } else {
        vertices = calcThickLines(points, this->thickness);
    }

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateLine(const glm::vec3& p1, const glm::vec3& p2) {
    std::vector<glm::vec3> vertices;

    // normalize direction
    glm::vec3 dir = glm::normalize(p2 - p1);

    // get normal
    glm::vec3 normal{-dir.z, 0, dir.x};
    glm::vec3 offset = normal * (this->thickness / 2);

    // triangle 1
    vertices.emplace_back(p1 - offset);
    vertices.emplace_back(p1 + offset);
    vertices.emplace_back(p2 - offset);

    // triangle 2
    vertices.emplace_back(p2 + offset);
    vertices.emplace_back(p2 - offset);
    vertices.emplace_back(p1 + offset);

    if (!this->input_colors.empty()) this->color = this->input_colors[0];

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateArrow(SizeVec2 size) {
    if (!this->input_colors.empty()) {
        this->color = this->input_colors[0];
    }

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> points;

    points.emplace_back(0, 0, -size.size_x / 2);
    points.emplace_back(0, 0, 0);

    glm::vec3 p0 = points[0];
    glm::vec3 p1 = points[1];
    glm::vec3 dir = glm::normalize(p1 - p0);

    // get normal
    glm::vec3 normal{-dir.z, 0, dir.x};
    glm::vec3 offset = normal * (this->thickness / 2);

    vertices.clear();

    // triangle 1
    vertices.emplace_back(p0 - offset);
    vertices.emplace_back(p0 + offset);
    vertices.emplace_back(p1 - offset);

    // triangle 2
    vertices.emplace_back(p1 + offset);
    vertices.emplace_back(p1 - offset);
    vertices.emplace_back(p0 + offset);

    // filled triangle
    vertices.emplace_back(-size.size_y / 2, 0, 0);
    vertices.emplace_back(0, 0, size.size_x / 2);
    vertices.emplace_back(size.size_y / 2, 0, 0);

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateLineStrip(const std::vector<glm::vec3>& points, bool closed_path) {
    if (this->points.empty()) return {};

    std::vector<glm::vec3> vertices;

    // ensure that there are enough colors
    if (!this->input_colors.empty()) {
        for (size_t i = this->input_colors.size(); i < points.size(); i++) {
            this->input_colors.emplace_back(glm::vec4{1, 1, 1, 1});
        }
    }

    this->colors.clear();
    size_t point_size = points.size();
    for (size_t i = 0; i < point_size; i++) {
        glm::vec3 p0 = points[i];
        glm::vec3 p1 = points[i];
        glm::vec3 p2 = points[i];
        glm::vec3 p3 = points[i];

        if (point_size > 1) {
            size_t last_index = i == 0 ? point_size - 1 : i - 1;
            size_t next_index = i + 1 < point_size ? i + 1 : 0;
            size_t next_next_index =
                i + 2 < point_size ? i + 2 : std::max(std::min((i + 2) - point_size, point_size), (size_t)0);

            p0 = points[last_index];
            p2 = points[next_index];
            p3 = points[next_next_index];
        }

        // normalize direction
        glm::vec3 dir0 = glm::normalize(p1 - p0);
        glm::vec3 dir1 = glm::normalize(p2 - p1);
        glm::vec3 dir2 = glm::normalize(p3 - p2);

        // get normal
        glm::vec3 normal0{-dir0.z, 0, dir0.x};
        glm::vec3 normal1{-dir1.z, 0, dir1.x};
        glm::vec3 normal2{-dir2.z, 0, dir2.x};

        glm::vec3 offset = normal1 * (this->thickness / 2);

        if (i < point_size - 1 || closed_path) {
            // triangle 1
            vertices.emplace_back(p1 - offset);
            vertices.emplace_back(p1 + offset);
            vertices.emplace_back(p2 - offset);

            // triangle 2
            vertices.emplace_back(p2 - offset);
            vertices.emplace_back(p1 + offset);
            vertices.emplace_back(p2 + offset);
        }

        // add color per vertice
        if (!this->input_colors.empty()) {
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
            this->colors.emplace_back(this->input_colors[i]);
        }
    }

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateCustomStrip(const std::vector<glm::vec3>& points, const Type2D& type,
                                                          SizeVec2 size = {DEFAULT_SIZE},
                                                          float circle_radius = DEFAULT_SIZE) {
    if (type != Type2D::ARROW2D && type != Type2D::CIRCLE2D && type != Type2D::RECT2D) {
        LOG_DEBUG(LOGGER, "this type of marker is not supported for CustomStrip: {}", static_cast<int>(type));
        return {};
    }

    // ensure there are rotations set for each component (=points.size)
    for (size_t i = this->rotations_y.size(); i < points.size(); ++i) {
        this->rotations_y.emplace_back(glm::identity<glm::quat>());
    }

    std::vector<glm::vec3> vertices;
    this->colors.clear();

    // add the vertices of the selected marker strip type for every point
    int i = 0;
    for (glm::vec3 p : points) {
        std::vector<glm::vec3> verts;
        switch (type) {
            case Type2D::ARROW2D:
                // calculate arrow vertices
                verts = calculateArrow(size);
                for (glm::vec3 v : verts) {
                    vertices.emplace_back(v * rotations_y[i] + p);
                }
                break;
            case Type2D::CIRCLE2D:
                // calculate circle vertices
                verts = calculateCircle(circle_radius, DEFAULT_RESOLUTION, true);
                for (glm::vec3 v : verts) {
                    vertices.emplace_back(v + p);
                }
                break;
            case Type2D::RECT2D:
                // calculate circle vertices
                verts = calculateRect(size, true);
                for (glm::vec3 v : verts) {
                    vertices.emplace_back(v + p);
                }
                break;
            default:
                LOG_DEBUG(LOGGER, "this type of marker is not supported for CustomStrip: {}", static_cast<int>(type));
                break;
        }
        ++i;
    }

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateHeatmap(SizeVec2 size, SizeVec2 data_size) {
    const size_t num_verts = 6;
    auto num_points = static_cast<size_t>(data_size.size_x * data_size.size_y * num_verts);

    std::vector<glm::vec3> vertices;
    this->colors.clear();

    // ensure that there are enough colors
    for (size_t i = this->input_colors.size(); i < num_points; i++) {
        this->input_colors.emplace_back(1, 1, 1, 1);
    }

    // calc the half distance between the points
    double distance_x = size.size_x / data_size.size_x;
    double distance_y = size.size_y / data_size.size_y;
    double x_2 = distance_x / 2;
    double y_2 = distance_y / 2;
    double offset_x = size.size_x / 2 - x_2;
    double offset_y = size.size_y / 2 - y_2;

    int index = 0;
    // take the points and create a square ("superpixel") for every point
    const float y_offset = 0.001f;
    for (int x = 0; x < data_size.size_x; ++x) {
        for (int y = 0; y < data_size.size_y; ++y) {
            // 2 triangles with p in the middle
            glm::vec3 p{x * distance_x - offset_x, y_offset, y * distance_y - offset_y};
            vertices.emplace_back(p + glm::vec3{-x_2, 0, -y_2});
            vertices.emplace_back(p + glm::vec3{x_2, 0, y_2});
            vertices.emplace_back(p + glm::vec3{x_2, 0, -y_2});

            vertices.emplace_back(p + glm::vec3{x_2, 0, y_2});
            vertices.emplace_back(p + glm::vec3{-x_2, 0, -y_2});
            vertices.emplace_back(p + glm::vec3{-x_2, 0, y_2});

            // colors
            for (size_t i = 0; i < num_verts; ++i) {
                this->colors.emplace_back(this->input_colors[index]);
            }
            index++;
        }
    }

    return vertices;
}

std::vector<glm::vec3> Marker2DImpl::calculateCircularHeatmap(float radius) {
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> points;

    const unsigned int num_segments = this->input_colors.size();
    for (unsigned int i = 0; i < num_segments; i++) {
        // Angle between each side in radians
        float ang = glm::pi<float>() * 2 / num_segments * i;

        // Offset from center of point
        glm::vec3 offset = glm::vec3(cos(ang), 0.0, -sin(ang)) * radius;
        points.emplace_back(offset);
    }

    // get center point
    glm::vec3 center = std::reduce(points.begin(), points.end());
    center /= points.size();
    // for every circle point add a triangle to fill the circle
    for (size_t i = 0; i < points.size() - 1; i++) {
        vertices.emplace_back(points[i]);
        vertices.emplace_back(points[i + 1]);
        vertices.emplace_back(center);
    }
    // last triangle
    if (points.size() > 0) {
        vertices.emplace_back(points[points.size() - 1]);
        vertices.emplace_back(points[0]);
        vertices.emplace_back(center);
    }

    // append the colors
    this->colors.clear();
    for (glm::vec4 col : this->input_colors) {
        this->colors.emplace_back(col);
        this->colors.emplace_back(col);
        this->colors.emplace_back(col);
    }

    return vertices;
}

void Marker2DImpl::recalculateVertices() {
    switch (this->type_2d) {
        case Type2D::LINE2D:
            this->vertices = calculateLine(this->p1, this->p2);
            break;
        case Type2D::RECT2D:
            this->vertices = calculateRect(this->size, this->filled);
            break;
        case Type2D::ARROW2D:
            this->vertices = calculateArrow(this->size);
            break;
        case Type2D::CIRCLE2D:
            this->vertices = calculateCircle(this->radius, this->resolution, this->filled);
            break;
        case Type2D::LINE_STRIP2D:
            this->vertices = calculateLineStrip(this->points, this->closed_path);
            break;
        case Type2D::CUSTOM_STRIP2D:
            this->vertices = calculateCustomStrip(this->points, this->strip_type, this->size, this->radius);
            break;
        case Type2D::HEATMAP2D:
            this->vertices = calculateHeatmap(this->heatmap_size, this->heatmap_data_size);
            break;
        case Type2D::CIRCULAR_HEATMAP2D:
            this->vertices = calculateCircularHeatmap(this->radius);
            break;
        default:
            LOG_WARNING(LOGGER, "MarkerType {} from namespace {} is no valid 2DMarker Type",
                        static_cast<int>(this->type), this->ns);
    }
}

}  // namespace luhsoccer::marker
