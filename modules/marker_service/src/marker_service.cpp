#include "marker_service/marker_service.hpp"
#include "marker_service/marker_2d_impl.hpp"
#include "marker_service/marker_impl.hpp"
#include "marker_service/luhviz_impl.hpp"

namespace luhsoccer::marker {

const logger::Logger MarkerService::LOGGER = logger::Logger("marker_service");

std::unique_ptr<LuhvizMarkers> MarkerService::getLuhvizMarkers() {
    this->updateTransforms();
    std::lock_guard<std::mutex> lock_luhviz(this->luhviz_marker_mutex);
    return std::make_unique<LuhvizMarkers>(*this->luhviz_markers);
}

MarkerService::MarkerService() : luhviz_markers(std::make_unique<LuhvizMarkers>()){};
MarkerService::~MarkerService() = default;

void MarkerService::displayMarker(Info marker) {
    std::string key = marker.getNs();

    std::lock_guard<std::mutex> guard(info_mutex);

    if (this->infos.find(key) == this->infos.end()) {
        std::map<std::string, Info> info{{marker.getInfoText(), std::move(marker)}};
        this->infos.insert(std::make_pair(std::move(key), std::move(info)));
    } else {
        auto& info = this->infos.at(key);
        info.insert_or_assign(marker.getInfoText(), std::move(marker));
    }
}

void MarkerService::displayMarker(RobotInfo marker) {
    std::string key = getKey(marker.getNs(), marker.getId());

    std::lock_guard<std::mutex> guard(robot_info_mutex);
    auto it = this->robot_infos.find(key);
    if (it != this->robot_infos.end()) {
        // if found, add data (overwrites value if key exists)
        for (const auto& param : marker.getParams()) {
            it->second.addParam(param.first, param.second);
        }
        for (const auto& [key, badge] : marker.getBadges()) {
            it->second.addBadge(key, badge);
        }
    } else {
        // insert it
        this->robot_infos.insert_or_assign(std::move(key), std::move(marker));
    }
}

void MarkerService::removeRobotInfoMarker(RobotIdentifier robot) {
    const std::lock_guard robot_lock(this->robot_info_mutex);

    RobotInfo info(robot);
    std::string key = getKey(info.getNs(), info.getId());
    auto it = this->robot_infos.find(key);

    if (it != this->robot_infos.end()) {
        this->robot_infos.erase(it);
    }
}

void MarkerService::displayMarker(LinePlot plot) {
    std::lock_guard<std::mutex> guard(line_plots_mutex);
    auto it = this->line_plots.find(plot.getNs());
    // insert or overwrite lineplot
    if (it != this->line_plots.end()) {
        // if found, swap queue data (update values)
        it->second.swap(plot);
    } else {
        // insert it
        this->line_plots.insert_or_assign(plot.getNs(), std::move(plot));
    }
}

void MarkerService::displayMarker(Marker marker) {
    // update display time of the marker, =0 means infinite
    if (marker.getLifetime() == 0) {
        marker.setLuhvizTimer(0);
    } else {
        marker.setLuhvizTimer(time::now().asSec() + marker.getLifetime());
    }

    // insert or update marker in map
    std::string key = getKey(marker.getNs(), marker.getId());

    std::lock_guard<std::mutex> lock(this->marker_mutex);
    this->markers.insert_or_assign(std::move(key), std::move(marker));
}

void MarkerService::deleteMarker(std::string ns, size_t id) {
    std::string key = getKey(ns, id);
    std::lock_guard<std::mutex> lock(this->marker_mutex);
    this->markers.erase(key);

    std::lock_guard<std::mutex> lock_luhviz(this->luhviz_marker_mutex);
    luhviz_markers->markers.erase(key);
    luhviz_markers->markers2d.erase(key);
}

void MarkerService::updateTransforms() {
    std::lock_guard<std::mutex> lock(this->marker_mutex);

    // remove markers whoem lifetime has ended
    for (auto it = this->markers.begin(); it != this->markers.end();) {
        if (it->second.getLuhvizExpirationTime() == 0) {
            ++it;
            continue;
        }

        if (it->second.getLuhvizExpirationTime() < time::now().asSec()) {
            it = this->markers.erase(it);
        } else {
            ++it;
        }
    }

    std::lock_guard<std::mutex> lock_luhviz(this->luhviz_marker_mutex);
    // check if gdp has already set the real worldmodel
    if (this->real_wm.has_value()) {
        luhviz_markers->markers.clear();
        luhviz_markers->markers2d.clear();

        // go through all markers in map and update their pose from worldmodel
        for (auto& marker : this->markers) {
            std::optional<Eigen::Affine2d> pose = marker.second.getPosition().getCurrentPosition(this->real_wm.value());

            if (pose.has_value()) {
                // auto vel = this->real_wm.value()->getVelocity(marker.second.getPosition().getFrame());
                // if (vel.has_value()) {
                //     convertMarkerForLuhviz(marker.second, pose.value(), vel.value());
                // } else {
                convertMarkerForLuhviz(marker.second, pose.value(), transform::Velocity{});
                // }
            }
        }

        // go through all transforms in worldmodel and add a frame marker for every tf
        const std::vector<std::string>& frames = this->real_wm->get()->getAllTransformFrames();
        size_t index = 0;
        for (const auto& frame : frames) {
            auto tf = this->real_wm->get()->getTransform(frame);
            if (tf.has_value()) {
                // modify marker for luhviz to display them
                createFrameMarkerForLuhviz(tf.value(), index);
            }
            ++index;
        }
    }

    // move the info markers to luhviz marker struct
    // TODO: dont copy that
    const std::lock_guard info_lock(this->info_mutex);
    luhviz_markers->info_markers = infos;

    const std::lock_guard robot_lock(this->robot_info_mutex);
    luhviz_markers->robot_info_markers = this->robot_infos;

    const std::lock_guard plots_lock(this->line_plots_mutex);
    luhviz_markers->plots = this->line_plots;
}

void MarkerService::createFrameMarkerForLuhviz(const transform::Transform& tf, size_t index) {
    Eigen::Vector2d position = tf.transform.translation();
    double angle = Eigen::Rotation2Dd(tf.transform.rotation()).angle();

    const std::string frame_ns = "Frames";
    std::string key = frame_ns + "_" + tf.header.child_frame;

    // add new frame marker
    MarkerImpl m_impl;
    // luhviz coordinate system is different, x,y,height is mapped to x=x, y=height and z = -y
    m_impl.setPosition(glm::dvec2(position.x(), -position.y()));
    m_impl.setRotation(angle);
    m_impl.setType(MType::FRAME);
    m_impl.setType3D(Type3D::FRAME3D);
    m_impl.setNs(frame_ns);
    m_impl.setText(tf.header.child_frame);
    m_impl.setId(index);
    luhviz_markers->markers.insert_or_assign(std::move(key), std::move(m_impl));

    constexpr int ID_OFFSET = 1000;
    constexpr double TEXT_SCALE = 0.4;
    const std::string frame_names_ns = "Frame_names";
    std::string key_name = frame_names_ns + "_" + tf.header.child_frame;

    transform::Transform copy = tf;
    Eigen::Vector2d offset(0, -0.1);
    copy.transform *= Eigen::Translation2d(offset);
    auto text_pos = copy.transform.translation();
    auto text_angle = Eigen::Rotation2Dd(copy.transform.rotation()).angle();
    // add new text displaying the frame name
    MarkerImpl text;
    text.setType(MType::TEXT);
    text.setType3D(Type3D::TEXT3D);
    text.setNs(frame_names_ns);
    text.setId(ID_OFFSET + index);
    text.setText(tf.header.child_frame);
    text.setScale(TEXT_SCALE);
    text.setColor(toVec(Color::WHITE()));
    text.setPosition({text_pos.x(), 0, -text_pos.y()});
    text.setRotation(text_angle);
    luhviz_markers->markers.insert_or_assign(std::move(key_name), std::move(text));
}

void MarkerService::convertMarkerForLuhviz(Marker& marker, Eigen::Affine2d pose, const transform::Velocity& vel) {
    // luhviz coordinate system is different, x,y,height is mapped to x=x, y=height and z = -y
    Eigen::Vector3d position = {pose.translation().x(), marker.getHeight(), -pose.translation().y()};
    double angle = Eigen::Rotation2Dd(pose.rotation()).angle();

    std::string key = getKey(marker.getNs(), marker.getId());

    if (marker.getType() == MType::GOAL_BORDERS_DIVA || marker.getType() == MType::GOAL_BORDERS_DIVB ||
        marker.getType() == MType::ROBOT || marker.getType() == MType::BALL || marker.getType() == MType::FRAME ||
        marker.getType() == MType::CONE || marker.getType() == MType::CUBE || marker.getType() == MType::CYLINDER ||
        marker.getType() == MType::SPHERE || marker.getType() == MType::TORUS || marker.getType() == MType::ARROW ||
        marker.getType() == MType::SUZANNE || marker.getType() == MType::TEXT) {
        // populate marker 3d
        MarkerImpl m_impl{};
        m_impl.setPosition(glm::dvec3(position.x(), position.y(), position.z()));
        m_impl.setVelocity(vel.velocity.x(), vel.velocity.y(), vel.velocity.z());
        m_impl.setRotation(angle);
        m_impl.setScale({marker.getScale().x, marker.getScale().y, marker.getScale().z});
        m_impl.setColor(toVec(marker.getColor()));
        m_impl.setType(marker.getType());
        m_impl.setType3D(this->get3DMarkerType(marker.getType()));
        m_impl.setNs(marker.getNs());
        m_impl.setId(marker.getId());
        if (marker.getType() == MType::ROBOT && marker.getRobotIdentifier().has_value()) {
            m_impl.setRobotIdentifier(marker.getRobotIdentifier().value());
        }
        if (marker.getType() == MType::TEXT) {
            m_impl.setText(marker.getText());
        }

        luhviz_markers->markers.insert_or_assign(std::move(key), std::move(m_impl));

    } else if (marker.getType() == MType::LINE || marker.getType() == MType::RECT ||
               marker.getType() == MType::CIRCLE || marker.getType() == MType::ARROW_2D ||
               marker.getType() == MType::LINE_STRIP || marker.getType() == MType::CUSTOM_STRIP ||
               marker.getType() == MType::HEATMAP || marker.getType() == MType::CIRCULAR_HEATMAP) {
        //
        // populate marker 2d
        Marker2DImpl m2d_impl;
        m2d_impl.setType(marker.getType());
        m2d_impl.setType2D(this->get2DMarkerType(marker.getType()));
        m2d_impl.setPosition(Point(position.x(), position.y(), position.z()));
        m2d_impl.setRotation(static_cast<float>(angle));
        m2d_impl.setScale({marker.getScale().x, marker.getScale().y, marker.getScale().z});
        m2d_impl.setColor(marker.getColor());
        if (marker.type != MType::HEATMAP) m2d_impl.setPointColors(marker.getColors());
        m2d_impl.setLineThickness(static_cast<float>(marker.getThickness()));
        m2d_impl.setNs(marker.getNs());
        m2d_impl.setId(marker.getId());
        m2d_impl.setLimitedLifetime(marker.getLifetime() > 0);

        if (marker.getType() == MType::LINE) {
            m2d_impl.setPoints(marker.getLineStart(), marker.getLineEnd());
        } else if (marker.getType() == MType::RECT) {
            m2d_impl.setSize(marker.getRectSize());
            m2d_impl.setFilled(marker.isRectFilled());
        } else if (marker.getType() == MType::CIRCLE) {
            m2d_impl.setRadius(static_cast<float>(marker.getCircleRadius()));
            m2d_impl.setFilled(marker.isCircleFilled());
        } else if (marker.getType() == MType::ARROW_2D) {
            m2d_impl.setSize(marker.getArrowSize());
        } else if (marker.getType() == MType::LINE_STRIP) {
            m2d_impl.setPoints(marker.getPoints());
            m2d_impl.setClosedPath(marker.isLinestripClosed());
        } else if (marker.getType() == MType::CUSTOM_STRIP) {
            m2d_impl.setStripType(this->get2DMarkerType(marker.getMarkerStripType()));
            m2d_impl.setPoints(marker.getPoints());
            m2d_impl.setRotationsY(marker.getRotations());

            // set strip-type specific parameter
            if (marker.getMarkerStripType() == MType::ARROW_2D) {
                m2d_impl.setSize(marker.getArrowSize());
            } else if (marker.getMarkerStripType() == MType::RECT) {
                m2d_impl.setSize(marker.getRectSize());
            } else if (marker.getMarkerStripType() == MType::CIRCLE) {
                m2d_impl.setRadius(static_cast<float>(marker.getCircleRadius()));
            }
        } else if (marker.getType() == MType::HEATMAP) {
            // convert heatmap data
            std::vector<Color> colors;
            Eigen::MatrixXd data = marker.getHeatmapData();
            Color cold = marker.getHeatmapColors()[0];
            Color hot = marker.getHeatmapColors()[1];
            SizeVec2 data_size{static_cast<double>(data.rows()), static_cast<double>(data.cols())};
            double min_value = marker.getHeatmapMinMax()[0];
            double max_value = marker.getHeatmapMinMax()[1];
            if (marker.getHeatmapMinMax()[0] == marker.getHeatmapMinMax()[1]) {
                // else get default min/max values from provided data
                min_value = data.minCoeff();
                max_value = data.maxCoeff();
            }

            for (int x = 0; x < data_size.size_x; ++x) {
                for (int y = 0; y < data_size.size_y; ++y) {
                    // interpolate the color between cold color and hot color or use a provided (or default) gradient
                    double fraction = (data(x, y) - min_value) / (max_value - min_value);
                    if (cold.equals(hot)) {
                        auto gradient = marker.getHeatmapGradient();
                        if (gradient.has_value()) {
                            // interpolate on provided gradient
                            colors.emplace_back(Color::interpolateGradient(fraction, gradient.value()));
                        } else {
                            // interpolate on default
                            colors.emplace_back(Color::interpolateGradient(fraction));
                        }
                    } else {
                        // interpolate linear between cold and hot color
                        colors.emplace_back(Color::interpolate(cold, hot, fraction));
                    }
                }
            }
            // set the interpolated colors
            m2d_impl.setPointColors(colors);
            m2d_impl.setHeatmapSize(marker.getHeatmapSize(), data_size);
        } else if (marker.getType() == MType::CIRCULAR_HEATMAP) {
            const std::vector<double> data = marker.getCircularHeatmapData();
            const Color cold = marker.getCircularHeatmapColors()[0];
            const Color hot = marker.getCircularHeatmapColors()[1];
            double min = marker.getHeatmapMinMax()[0];
            double max = marker.getHeatmapMinMax()[1];
            if (min == max) {
                // else get default min/max values from provided data
                const auto [a, b] = std::minmax_element(std::begin(data), std::end(data));
                min = *a;
                max = *b;
            }
            std::vector<Color> interpolated_colors;
            for (const double& value : data) {
                double fraction = (value - min) / (max - min);
                if (cold.equals(hot)) {
                    auto gradient = marker.getHeatmapGradient();
                    if (gradient.has_value()) {
                        interpolated_colors.emplace_back(Color::interpolateGradient(fraction, gradient.value()));
                    } else {
                        interpolated_colors.emplace_back(Color::interpolateGradient(fraction));
                    }
                } else {
                    interpolated_colors.emplace_back(Color::interpolate(cold, hot, fraction));
                }
            }
            m2d_impl.setPointColors(interpolated_colors);
            m2d_impl.setRadius(static_cast<float>(marker.getCircleRadius()));
        }

        if (marker.vertices_changed || marker.colors_changed) {
            m2d_impl.setVerticesChanged(marker.vertices_changed);
            m2d_impl.setColorsChanged(marker.colors_changed);
            m2d_impl.recalculateVertices();
            marker.vertices_changed = false;
            marker.colors_changed = false;
        }
        luhviz_markers->markers2d.insert_or_assign(std::move(key), std::move(m2d_impl));
    } else {
        LOGGER.warning("marker type was not set correctly (was {})", static_cast<int>(marker.getType()));
    }
}

std::string MarkerService::getKey(const std::string& ns, size_t id) { return ns + "_" + std::to_string(id); }

void MarkerService::setup() {}
void MarkerService::stop() {}

}  // namespace luhsoccer::marker
