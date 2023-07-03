#include "include/render_view.hpp"
#include "marker_service/marker_impl.hpp"
#include "marker_service/marker_2d_impl.hpp"
#include "marker_service/luhviz_impl.hpp"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui_internal.h"

using namespace glm;

namespace luhsoccer::luhviz {

void RenderView::init(GLFWwindow* w) {
    this->window = w;

    // Enable depth test
    this->context->enable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    this->context->depthFunc(GL_LESS);
    // Cull triangles which normal is not towards the camera
    this->context->enable(GL_CULL_FACE);
    // enable alpha
    this->context->enable(GL_BLEND);
    this->context->blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // enable MSAA antialiasing
    this->context->enable(GL_MULTISAMPLE);

    constexpr float CLEAR_COLOR = 0.175f;
    this->context->clearColor(CLEAR_COLOR, CLEAR_COLOR, CLEAR_COLOR, 1.0f);

    // // build and compile texture shader programs
    this->shader_program.create(ver_shader, frag_shader);

    // load textures
    this->frame_gltexture = std::make_shared<GLTexture>(frame_texture, false);

    // load 3d models
    this->models.insert(std::make_pair(marker::Type3D::FRAME3D, Model{frame_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::GOAL_BORDERS3D, Model{goalborders_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::BALL3D, Model{ball_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::ROBOT3D, Model{robot_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::CONE3D, Model{cone_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::CUBE3D, Model{cube_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::CYLINDER3D, Model{cylinder_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::SPHERE3D, Model{sphere_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::TORUS3D, Model{torus_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::ARROW3D, Model{arrow_obj, shader_program}));
    this->models.insert(std::make_pair(marker::Type3D::SUZANNE3D, Model{suzanne_obj, shader_program}));

    // Get a handle for our uniforms
    this->mvp_id = GLUniformMat4vf{shader_program.getId(), "mvp"};
    this->mv_id = GLUniformMat4vf{shader_program.getId(), "mv"};
    this->m_id = GLUniformMat4vf{shader_program.getId(), "m"};
    this->v_id = GLUniformMat4vf{shader_program.getId(), "v"};
    this->light_id = GLUniform3f{shader_program.getId(), "light_pos_worldspace"};
    this->color_id = GLUniform4f{shader_program.getId(), "color"};
    this->tex_id = GLUniform1i{shader_program.getId(), "texture_sampler"};
    this->multicolor_id = GLUniform1i{shader_program.getId(), "multicolor"};
    this->type_id = GLUniform1i{shader_program.getId(), "type"};

    // render the markers offscreen to a 4xsampled texture to later render that in a single texture to display that in
    // imgui
    setupMSAAOffscreenRendering();

    loadMouseCursorImages();
}

void RenderView::loadMouseCursorImages() {
    this->teleport_cursor.create(p_teleport, true, true);
    this->setpoint_cursor.create(p_setpoint, true, true);
    this->setdir_cursor.create(p_setdir, true, true);
    this->execskill_cursor.create(p_execskill, true, true);
    this->ballslingshot_cursor.create(p_slingshot, true, true);
}

void RenderView::cleanMSAABuffer() {
    glDeleteTextures(1, &texture_color_buffer_multisampled);
    glDeleteTextures(1, &screen_texture);
    glDeleteFramebuffers(1, &framebuffer);
    glDeleteRenderbuffers(1, &rbo);
    glDeleteFramebuffers(1, &intermediate_fbo);
}

void RenderView::setupMSAAOffscreenRendering() {
    // The framebuffer, which regroups 0, 1, or more textures,
    //     and0 or 1 depth buffer.
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // generate texture (multisampled for MSAA)
    glGenTextures(1, &texture_color_buffer_multisampled);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, texture_color_buffer_multisampled);
    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, msaa_sample_count, GL_RGB, static_cast<int>(window_size.x),
                            static_cast<int>(window_size.y), GL_TRUE);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE,
                           texture_color_buffer_multisampled, 0);

    // The depth buffer (multisampled)
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, msaa_sample_count, GL_DEPTH24_STENCIL8,
                                     static_cast<int>(window_size.x), static_cast<int>(window_size.y));
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

    // Error check
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR(logger, "Framebuffer not complete error while initialising {}",
                  glCheckFramebufferStatus(GL_FRAMEBUFFER));
        return;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // configure second framebuffer for the result
    glGenFramebuffers(1, &intermediate_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, intermediate_fbo);
    // create a color attachment texture
    glGenTextures(1, &screen_texture);
    glBindTexture(GL_TEXTURE_2D, screen_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, static_cast<GLsizei>(window_size.x), static_cast<GLsizei>(window_size.y), 0,
                 GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, screen_texture, 0);
    // Poor filtering
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        LOG_ERROR(logger, "Framebuffer not complete error while initialising {}",
                  glCheckFramebufferStatus(GL_FRAMEBUFFER));
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderView::update(marker::LuhvizMarkers& luhviz_markers) {
    // save robot data
    const size_t num_robots = 16;
    this->proxy.robot_data.clear();
    for (auto& m : luhviz_markers.markers) {
        if (m.second.getType3D() == marker::Type3D::ROBOT3D && m.second.getRobotIdentifier().has_value()) {
            // save robot positions
            size_t id = m.second.getRobotIdentifier().value().isAlly()
                            ? m.second.getRobotIdentifier().value().id
                            : m.second.getRobotIdentifier().value().id + num_robots;

            this->proxy.getRobotMarkers().insert_or_assign(id, m.second);
            this->proxy.robot_data.insert_or_assign(id, RobotData{id,
                                                                  m.second.getNs(),
                                                                  {m.second.getPosition().x, m.second.getPosition().z},
                                                                  m.second.getRotation2D()});
        }
        if (m.second.getType3D() == marker::Type3D::BALL3D && m.second.getNs().compare("ball") == 0) {
            this->proxy.ball_data = BallData{m.second.getNs(), {m.second.getPosition().x, m.second.getPosition().z}};
        }
    }

    // display camera focus point
    constexpr double FOCUS_POINT_SCALE = 0.2;
    constexpr glm::dvec4 FOCUS_POINT_COLOR = {1, 1, 1, .5f};
    marker::MarkerImpl camera_target{marker::Type3D::SPHERE3D, controls.getCameraTargetPosition()};
    camera_target.setScale(FOCUS_POINT_SCALE);
    camera_target.setColor(FOCUS_POINT_COLOR);
    luhviz_markers.markers.insert_or_assign("camera_target", camera_target);

    // display frame and direction arrow while choosing the point
    constexpr double TINY_FRAME_SCALE = 0.5;
    // display choosen points
    int i = 0;
    for (const auto& p : this->last_skill_points) {
        marker::MarkerImpl skillpoint_frame{marker::Type3D::FRAME3D, {p.x, 0, p.y}};
        skillpoint_frame.setScale(TINY_FRAME_SCALE);
        skillpoint_frame.setRotation(this->angle);
        skillpoint_frame.setType(marker::MType::FRAME);
        luhviz_markers.markers.insert_or_assign("skillpoint_frame_" + std::to_string(i), skillpoint_frame);
        ++i;
    }
    if (this->proxy.getManipulationMode() == ManipulationMode::ADD_POINT ||
        this->proxy.getManipulationMode() == ManipulationMode::ADD_DIRECTION) {
        // display point choose orientation arrow
        if (this->skillpoint_direction.has_value() && this->skillpoint_position.has_value()) {
            // display orientation vector as arrow
            marker::MarkerImpl direction_arrow{marker::Type3D::ARROW3D,
                                               {this->skillpoint_position->x, 0, this->skillpoint_position->y}};
            direction_arrow.setRotation(angle);
            direction_arrow.setColor(toVec(marker::Color::RED()));
            luhviz_markers.markers.insert_or_assign("direction_arrow", direction_arrow);
        }
    }

    if (this->proxy.getManipulationMode() == ManipulationMode::BALL_SLINGSHOT) {
        const auto p = this->start_ball_pos;
        if (p.has_value() && this->slingshot_velocity.has_value()) {
            auto pos = p->translation();
            marker::MarkerImpl direction_arrow{marker::Type3D::ARROW3D, {pos.x(), 0, -pos.y()}};
            glm::vec2 vel = {this->slingshot_velocity->x(), this->slingshot_velocity->y()};
            double len = glm::length(vel) / 2.0;
            vel = glm::normalize(vel);
            float angle = 0;
            if (vel.y >= 0) {
                angle = glm::angle(vel, glm::vec2{1, 0});
            } else {
                angle = -glm::angle(vel, glm::vec2{1, 0});
            }
            direction_arrow.setRotation(angle);
            direction_arrow.setColor(toVec(marker::Color::ORANGE()));
            direction_arrow.setScale({len, 1.0, 1.0});
            luhviz_markers.markers.insert_or_assign("direction_arrow", direction_arrow);
        }
    }

    // display mouse click selected skill points as tiny frames (2d position and rotation)
    for (const auto& p : this->proxy.getTDPoints()) {
        marker::MarkerImpl skillpoint_frame{marker::Type3D::FRAME3D, {p.x, 0, -p.y}};
        skillpoint_frame.setScale(TINY_FRAME_SCALE);
        skillpoint_frame.setRotation(p.z);
        luhviz_markers.markers.insert_or_assign("skillpoint_frame", skillpoint_frame);
    }

    // display selected robot, if any
    const glm::dvec4 selected_robot_color = toVec(marker::Color::RED(0.4));
    if (this->proxy.getSelectedRobot().has_value()) {
        bool ally = this->proxy.getSelectedRobot().value().isAlly();
        size_t key =
            ally ? this->proxy.getSelectedRobot().value().id : this->proxy.getSelectedRobot().value().id + num_robots;
        marker::MarkerImpl selected_robot_marker{
            marker::Type3D::SPHERE3D, this->proxy.getRobotMarkers()[key].getPosition() + robot_collider_center};
        selected_robot_marker.setColor(selected_robot_color);
        luhviz_markers.markers.insert_or_assign("selected_robot_marker", selected_robot_marker);
    }

    // display selected robot 2
    const glm::dvec4 selected_robot_color2 = toVec(marker::Color::BLUE(0.4));
    if (this->proxy.getSelectedRobot2().has_value()) {
        bool ally = this->proxy.getSelectedRobot2().value().isAlly();
        size_t key =
            ally ? this->proxy.getSelectedRobot2().value().id : this->proxy.getSelectedRobot2().value().id + num_robots;
        marker::MarkerImpl selected_robot_marker{
            marker::Type3D::SPHERE3D, this->proxy.getRobotMarkers()[key].getPosition() + robot_collider_center};
        selected_robot_marker.setColor(selected_robot_color2);
        luhviz_markers.markers.insert_or_assign("selected_robot_marker2", selected_robot_marker);
    }

    // display selected skill related robots, if any
    const glm::dvec4 selected_related_robot_color = toVec(marker::Color::PINK(0.4));
    for (const auto& rel_robot : this->proxy.getSelectedRelatedRobots()) {
        if (rel_robot.has_value() && rel_robot != EMPTY_IDENTIFIER) {
            size_t key = rel_robot.value().id;
            if (rel_robot->isEnemy()) key += num_robots;
            marker::MarkerImpl selected_related_robot_marker{
                marker::Type3D::SPHERE3D, this->proxy.getRobotMarkers()[key].getPosition() + robot_collider_center};
            selected_related_robot_marker.setColor(selected_related_robot_color);
            luhviz_markers.markers.insert_or_assign("selected_related_robot_marker", selected_related_robot_marker);
        }
    }

    // create / update meshes

    // 3d markers from marker service
    this->render_meshes.clear();
    this->render_texts.clear();
    const auto vp_matrix = this->projection_matrix * this->view_matrix;
    // glm::dvec3 camera_position = this->controls.getCameraPosition();
    // const auto camera_right_perspective = -glm::dvec3{this->controls.getCameraRight()};
    // const auto camera_right_orthographic = -glm::dvec3{this->controls.getCameraRight()};
    for (auto& [key, mimpl] : luhviz_markers.markers) {
        auto model_it = this->models.find(mimpl.getType3D());

        if (mimpl.getType() == marker::MType::TEXT) {
            // saving texts for later rendering with msdfgl lib
            auto text_it = this->texts.find(key);

            if (text_it != this->texts.end()) {
                // if text exists apply changed properties
                // TODO: text face to camera option?
                // glm::dvec3 direction = glm::normalize(camera_position - mimpl.getPosition());
                // glm::dvec3 normal;
                // if (this->controls.isPerspectiveView()) {
                //     normal = glm::normalize(glm::cross(camera_right_perspective, direction));
                // } else {
                //     normal = glm::normalize(glm::cross(camera_right_orthographic, direction));
                // }
                // glm::dquat rotation = glm::quatLookAt(direction, normal);

                // auto m = getModelMatrix(mimpl.getPosition(), rotation, mimpl.getScale());

                auto rotation_offset_x = glm::angleAxis(glm::radians(90.0), glm::dvec3{1, 0, 0});
                auto rotation = mimpl.getRotation() * rotation_offset_x;

                auto m_matrix = getModelMatrix(mimpl.getPosition(), rotation, mimpl.getScale());
                auto mvp = vp_matrix * m_matrix;
                text_it->second->setMVP(mvp);
                text_it->second->setColor(mimpl.getColor());
                text_it->second->setText(mimpl.getText());
                this->render_texts.emplace_back(text_it->second);
            } else {
                // create new textmesh
                this->texts[key] = std::make_shared<TextMesh>(this->context);
            }

        } else {
            auto mesh_it = this->meshes.find(key);

            if (mesh_it != this->meshes.end()) {
                // mesh is already created, just update the properties

                // update data
                auto m_matrix = getModelMatrix(mimpl.getPosition(), mimpl.getRotation(), mimpl.getScale());
                glm::mat4 mvp = vp_matrix * m_matrix;
                mesh_it->second->setMVP(mvp);
                mesh_it->second->setMV(this->view_matrix * m_matrix);
                mesh_it->second->setM(m_matrix);
                mesh_it->second->setV(this->view_matrix);
                mesh_it->second->setLightPos(this->light_pos);
                mesh_it->second->setColor(mimpl.getColor());
                mesh_it->second->setOrigin(mimpl.getPosition());

                this->render_meshes.emplace_back(mesh_it->second);

            } else {
                // mesh needs to be created from model
                if (model_it != this->models.end()) {
                    // mesh needs a model to render
                    if (mimpl.getType() == marker::MType::FRAME) {
                        // model := Frame with texture
                        this->meshes[key] = std::make_shared<Mesh>(model_it->second, this->context, this->mvp_id,
                                                                   this->mv_id, this->m_id, this->v_id, this->light_id,
                                                                   this->type_id, this->frame_gltexture, this->tex_id);
                    } else {
                        // model := colored shape
                        // set the mvp matrix
                        this->meshes[key] = std::make_shared<Mesh>(model_it->second, this->context, this->mvp_id,
                                                                   this->mv_id, this->m_id, this->v_id, this->light_id,
                                                                   this->type_id, mimpl.getColor(), this->color_id);
                    }
                } else {
                    // LOG_DEBUG(logger, "MarkerType {} could not be added as mesh",
                    // static_cast<int>(mimpl.getType()));
                }
            }
        }
    }

    // 2d markers from marker service
    for (auto& [key, m2dimpl] : luhviz_markers.markers2d) {
        auto mesh_it = this->meshes.find(key);

        if (mesh_it != this->meshes.end()) {
            // mesh is already created

            // update data
            mesh_it->second->setOrigin(m2dimpl.getPosition());
            auto m_matrix = getModelMatrix(m2dimpl.getPosition(), m2dimpl.getRotation(), m2dimpl.getScale());
            glm::mat4 mvp = vp_matrix * m_matrix;
            mesh_it->second->setMVP(mvp);
            mesh_it->second->setMV(this->view_matrix * m_matrix);
            mesh_it->second->setM(m_matrix);
            mesh_it->second->setV(this->view_matrix);
            mesh_it->second->setLightPos(this->light_pos);
            mesh_it->second->setColor(m2dimpl.getColor());

            // check if verts or colors had been changed, if so, update them
            if (m2dimpl.isVerticesChanged()) {
                mesh_it->second->setVertices(m2dimpl.getVertices());
                m2dimpl.setVerticesChanged(false);
                mesh_it->second->setColors(m2dimpl.getColors());
                m2dimpl.setColorsChanged(false);
            }
            if (m2dimpl.isColorsChanged()) {
                mesh_it->second->setColors(m2dimpl.getColors());
                m2dimpl.setColorsChanged(false);
            }

            this->render_meshes.emplace_back(mesh_it->second);

        } else {
            // mesh needs to be created
            this->meshes[key] = std::make_shared<Mesh>(
                this->context, this->mvp_id, this->mv_id, this->m_id, this->v_id, this->light_id, this->type_id,
                m2dimpl.getVertices(), m2dimpl.getColors(), m2dimpl.getColor(), this->color_id, this->multicolor_id);

            // set the mvp matrix
            auto mesh = this->meshes.at(key);
            auto m_matrix = getModelMatrix(m2dimpl.getPosition(), m2dimpl.getRotation(), m2dimpl.getScale());
            auto mvp = vp_matrix * m_matrix;
            mesh->setMVP(mvp);
            mesh->setOrigin(m2dimpl.getPosition());

            this->render_meshes.emplace_back(mesh);
        }
    }
}

void RenderView::render() {
    if (window_size_changed) {
        window_size_changed = false;
        cleanMSAABuffer();
        setupMSAAOffscreenRendering();
    }

    auto window_width = static_cast<GLsizei>(window_size.x);
    auto window_height = static_cast<GLsizei>(window_size.y);

    // render everything after this to the framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glViewport(0, 0, window_width, window_height);

    this->context->clear();

    // use shader program
    this->context->useProgram(this->shader_program);

    // render opaque meshes
    this->transparent_meshes.clear();
    for (const auto& mesh : this->render_meshes) {
        if (mesh != nullptr) {
            if (mesh->isOpaque()) {
                mesh->render();
            } else {
                this->transparent_meshes.emplace_back(mesh);
            }
        }
    }

    // sort transparent meshes (front to back)
    const glm::vec3 cam_position = this->controls.getCameraPosition();
    auto mesh_sort_function = [&cam_position](const std::shared_ptr<Mesh>& m1, const std::shared_ptr<Mesh>& m2) {
        auto distance1 = glm::distance(m1->getOrigin(), cam_position);
        auto distance2 = glm::distance(m2->getOrigin(), cam_position);
        return distance1 < distance2;
    };
    std::sort(this->transparent_meshes.begin(), this->transparent_meshes.end(), mesh_sort_function);

    // render transparent meshes
    for (const auto& mesh : this->transparent_meshes) {
        mesh->render();
    }

    bool render_text_on_top = this->proxy.getConfigBool(luhviz_config_name, "render_text_on_top");
    if (!render_text_on_top) {
        // render texts in the scene
        for (const auto& rt : this->render_texts) {
            rt->render();
        }
    }
    // blit the multisample buffer into the normal buffer
    glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, intermediate_fbo);
    glBlitFramebuffer(0, 0, window_width, window_height, 0, 0, window_width, window_height, GL_COLOR_BUFFER_BIT,
                      GL_NEAREST);

    // render texts on top of the rendered image
    if (render_text_on_top) {
        for (const auto& rt : this->render_texts) {
            rt->render();
        }
    }

    // reactivate normal rendering to screen
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, window_width, window_height);
}

void RenderView::renderToTexture() {
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNavFocus |
                                    ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
                                    ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("RenderView", nullptr, window_flags);
    ImGui::PopStyleColor();

    ImVec2 wsize = ImGui::GetWindowSize();
    ImGui::Image((ImTextureID)screen_texture, wsize, ImVec2(0, 1), ImVec2(1, 0));

    // update frame buffer for changed window size
    if (wsize.x != window_size.x || wsize.y != window_size.y) {
        window_size = vec2(wsize.x, wsize.y);
        viewport = window_size;
        window_size_changed = true;
    }

    updateMouseControl();

    const float margin_x = 15;
    const ImVec2 pos = ImVec2(margin_x, 40);
    // change 3d View / Top view in controls.cpp
    ImGui::SetCursorPos(pos);
    if (ImGui::Button(change_view_button_text.c_str())) {
        controls.setView3d(!this->controls.isPerspectiveView());
        change_view_button_text = this->controls.isPerspectiveView() ? "3D View" : "Top View";
    }

    // reset to standard view
    ImGui::SetCursorPosX(margin_x);
    if (ImGui::Button("Reset View")) {
        controls.resetView();
    }

    std::string manipulation_status{"nothing"};
    switch (this->proxy.getManipulationMode()) {
        case ManipulationMode::SELECT:
            manipulation_status = "Mode: Normal selection";
            break;
        case ManipulationMode::TELEPORT_ROBOT:
            manipulation_status = "Mode: Teleport Robot";
            break;
        case ManipulationMode::TELEPORT_BALL:
            manipulation_status = "Mode: Teleport Ball";
            break;
        case ManipulationMode::BALL_SLINGSHOT:
            manipulation_status = "Mode: Ball Slingshot";
            break;
        case ManipulationMode::ADD_POINT:
            manipulation_status = "Mode: Choose point";
            break;
        case ManipulationMode::ADD_DIRECTION:
            manipulation_status = "Mode: Choose direction";
            break;
        case ManipulationMode::EXECUTE_SKILL:
            manipulation_status = "Mode: Execute skill";
            break;
    }
    ImGui::SetCursorPosX(margin_x);
    ImGui::Text(" %s", manipulation_status.c_str());

    ImGui::End();
}

void RenderView::updateMouseControl() {
    bool was_hovered = this->is_hovered;
    this->is_hovered = ImGui::IsWindowHovered();

    // update user movement with mouse
    ImVec2 mouse = ImGui::GetMousePos();
    ImVec2 w_pos = ImVec2(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y);
    ImVec2 w_size =
        ImVec2(ImGui::GetWindowPos().x + ImGui::GetWindowSize().x, ImGui::GetWindowPos().y + ImGui::GetWindowSize().y);
    bool mouse_active =
        (is_hovered || window_size_changed) && this->proxy.getManipulationMode() == ManipulationMode::SELECT;

    vec2 mouse_norm = glm::vec2(-1.0f + 2 * ((mouse.x - w_pos.x) / (w_size.x - w_pos.x)),
                                -1.0f + 2 * ((mouse.y - w_pos.y) / (w_size.y - w_pos.y)));

    // retrieve transformation matrices
    float scroll_value =
        (this->proxy.getManipulationMode() == ManipulationMode::SELECT && is_hovered) ? ImGui::GetIO().MouseWheel : 0;

    controls.computeMatricesFromInputs(window, window_size, mouse_norm, mouse_active, window_size_changed, scroll_value,
                                       is_hovered, this->proxy.getManipulationMode() != ManipulationMode::SELECT);

    glm::mat4 projection_matrix = controls.getProjectionMatrix();
    this->view_matrix = controls.getViewMatrix();
    this->projection_matrix = controls.getProjectionMatrix();
    constexpr float Y_CORRECTION = 0.07f;

    // update interactions
    glm::dvec2 pos = {mouse_norm.x, mouse_norm.y * -1 + Y_CORRECTION};  // invert y, correct mouse absolute position
    glm::dvec3 cam_pos = controls.getCameraPosition();

    // stop here if mouse is not in the renderview window
    if (!is_hovered) return;

    bool ortho_view = !controls.isPerspectiveView();

    switch (this->proxy.getManipulationMode()) {
        case ManipulationMode::SELECT: {
            setCursorImage(nullptr);  // reset cursor

            bool select_related_robot = ImGui::IsKeyDown(ImGuiKey_LeftCtrl);
            bool select_second_skill_robot =
                ImGui::IsKeyDown(ImGuiKey_LeftShift) && this->proxy.getSecondSkillEnabled();
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                // normal select behaviour
                auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);

                marker::MarkerImpl hit_robot{};
                bool hit = Physics::rayRobotIntersection(this->proxy.getRobotMarkers(), cam_pos, ray_world,
                                                         this->robot_collider_radius, this->robot_collider_center,
                                                         hit_robot, ortho_view);
                if (!hit) {
                    // get ground hit position
                    this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);
                } else if (hit_robot.getRobotIdentifier().has_value()) {
                    if (select_related_robot) {
                        // select a related robot
                        this->proxy.updateNextTDRelatedRobot(hit_robot.getRobotIdentifier().value());
                    } else if (select_second_skill_robot) {
                        // robot selection for second skill
                        if (this->proxy.getSelectedRobot2().has_value() &&
                            this->proxy.getSelectedRobot2().value() == hit_robot.getRobotIdentifier().value()) {
                            // deselect same robot
                            this->proxy.getSelectedRobot2().reset();
                        } else {
                            // just select the robot
                            this->proxy.getSelectedRobot2() = hit_robot.getRobotIdentifier().value();
                        }
                    } else {
                        // normal robot selection
                        if (this->proxy.getSelectedRobot().has_value() &&
                            this->proxy.getSelectedRobot().value() == hit_robot.getRobotIdentifier().value()) {
                            // deselect same robot
                            this->proxy.getSelectedRobot().reset();
                        } else {
                            // just select the robot
                            this->proxy.getSelectedRobot() = hit_robot.getRobotIdentifier().value();
                        }
                    }
                }
            }
            break;
        }
        case ManipulationMode::TELEPORT_BALL: {
            setCursorImage(&this->teleport_cursor);
            auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
            this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);

            if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && was_hovered) {
                // save the slingshot start point
                auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
                this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);
                this->start_ball_pos = proxy.getCurrentBallPos();
                this->proxy.getManipulationMode() = ManipulationMode::BALL_SLINGSHOT;
            } else if (this->ground_hit_position.has_value()) {
                // teleport ball
                Eigen::Affine2d target =
                    Eigen::Translation2d{ground_hit_position.value().x, -ground_hit_position.value().z} *
                    Eigen::Rotation2Dd{0};
                this->proxy.teleportBall(target);
            }

            break;
        }
        case ManipulationMode::BALL_SLINGSHOT: {
            // apply velocity to ball in defined direction, just like in angry birds or similar
            setCursorImage(&this->ballslingshot_cursor);

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
                // apply ball velocity
                if (this->start_ball_pos.has_value() && this->slingshot_velocity.has_value()) {
                    proxy.teleportBall(this->start_ball_pos.value(), this->slingshot_velocity.value());
                    this->ground_hit_position = std::nullopt;
                    this->slingshot_velocity = std::nullopt;
                }
                // change back to selection modus
                this->mousewheel_rotation.reset();
                this->ground_hit_position = std::nullopt;
                this->proxy.getManipulationMode() = ManipulationMode::SELECT;
            } else if (ground_hit_position.has_value()) {
                // if dragging, recalculate velocity vector
                auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
                glm::vec3 cursor_pos = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view).value();
                glm::vec3 vel = cursor_pos - this->ground_hit_position.value();
                if (glm::length(vel) < 0.2) {
                    this->slingshot_velocity = std::nullopt;
                    break;  // ignore small mouse movement
                }

                const double slingshot_speed = 5.0;
                vel *= slingshot_speed;
                // cap velocity at max allowed ball velocity
                const double max_ball_velocity = 6.5;  // max allowed ball speed = 6.5 m/s
                if (glm::length(vel) > max_ball_velocity) {
                    vel = glm::normalize(vel) * glm::vec3{max_ball_velocity, max_ball_velocity, max_ball_velocity};
                }

                this->slingshot_velocity = {-vel.x, vel.z, 0};  // (x,-z) inversed
            }
            break;
        }
        case ManipulationMode::TELEPORT_ROBOT: {
            setCursorImage(&this->teleport_cursor);

            auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
            this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && was_hovered) {
                // change back to selection modus
                this->mousewheel_rotation.reset();
                this->ground_hit_position = std::nullopt;
                this->proxy.getManipulationMode() = ManipulationMode::SELECT;
            } else if (this->ground_hit_position.has_value()) {
                if (this->proxy.getSelectedRobot().has_value()) {
                    // teleport selected robot
                    if (!this->mousewheel_rotation.has_value() && this->proxy.getSelectedRobot().has_value()) {
                        const auto& sel_robot =
                            this->proxy.getRobotMarkers()[this->proxy.getSelectedRobot().value().id];
                        this->mousewheel_rotation = sel_robot.getRotation2D() - L_PI / 2;
                    }

                    // per scroll tick we want 1 / 2PI rotation steps
                    const double rotation_mult = 1.0 / 2 * L_PI;
                    double scroll_val = mousewheel_rotation.value() + ImGui::GetIO().MouseWheel * rotation_mult;
                    // apply smooth rotation
                    this->mousewheel_rotation = this->mousewheel_rotation.value() * 0.9 + scroll_val * 0.1;

                    // selected robot follows mouse world pos
                    Eigen::Affine2d target =
                        Eigen::Translation2d{ground_hit_position.value().x, -ground_hit_position.value().z} *
                        Eigen::Rotation2Dd{this->mousewheel_rotation.value()};
                    TeamColor team_color =
                        this->proxy.getSelectedRobot().value().isAlly() ? TeamColor::BLUE : TeamColor::YELLOW;

                    this->proxy.teleportRobot(this->proxy.getSelectedRobot().value().id, team_color, target,
                                              this->proxy.isRobotPresent());
                } else {
                    LOG_WARNING(logger, "Either select a robot or the ball to use the teleport functionality");
                    this->ground_hit_position = std::nullopt;
                    this->proxy.getManipulationMode() = ManipulationMode::SELECT;
                }
            }
            break;
        }
        case ManipulationMode::ADD_POINT: {
            setCursorImage(&this->setpoint_cursor);

            // check if point is needed, if not go to execute skill
            if (this->proxy.getRemainingPointsToChoose() == 0 &&
                (this->proxy.getRemainingPointsToChoose2() == 0 || !this->proxy.getSecondSkillEnabled())) {
                this->ground_hit_position = std::nullopt;
                this->proxy.getManipulationMode() = ManipulationMode::EXECUTE_SKILL;
                break;
            }

            // visual select point on mouse down
            if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
                auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
                this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);
                this->skillpoint_position = {this->ground_hit_position.value().x, this->ground_hit_position.value().z};
                this->last_skill_points.emplace_back(this->skillpoint_position.value());
                this->proxy.getManipulationMode() = ManipulationMode::ADD_DIRECTION;
            }

            // remove last set point
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                this->proxy.removeNewestTDPoint();
            }

            break;
        }
        case ManipulationMode::ADD_DIRECTION: {
            setCursorImage(&this->setdir_cursor);

            if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) && this->skillpoint_position.has_value()) {
                // apply selected point
                glm::vec2 pos{this->skillpoint_position.value().x, -this->skillpoint_position.value().y};
                if (this->proxy.getRemainingPointsToChoose() == 0) {
                    this->proxy.addTDPoint2({pos.x, pos.y, this->angle});
                } else {
                    this->proxy.addTDPoint({pos.x, pos.y, this->angle});
                }
                // add another point or execute skill
                this->skillpoint_position = std::nullopt;
                this->skillpoint_direction = std::nullopt;
                this->ground_hit_position = std::nullopt;
                this->proxy.getManipulationMode() = ManipulationMode::ADD_POINT;
            } else {
                // can drag to give a orientation
                // release mouse to apply point with optional direction, else direction = (0,0,0)
                auto ray_world = Physics::raycast(pos, view_matrix, projection_matrix, ortho_view);
                this->ground_hit_position = Physics::rayGroundIntersection(ray_world, cam_pos, ortho_view);
                glm::vec2 direction =
                    glm::vec2{this->ground_hit_position.value().x, this->ground_hit_position.value().z} -
                    this->skillpoint_position.value();
                const float enable_direction_len = 0.1f;
                if (glm::length(direction) >= enable_direction_len) {
                    this->skillpoint_direction = glm::normalize(direction);

                    // convert optional direction to angle
                    glm::vec2 dir{this->skillpoint_direction.value().x, this->skillpoint_direction.value().y};
                    if (dir.y >= 0) {
                        this->angle = -glm::angle(dir, glm::vec2{1, 0});
                    } else {
                        this->angle = glm::angle(dir, glm::vec2{1, 0});
                    }
                } else {
                    this->skillpoint_direction = std::nullopt;
                    this->angle = 0;
                }
            }
            break;
        }
        case ManipulationMode::EXECUTE_SKILL: {
            setCursorImage(&this->execskill_cursor);

            // execute the skill on mouse click with the given parameters
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
                // check if all required parameters are set
                bool success = this->proxy.sendSkillsToRobots();
                this->proxy.getManipulationMode() = ManipulationMode::SELECT;
                if (!success) {
                    LOG_WARNING(logger, "Send Skill failed. Check if parameters are correct!");
                } else {
                    this->last_skill_points.clear();
                }
            }

            // remove last set point
            if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
                this->proxy.removeNewestTDPoint();
            }

            break;
        }
    }
}

void RenderView::saveAndCleanup() {
    // save params
    this->proxy.setConfigInt("window_width", static_cast<int>(this->window_size.x));
    this->proxy.setConfigInt("window_height", static_cast<int>(this->window_size.y));
    controls.saveParams(this->proxy);

    // delete to texture rendering buffers
    cleanMSAABuffer();
}

void RenderView::setCursorImage(GLTexture* cursor_image) {
    if (cursor_image != nullptr) {
        constexpr int HALF_CURSOR_SIZE = 15;

        // disable default cursor image
        ImGui::SetMouseCursor(ImGuiMouseCursor_None);

        // draw custom image at mouse position
        ImVec2 p_min = {ImGui::GetMousePos().x - HALF_CURSOR_SIZE, ImGui::GetMousePos().y + HALF_CURSOR_SIZE};
        ImVec2 p_max = {ImGui::GetMousePos().x + HALF_CURSOR_SIZE, ImGui::GetMousePos().y - HALF_CURSOR_SIZE};
        if (cursor_image->getId() == this->teleport_cursor.getId() ||
            cursor_image->getId() == this->setpoint_cursor.getId()) {
            // adapt position to match raycast position
            p_min = {ImGui::GetMousePos().x - HALF_CURSOR_SIZE, ImGui::GetMousePos().y};
            p_max = {ImGui::GetMousePos().x + HALF_CURSOR_SIZE, ImGui::GetMousePos().y - 2 * HALF_CURSOR_SIZE};
        }
        ImGui::GetForegroundDrawList()->AddImage(cursor_image->getImguiId(), p_min, p_max);
    } else {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
    }
}

mat4 RenderView::getModelMatrix(const dvec3& pos, const dquat& rot, const dvec3& scale) {
    glm::mat4 translate = glm::translate(glm::mat4(1.0), {pos});
    glm::mat4 rotate = glm::mat4_cast(rot);
    glm::mat4 scaling = glm::scale(glm::mat4(1.0), {scale});

    return translate * rotate * scaling;
}

}  // namespace luhsoccer::luhviz