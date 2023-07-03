#pragma once

#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#include "common/include/controls.hpp"
#include "include/data_proxy.hpp"
#include "marker_service/marker_service.hpp"
#include "new_rendering/include/gl_shader_program.hpp"
#include "new_rendering/include/gl_uniform.hpp"
#include "new_rendering/include/gl_context.hpp"
#include "render_view/include/mesh.hpp"
#include "render_view/include/model.hpp"
#include "common/include/physics.hpp"
#include <glm/gtx/vector_angle.hpp>

namespace luhsoccer::luhviz {

class RenderView {
   public:
    /**
     * @brief Construct a new Render View object
     *
     * @param w opengl context window
     * @param ms reference to global marker service module
     */
    RenderView(marker::MarkerService& ms, luhviz::DataProxy& p, std::unique_ptr<GLContext>& context)
        : context(context),
          marker_service(ms),
          proxy(p),
          controls(p.getConfigDouble(luhviz_internal_config_name, "camera_angle_x"),
                   p.getConfigDouble(luhviz_internal_config_name, "camera_angle_y"),
                   p.getConfigDouble(luhviz_internal_config_name, "camera_zoom"),
                   p.getConfigBool(luhviz_internal_config_name, "perspective_view")),
          msaa_sample_count(static_cast<int>(
              std::pow(2, static_cast<float>(p.getConfigInt(luhviz_config_name, "antialiasing_quality"))))),
          change_view_button_text(this->controls.isPerspectiveView() ? "3D View" : "Top View") {
        this->window_size = glm::vec2{static_cast<float>(p.getConfigInt(luhviz_internal_config_name, "window_width")),
                                      static_cast<float>(p.getConfigInt(luhviz_internal_config_name, "window_height"))};
    }

    /**
     * @brief initialises the opengl related contents like opengl settings, loading 3d models, compiling shaders
     *
     */
    void init(GLFWwindow* w);

    /**
     * @brief get the newest markers from the markerservice module and prepare them for displaying
     *
     */
    void update(marker::LuhvizMarkers& luhviz_markers);

    /**
     * @brief renders the current markers to an MSAA multisampled (anti aliasing) offscreen image
     *
     */
    void render();

    /**
     * @brief renders the offscreen image from render() into an image component of dearIMGui, updates the mouse
     input
     * control
     *
     */
    void renderToTexture();

    /**
     * @brief cleans up all allocated memory on program quit
     *
     */
    void saveAndCleanup();

   private:
    const std::string luhviz_config_name = "luhviz";
    const std::string luhviz_internal_config_name = "luhviz_internal";

    // path of the ressources folder
    const std::string res_path = "res/";
    // paths of the different shaders
    const std::string ver_shader = res_path + "shader/vertexShader.glsl";
    const std::string frag_shader = res_path + "shader/fragmentShader.glsl";
    // paths of the different .obj files
    const std::string goalborders_obj = res_path + "obj/goalborders.obj";
    const std::string frame_obj = res_path + "obj/frame.obj";
    const std::string frame_texture = res_path + "obj/frame.bmp";
    const std::string robot_obj = res_path + "obj/soccerrobot.obj";
    const std::string ball_obj = res_path + "obj/ball.obj";
    const std::string cone_obj = res_path + "obj/cone.obj";
    const std::string cube_obj = res_path + "obj/cube.obj";
    const std::string cylinder_obj = res_path + "obj/cylinder.obj";
    const std::string sphere_obj = res_path + "obj/sphere.obj";
    const std::string torus_obj = res_path + "obj/torus.obj";
    const std::string arrow_obj = res_path + "obj/arrow.obj";
    const std::string suzanne_obj = res_path + "obj/suzanne.obj";
    const std::string plane_obj = res_path + "obj/plane.obj";
    const std::string text_texture = res_path + "msdf/";
    // custom cursor images paths
    const std::string p_teleport = res_path + "images/cursor_icons/teleport_cursor.png";
    const std::string p_setpoint = res_path + "images/cursor_icons/setpoint_cursor.png";
    const std::string p_setdir = res_path + "images/cursor_icons/setdir_cursor.png";
    const std::string p_execskill = res_path + "images/cursor_icons/execskill_cursor.png";
    const std::string p_slingshot = res_path + "images/cursor_icons/ballslingshot_cursor.png";

    // default position of the light
    constexpr static glm::vec3 DEFAULT_LIGHT_POS = glm::dvec3(0, 20, 0);
    glm::dvec3 light_pos{DEFAULT_LIGHT_POS};
    // default size of the window
    const glm::vec2 default_window_size = glm::vec2(1280, 640);

    luhsoccer::logger::Logger logger{"luhviz/render_view"};

    // ----- members -----
    std::unique_ptr<GLContext>& context;
    // Shader
    GLShaderProgram shader_program;

    // Uniforms
    GLUniformMat4vf mvp_id;
    GLUniformMat4vf mv_id;
    GLUniformMat4vf m_id;
    GLUniformMat4vf v_id;
    GLUniform3f light_id;
    GLUniform1i tex_id;
    GLUniform4f color_id;
    GLUniform1i multicolor_id;
    GLUniform1i type_id;

    std::shared_ptr<GLTexture> frame_gltexture{nullptr};

    GLuint intermediate_fbo{};
    GLuint rbo{};
    GLuint framebuffer{};
    GLuint texture_color_buffer_multisampled{};
    GLuint screen_texture{};
    glm::dvec2 viewport{};
    glm::mat4 view_matrix{};
    glm::mat4 projection_matrix{};

    marker::MarkerService& marker_service;
    GLFWwindow* window{nullptr};
    luhviz::DataProxy& proxy;
    Controls controls;
    glm::vec2 window_size = default_window_size;
    bool window_size_changed = false;
    const int msaa_sample_count;  // set this to 1,2,4 or 8 (higher means better antialiasing)

    std::string change_view_button_text = "3D View";
    std::optional<glm::vec3> ground_hit_position{std::nullopt};
    std::optional<glm::vec2> skillpoint_position{std::nullopt};
    std::vector<glm::vec2> last_skill_points{};
    std::optional<glm::vec2> skillpoint_direction{std::nullopt};
    std::optional<Eigen::Vector3d> slingshot_velocity{std::nullopt};
    std::optional<Eigen::Affine2d> start_ball_pos{std::nullopt};
    double angle{};

    bool is_hovered = false;

    // mouse cursor image ids
    GLTexture teleport_cursor;
    GLTexture setpoint_cursor;
    GLTexture setdir_cursor;
    GLTexture execskill_cursor;
    GLTexture ballslingshot_cursor;

    // robot selection
    const double robot_collider_radius = 0.12;
    const glm::dvec3 robot_collider_center{0, 0.08, 0};
    std::optional<double> mousewheel_rotation{std::nullopt};

    // meshTypes
    std::unordered_map<marker::Type3D, Model> models;
    std::unordered_map<std::string, std::shared_ptr<Mesh>> meshes;
    std::unordered_map<std::string, std::shared_ptr<TextMesh>> texts{};
    std::vector<std::shared_ptr<Mesh>> render_meshes;
    std::vector<std::shared_ptr<Mesh>> transparent_meshes{};
    std::vector<std::shared_ptr<TextMesh>> render_texts{};

    // ----- methods -----
    /**
     * @brief standard method to load a 3d mesh onto the gpu
     *
     * @param texture_path
     * @param model_path
     * @param shader_program
     * @return Mesh
     */
    Mesh loadMesh(const std::string& texture_path, const std::string& model_path, const GLuint shader_program);

    /**
     * @brief loads a colored 3d mesh onto the gpu ready for rendering
     *
     * @param model_path
     * @param shader_program
     * @return Mesh
     */
    Mesh loadColoredMesh(const std::string& model_path, const GLuint shader_program);

    /**
     * @brief loads a textured 3d mesh onto the gpu ready for rendering
     *
     * @param texture_path
     * @param model_path
     * @param shader_program
     * @return Mesh
     */
    Mesh loadTexturedMesh(const std::string& texture_path, const std::string& model_path, const GLuint shader_program);

    /**
     * @brief generates textures used for the msaa offscreen rendering and setups opengl with them
     *
     */
    void setupMSAAOffscreenRendering();

    /**
     * @brief clean the allocated memory from multisampling rendering to offscreen texture
     *
     */
    void cleanMSAABuffer();

    /**
     * @brief update the mouse input for camera movement and rotation
     *
     */
    void updateMouseControl();

    /**
     * @brief create a Transformation matrix from position, rotation and scale components
     *
     * @param pos
     * @param rot
     * @param scale
     * @param is_text
     * @return glm::mat4
     */
    glm::mat4 getModelMatrix(const glm::dvec3& pos, const glm::dquat& rot, const glm::dvec3& scale);

    /**
     * @brief loads custom images for mouse cursor
     *
     */
    void loadMouseCursorImages();

    /**
     * @brief Set the Cursor Image object
     *
     * @param cursor_image
     */
    void setCursorImage(GLTexture* cursor_image);

    /**
     * @brief Get the Selected Robot Id object
     *
     * @return size_t
     */
    std::optional<size_t> getSelectedRobotId() {
        if (this->proxy.getSelectedRobot().has_value()) {
            return this->proxy.getSelectedRobot().value().id;
        }
        return std::nullopt;
    }
};
}  // namespace luhsoccer::luhviz
